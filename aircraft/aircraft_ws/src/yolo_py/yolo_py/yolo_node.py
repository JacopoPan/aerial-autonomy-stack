import rclpy
from rclpy.node import Node
import cv2
import json
import yaml
import numpy as np
import onnxruntime as ort
import argparse
import os
import matplotlib.pyplot as plt
import threading
import queue
import time
import math
import platform

from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesis, ObjectHypothesisWithPose
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


CONF_THRESH = 0.5

class YoloInferenceNode(Node):
    def __init__(self, camera_id, headless, hitl, remote_video_streams, hfov, ros2_frame_publisher, no_inference):
        super().__init__('yolo_inference_node')
        self.camera_id = camera_id
        self.headless = headless
        self.hitl = hitl
        self.remote_video_streams = remote_video_streams
        self.hfov = hfov
        self.ros2_frame_publisher = ros2_frame_publisher
        self.run_inference = not no_inference # # Invert flag for readability

        self.udp_port = 5600 + self.camera_id  # 0 -> 5600, 1 -> 5601
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.architecture = platform.machine()
        self.is_jetson = (self.architecture == 'aarch64')
        
        if self.run_inference:
            # Load classes
            names_file = "/aas/yolo/coco.json"
            with open(names_file, "r") as f:
                self.classes = {int(k): v for k, v in json.load(f).items()}
            colors_rgba = plt.cm.hsv(np.linspace(0, 1, len(self.classes)))
            self.colors = (colors_rgba[:, [2, 1, 0]] * 255).astype(np.uint8) # From RGBA (0-1 float) to BGR (0-255 int)
            # Pre-allocate reusable arrays for scaling to avoid allocation in hot loops
            self.scale_factors = np.zeros(4, dtype=np.float32)

        # Defer model loading until the video stream is opened in run_inference_loop
        self.input_size = None
        self.session = None
        self.input_name = None
        
        # Create publishers
        if self.run_inference:
            self.detection_publisher = self.create_publisher(Detection2DArray, 'detections', 10)
        if self.ros2_frame_publisher:
            self.image_publisher = self.create_publisher(Image, f'camera_frames_{self.camera_id}', 10)
        self.bridge = CvBridge()
        
        self.get_logger().info("YOLO inference started.")

    def run_inference_loop(self):
        # Acquire video stream
        if self.architecture == 'x86_64':
            # # GPU pipeline: TODO NOT WORKING
            # gst_pipeline_string = (
            #     f"udpsrc port={self.udp_port} ! "
            #     "application/x-rtp, media=(string)video, encoding-name=(string)H264 ! "
            #     "rtph264depay ! "
            #     "h264parse ! "
            #     "nvh264dec ! "
            #     "nvvidconv ! "  # Use NVIDIA's GPU-accelerated converter
            #     "video/x-raw(memory:NVMM), format=BGRx ! "
            #     "videoconvert ! "
            #     "video/x-raw, format=BGR ! appsink"
            # )
            # CPU pipeline
            gst_pipeline_string = (
                f"udpsrc port={self.udp_port} ! "
                "application/x-rtp, media=(string)video, encoding-name=(string)H264 ! "
                "rtph264depay ! "
                "avdec_h264 ! " # Use CPU decoder
                "videoconvert ! "
                "video/x-raw, format=BGR ! appsink"
            )
            cap = cv2.VideoCapture(gst_pipeline_string, cv2.CAP_GSTREAMER)
        elif self.architecture == 'aarch64':
            if self.hitl: # For HITL, acquire UDP stream from gz-sim
                # GPU pipeline:
                gst_pipeline_string = (
                    f"udpsrc port={self.udp_port} ! "
                    "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264 ! "
                    "rtpjitterbuffer latency=100 drop-on-latency=true ! " # Handle network jitter while adding latency (in ms)
                    "rtph264depay ! "
                    "h264parse config-interval=-1 ! "
                    "nvv4l2decoder enable-max-performance=1 ! "     # Hardware Decoding: Uses the Orin's dedicated engine
                    "nvvidconv ! "         # NVMM-to-CPU Memory Conversion
                    "video/x-raw, format=I420 ! " # If needed, add "queue max-size-buffers=2 leaky=downstream ! " to mitigate EGL crashes
                    "videoconvert ! "      # CPU Color Conversion: I420 to BGR
                    "video/x-raw, format=BGR ! "
                    "appsink drop=true max-buffers=1 sync=false "
                )
                # # CPU pipeline:
                # gst_pipeline_string = (
                #     "udpsrc port=5600 ! "
                #     "application/x-rtp, media=(string)video, encoding-name=(string)H264 ! "
                #     "rtph264depay ! "
                #     "avdec_h264 ! "      # Generic CPU H.264 decoder
                #     "videoconvert ! "
                #     "video/x-raw, format=BGR ! appsink"
                # )
                cap = cv2.VideoCapture(gst_pipeline_string, cv2.CAP_GSTREAMER)
            else: # Default, acquire CSI camera 
                # GPU pipeline:
                gst_pipeline_string = (
                    f"nvarguscamerasrc sensor-id={self.camera_id} ! "
                    "video/x-raw(memory:NVMM), width=1280, height=720, framerate=60/1 ! "
                    "nvvidconv ! "
                    "video/x-raw(memory:NVMM), format=RGBA ! "
                    "nvdewarper config-file=/aas/aircraft_resources/patches/imx219_dewarper_config.txt ! "
                    "nvvidconv ! "
                    "video/x-raw, width=640, height=360, format=BGRx ! "
                    "queue max-size-buffers=2 leaky=downstream ! " # Mitigate EGL crashes
                    "videoconvert ! video/x-raw, format=BGR ! "
                    "appsink drop=true max-buffers=1 sync=false"
                ) # Test with: gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! 'video/x-raw(memory:NVMM), width=1280, height=720, framerate=60/1' ! nvvidconv ! nv3dsink -e
                # GPU pipeline with nvinfer example:
                # gst_pipeline_string = (
                #     "nvarguscamerasrc sensor-id=0 ! "
                #     "video/x-raw(memory:NVMM), width=1280, height=720, framerate=60/1 ! "
                #     "nvvidconv ! "
                #     "video/x-raw(memory:NVMM), format=RGBA ! "
                #     "nvdewarper config-file=/aas/aircraft_resources/patches/imx219_dewarper_config.txt ! "
                #     "nvvideoconvert ! video/x-raw(memory:NVMM), format=NV12 ! "
                #     "m.sink_0 nvstreammux name=m batch-size=1 width=640 height=360 ! "
                #     "nvinfer config-file-path=/aas/aircraft_resources/patches/nvinfer_config.txt ! "
                #     "nvvideoconvert ! video/x-raw(memory:NVMM), format=RGBA ! "
                #     "nvdsosd ! "
                #     "nvvideoconvert ! video/x-raw, format=BGRx ! "
                #     "videoconvert ! video/x-raw, format=BGR ! "
                #     "appsink drop=true max-buffers=1 sync=false"
                # )
                cap = cv2.VideoCapture(gst_pipeline_string, cv2.CAP_GSTREAMER)
        # cap = cv2.VideoCapture("/sample.mp4") # Load sample video for testing
        if not cap.isOpened():
            raise RuntimeError("Failed to open video stream")
        print(f"Pipeline FPS: {cap.get(cv2.CAP_PROP_FPS)}")
        stream_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        stream_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Stream Resolution: {stream_width}x{stream_height}")

        # Intrinsics Setup
        if self.architecture == 'aarch64' and not self.hitl: # IMX219-200 CSI camera undistorted with nvdewarper
            calib_file = "/aas/aircraft_resources/patches/camera_calibration.yaml"
            if not os.path.exists(calib_file):
                raise FileNotFoundError(f"CRITICAL: Camera calibration file not found at {calib_file}.")
            with open(calib_file, 'r') as f:
                calib = yaml.safe_load(f)
                self.fx = calib['fx']
                self.fy = calib['fy']
                self.cx = calib['cx']
                self.cy = calib['cy']
            print(f"Loaded IMX219 intrinsics from {calib_file}")
        else: # Pinhole assumption in simulation
            self.cx = stream_width / 2.0
            self.cy = stream_height / 2.0
            clipped_hfov = min(self.hfov, 175.0) # Clip for pinhole approximation
            self.fx = self.cx / math.tan(math.radians(clipped_hfov) / 2.0)
            self.fy = self.fx
            print(f"Computed simulated camera intrinsics using HFOV {self.hfov}deg")
        hfov = math.degrees(2 * math.atan(self.cx / self.fx))
        vfov = math.degrees(2 * math.atan(self.cy / self.fy))
        dfov = math.degrees(2 * math.atan(math.sqrt(self.cx**2 + self.cy**2) / self.fx))
        print(f"Intrinsics: cx={self.cx:.2f}, cy={self.cy:.2f}, fx={self.fx:.2f}, fy={self.fy:.2f}")
        print(f"DFOV {dfov:.2f}deg, HFOV {hfov:.2f}deg, VFOV {vfov:.2f}deg")

        if self.run_inference:
            # Load YOLO model and runtime
            # Options, from fastest to most accurate, <10MB to >100MB: yolo26n, yolo26s, yolo26m, yolo26l, yolo26x, export in aircraft.dockerfile
            if self.architecture == 'x86_64':
                max_dim = max(stream_width, stream_height)
                if max_dim <= 320:
                    print("Stream resolution is <=320. Selecting 320 model.")
                    model_path = "/aas/yolo/yolo26n_320.onnx"
                    self.input_size = 320 # YOLO input size
                else:
                    print("Stream resolution is >320. Selecting 640 model.")
                    model_path = "/aas/yolo/yolo26n_640.onnx"
                    self.input_size = 640 # YOLO input size
                print("Loading CUDAExecutionProvider on AMD64 (x86) architecture.")
                self.session = ort.InferenceSession(model_path, providers=["CUDAExecutionProvider"]) # For simulation
            elif self.architecture == 'aarch64':
                model_path = "/aas/yolo/yolo26n_640.onnx" # Real CSI camera IMX219-200 is 1280x720, we resize to 640x640 for YOLO (this is slightly wasteful when self.hitl = True)
                self.input_size = 640 # YOLO input size
                print("Loading (with cache) TensorrtExecutionProvider on ARM64 architecture (Jetson).") # The first cache built takes ~10'
                cache_path = "/tensorrt_cache" # Mounted as volume by main_deploy.sh
                os.makedirs(cache_path, exist_ok=True)
                provider_options = {
                    'trt_engine_cache_enable': True,
                    'trt_engine_cache_path': cache_path,
                    'trt_fp16_enable': True, # Optional: enable FP16 for Jetson speedup (from 22 to 12ms on YOLOn, longer cache build time, 10 vs 3')
                }
                self.session = ort.InferenceSession(
                    model_path,
                    providers=[('TensorrtExecutionProvider', provider_options)] # For deployment on Jetson Orin, 60Hz inference on the IMX219-200
                )
            else:
                print(f"Loading CPUExecutionProvider on an unknown architecture: {self.architecture}")
                self.session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"]) # Backup, not recommended
            self.input_name = self.session.get_inputs()[0].name
            # Confirm execution providers
            self.get_logger().info(f"Execution providers in use: {self.session.get_providers()}")
        else:
            self.get_logger().info("Inference disabled")

        if not self.headless:
            drone_id = os.getenv('DRONE_ID', '1')
            self.WINDOW_NAME = f"YOLO (Aircraft {drone_id} - Cam {self.camera_id})"
            cv2.namedWindow(self.WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.moveWindow(self.WINDOW_NAME, 800+(int(drone_id)-1)*25 + (self.camera_id*400), 5+(int(drone_id)-1)*200)
            # cv2.resizeWindow(self.WINDOW_NAME, 400, 200)

        # Start the video capture thread
        is_running = threading.Event()
        is_running.set()
        frame_queue = queue.Queue(maxsize=1) # A queue to hold frames, reduce maxsize to reduce latency (buffer bloat)
        frame_thread = threading.Thread(target=self.frame_capture_thread, args=(cap, frame_queue, is_running), daemon=True)
        frame_thread.start()

        # ROS spinning thread
        ros_thread = threading.Thread(target=self.ros_spin_thread, daemon=True)
        ros_thread.start()

        while rclpy.ok():
            try:
                frame = frame_queue.get(timeout=1.0) # Get the most recent frame from the queue
            except queue.Empty:
                self.get_logger().info("Frame queue is empty, is the stream running?")
                continue
            if frame is None or frame.size == 0:
                self.get_logger().warn("Empty frame received. Skipping...")
                continue

            # Publish raw frames in ROS
            if self.ros2_frame_publisher:
                msg_header = Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id=f"camera_frame_{self.camera_id}"
                )
                self.image_publisher.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8", header=msg_header))
            
            if self.run_inference:
                # Inference
                with Profiler("do_yolo (includes ONNX Runtime)"):
                    boxes, confidences, class_ids = self.do_yolo(frame)

                # Publish detections
                if len(boxes) > 0:
                    self.publish_detections(boxes, confidences, class_ids)

                # Draw detections on the frame
                if len(boxes) > 0:
                    self.draw_detections(frame, boxes, confidences, class_ids)

            # Visualize frame with detections
            if not self.headless:
                cv2.imshow(self.WINDOW_NAME, frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            # Only on Jetson, stream to the ground station via UDP using GStreamer
            # On ports 5001, 5002, ..., and 5101, 5102, ..., based on DRONE_ID and self.camera_id
            if self.remote_video_streams and self.is_jetson: # Use cached boolean flag in loops
                if not hasattr(self, 'gnd_stream_writer'):
                    h, w = frame.shape[:2]
                    gnd_ip = os.getenv('AIR_SUBNET', '10.22') + '.90.' + os.getenv('GROUND_ID', '101')
                    port = 5000 + int(os.getenv('DRONE_ID', '1')) + (self.camera_id * 100)
                    gst_out = (
                        "appsrc do-timestamp=true ! video/x-raw, format=BGR ! queue max-size-buffers=2 leaky=downstream ! "
                        "videoconvert ! videorate drop-only=true ! "
                        "video/x-raw, format=BGRx, max-framerate=10/1 ! nvvidconv ! "
                        "nvv4l2h265enc maxperf-enable=1 preset-level=3 "
                        "control-rate=0 bitrate=800000 peak-bitrate=1200000 " # Variable bitrate, or use "control-rate=1 bitrate=1000000 " for constant bitrate
                        "insert-sps-pps=true idrinterval=20 ! "
                        f"h265parse ! rtph265pay pt=96 config-interval=1 mtu=1400 ! udpsink host={gnd_ip} port={port} sync=false async=false"
                    )
                    # Cap the framerate to 10FPS and use h265 to reduce bandwidith
                    # Optionally, re-scale the frames "nvvidconv ! video/x-raw, width=640, height=480 ! "
                    self.gnd_stream_writer = cv2.VideoWriter(gst_out, cv2.CAP_GSTREAMER, 0, 60.0, (w, h)) # Framerate upper limit of 60FPS
                    self.get_logger().info(f"Started UDP stream to {gnd_ip}:{port}")
                if self.gnd_stream_writer.isOpened():
                    ros_time_sec = self.get_clock().now().nanoseconds / 1e9
                    cv2.putText(frame, f"ROS T: {ros_time_sec:.3f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 1) # Timestamp frames
                    self.gnd_stream_writer.write(frame)

        # Cleanup
        is_running.clear()
        frame_thread.join()

        if hasattr(self, 'gnd_stream_writer') and self.gnd_stream_writer.isOpened():
            self.gnd_stream_writer.release()

        cap.release()
        if not self.headless:
            cv2.destroyAllWindows()

    def ros_spin_thread(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001) # This is only to get the simulation time from /clock

    def frame_capture_thread(self, cap, frame_queue, is_running):
        try:
            os.nice(-10)
        except:
            pass

        while is_running.is_set():
            with Profiler("cap.read()"):
                ret, frame = cap.read()

            if not ret:
                time.sleep(0.01) # Avoid busy loop if no frame is received
                continue
            if self.is_jetson: # Use cached boolean flag in loops
                frame = frame.copy() # On Jetson, avoid GStreamer and inference memory conflicts (at small CPU cost)
            try:
                if frame_queue.full():
                    try:
                        frame_queue.get_nowait()
                    except queue.Empty:
                        pass
                frame_queue.put_nowait(frame)
            except queue.Full:
                pass # Drop frame if the main thread is lagging

    def do_yolo(self, frame):
        # if frame.shape[2] == 4: # Handle 4-channel input (BGRx) to avoid "videoconvert ! video/x-raw, format=BGR ! "
        #     frame = frame[:, :, :3]
        h0, w0 = frame.shape[:2]

        img = cv2.dnn.blobFromImage(frame, 1/255.0, (self.input_size, self.input_size), swapRB=True, crop=False)
        # Consider letterboxed preprocessing instead of `blobFromImage`'s stretch-resize if aspect-ratio distortion hurts detection

        with Profiler("ONNX Runtime Inference"):
            outputs = self.session.run(None, {self.input_name: img})

        # YOLO26 is NMS-free https://docs.ultralytics.com/models/yolo26/#what-are-the-key-improvements-in-yolo26-compared-to-yolo11
        # Output shape is [1, max_det, 6] -> [batch, detections, [x1, y1, x2, y2, conf, class_id]]
        preds = outputs[0][0]
        boxes = preds[:, :4] # x1, y1, x2, y2
        confidences = preds[:, 4]
        class_ids = preds[:, 5].astype(int)

        # Filter by confidence threshold
        mask = confidences > CONF_THRESH

        if not mask.any():
            return np.array([]), np.array([]), np.array([])

        # Apply mask
        boxes = boxes[mask]
        confidences = confidences[mask]
        class_ids = class_ids[mask]

        # Convert [x1, y1, x2, y2] to [cx, cy, w, h]
        w = boxes[:, 2] - boxes[:, 0]
        h = boxes[:, 3] - boxes[:, 1]
        cx = boxes[:, 0] + (w * 0.5)
        cy = boxes[:, 1] + (h * 0.5)
        boxes = np.column_stack((cx, cy, w, h)).astype(np.float32)

        # Apply scaling factors to match original frame size
        self.scale_factors[:] = [w0 / self.input_size, h0 / self.input_size, w0 / self.input_size, h0 / self.input_size]
        boxes *= self.scale_factors

        return boxes, confidences, class_ids

    def publish_detections(self, boxes, confidences, class_ids):
        center_x = boxes[:, 0]
        center_y = boxes[:, 1]
        widths   = boxes[:, 2]
        heights  = boxes[:, 3]

        dx = center_x - self.cx
        dy = self.cy - center_y
        # Pinhole approximation
        azimuths = np.degrees(np.arctan(dx / self.fx))
        elevations = np.degrees(np.arctan(dy / self.fy))

        # Construct Message
        detection_array = Detection2DArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = f"camera_frame_{self.camera_id}"

        for i in range(len(boxes)):
            bbox = BoundingBox2D()
            bbox.center.position.x = float(center_x[i])
            bbox.center.position.y = float(center_y[i])
            bbox.size_x = float(widths[i])
            bbox.size_y = float(heights[i])

            hypothesis = ObjectHypothesis()
            hypothesis.class_id = str(self.classes[class_ids[i]])
            hypothesis.score = float(confidences[i])
            
            result = ObjectHypothesisWithPose()
            result.hypothesis = hypothesis
            result.pose.pose.position.x = float(azimuths[i]) # degrees
            result.pose.pose.position.y = float(elevations[i]) # degrees

            detection = Detection2D()
            detection.bbox = bbox
            detection.id = hypothesis.class_id
            detection.results.append(result)
            
            detection_array.detections.append(detection)

        self.detection_publisher.publish(detection_array)

    def draw_detections(self, frame, boxes, confidences, class_ids):
        for i in range(len(boxes)):
            cx, cy, w, h = boxes[i]
            x1 = int(cx - w/2)
            y1 = int(cy - h/2)
            x2 = int(cx + w/2)
            y2 = int(cy + h/2)
            conf = confidences[i]
            class_id = class_ids[i]
            class_name = self.classes[class_id]
            color = self.colors[class_id].tolist()
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"{class_name} {conf:.2f}", (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)

class Profiler:
    __slots__ = ('name', 'interval', 'start')
    _last_log_times = {}
    _counts = {}

    def __init__(self, name, interval=2.0):
        self.name = name
        self.interval = interval 
        self.start = 0.0

    def __enter__(self):
        self.start = time.perf_counter()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        end = time.perf_counter()
        dt = (end - self.start) * 1000 # ms
        Profiler._counts[self.name] = Profiler._counts.get(self.name, 0) + 1
        last_time = Profiler._last_log_times.get(self.name, 0)
        if end - last_time > self.interval:
            count = Profiler._counts[self.name]
            time_span = max(end - last_time, 0.001) 
            actual_hz = count / time_span
            print(f"[{self.name}] {dt:.2f}ms | {actual_hz:.2f}Hz")
            Profiler._last_log_times[self.name] = end
            Profiler._counts[self.name] = 0

def main(args=None):
    parser = argparse.ArgumentParser(description="YOLO ROS2 Inference Node.")
    parser.add_argument('--camera-id', type=int, default=0, help="Generic camera ID (0 for mono/left, 1 for right).")
    parser.add_argument('--headless', action='store_true', help="Run in headless mode.")
    parser.add_argument('--hitl', action='store_true', help="Open camerafrom gz-sim for HITL.")
    parser.add_argument('--remote-video-streams', action='store_true', help="Send video streams to the ground container.")
    parser.add_argument('--hfov', type=float, default=100.0, help="Horizontal field of view in degrees.")
    parser.add_argument('--ros2-frame-publisher', action='store_true', help="Publish raw frames to ROS 2.")
    parser.add_argument('--no-inference', action='store_true', help="Disable YOLO inference and only run camera acquisition/streaming.")
    cli_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)

    yolo_node = YoloInferenceNode(camera_id=cli_args.camera_id, headless=cli_args.headless, hitl=cli_args.hitl,
        remote_video_streams=cli_args.remote_video_streams, hfov=cli_args.hfov, ros2_frame_publisher=cli_args.ros2_frame_publisher,
        no_inference=cli_args.no_inference)
    yolo_node.run_inference_loop()
    
    yolo_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
