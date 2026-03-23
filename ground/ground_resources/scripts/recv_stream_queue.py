import cv2
import os
import threading
import queue
import time

DRONE_ID = os.getenv('DRONE_ID', '1')
print(f"Using DRONE_ID: {DRONE_ID}")

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

def frame_capture_thread(cap, frame_queue, is_running):
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
        try:
            if frame_queue.full():
                try:
                    frame_queue.get_nowait()
                except queue.Empty:
                    pass
            frame_queue.put_nowait(frame)
        except queue.Full:
            pass # Drop frame if the main thread is lagging

def run_inference_loop():
    # CPU pipeline
    gst_pipeline_string = (
        f"udpsrc port=560{DRONE_ID} ! "
        "application/x-rtp, media=(string)video, encoding-name=(string)H264 ! "
        "rtph264depay ! "
        "avdec_h264 ! " # Use CPU decoder
        "videoconvert ! "
        "video/x-raw, format=BGR ! appsink"
    )
    cap = cv2.VideoCapture(gst_pipeline_string, cv2.CAP_GSTREAMER)
    assert cap.isOpened(), "Failed to open video stream"
    print(f"Pipeline FPS: {cap.get(cv2.CAP_PROP_FPS)}")
    drone_id = os.getenv('DRONE_ID', '0')
    WINDOW_NAME = f"YOLOv8 (Aircraft {drone_id})"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.moveWindow(WINDOW_NAME, 800+(int(drone_id)-1)*25, 5+(int(drone_id)-1)*200)
    # cv2.resizeWindow(WINDOW_NAME, 400, 200)
    is_running = threading.Event()
    is_running.set()
    frame_queue = queue.Queue(maxsize=1) # A queue to hold frames, reduce maxsize to reduce latency (buffer bloat)
    frame_thread = threading.Thread(target=frame_capture_thread, args=(cap, frame_queue, is_running), daemon=True)
    frame_thread.start()
    while True:
        try:
            frame = frame_queue.get(timeout=1.0) # Get the most recent frame from the queue
        except queue.Empty:
            print("Frame queue is empty, is the stream running?")
            continue
        # Inference
        cv2.imshow(WINDOW_NAME, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    is_running.clear()
    frame_thread.join()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    run_inference_loop()