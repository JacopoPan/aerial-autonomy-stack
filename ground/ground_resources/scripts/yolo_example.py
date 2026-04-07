import cv2
import os
from ultralytics import YOLO

DRONE_ID = os.getenv('DRONE_ID', '0')
print(f"Using DRONE_ID: {DRONE_ID}")

MODEL_PATH = "/aas/ground_resources/models/yolov11-ime-target.pt"


def main():

    # Load YOLO model
    model = YOLO(MODEL_PATH)

    gst_pipeline_string = (
        f"udpsrc port=560{DRONE_ID} ! "
        "application/x-rtp, media=video, encoding-name=H264, payload=96 ! "
        "rtph264depay ! "
        "avdec_h264 ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! "
        "appsink sync=false drop=true"
    )

    cap = cv2.VideoCapture(gst_pipeline_string, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("Failed to open GStreamer pipeline")
        return

    print("Receiving stream... Press 'q' to exit")

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Frame not received")
            break

        # Run YOLO detection + ByteTrack tracking
        results = model.track(
            frame,
            persist=True,
            tracker="bytetrack.yaml",
            verbose=False
        )

        # Plot detections and tracking IDs
        annotated_frame = results[0].plot()

        cv2.imshow("UDP Stream + YOLO Tracking", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()