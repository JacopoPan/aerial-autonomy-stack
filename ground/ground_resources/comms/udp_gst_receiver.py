import cv2
import argparse
import sys

def main():
    parser = argparse.ArgumentParser(description="Receive a drone's GStreamer video over TCP and display it.")
    parser.add_argument('--host', type=str, required=True, help="Drone IP address to connect to.")
    parser.add_argument('--port', type=int, required=True, help="TCP port to connect to.")
    parser.add_argument('--name', type=str, required=True, help="Window name to display.")
    args = parser.parse_args()

    port = args.port
    name = args.name

    pipeline = (
        f"tcpclientsrc host={args.host} port={port} timeout=5 ! "
        "application/x-rtp-stream,media=video,clock-rate=90000,encoding-name=H265,payload=96 ! "
        "rtpstreamdepay ! "
        "rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! "
        "video/x-raw,format=BGR ! appsink max-buffers=1 drop=true sync=false"
    )
    # CPU h265 decoding, consider GPU decoding for many-drones scenarios
    # timeout=5: give up on a silent connection (e.g. drone rebooted) so the until-loop in ground.yml.erb reconnects
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    
    if not cap.isOpened():
        print(f"[{name}] Error: Could not open GStreamer pipeline on port {port}.")
        sys.exit(1)

    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    print(f"[{name}] Receiving stream from {args.host}:{port}...")

    while True:
        ret, frame = cap.read()
        if not ret:
            sys.exit(1) # Connection lost: exit so the until-loop in ground.yml.erb reconnects
        cv2.imshow(name, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
