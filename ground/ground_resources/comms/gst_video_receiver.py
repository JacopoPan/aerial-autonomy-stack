import cv2
import argparse
import signal
import sys

def main():
    parser = argparse.ArgumentParser(description="Receive a drone's GStreamer video, over TCP or UDP, and display it.")
    parser.add_argument('--host', type=str, required=True, help="Drone IP address to connect to (for TCP only).")
    parser.add_argument('--port', type=int, required=True, help="TCP port to connect to, or UDP port to listen on.")
    parser.add_argument('--name', type=str, required=True, help="Window name to display.")
    args = parser.parse_args()

    port = args.port
    name = args.name

    decode = "rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! video/x-raw,format=BGR ! appsink max-buffers=1 drop=true sync=false"
    # CPU h265 decoding, consider GPU decoding for many-drones scenarios
    # timeout=5: give up on a silent connection (e.g. drone rebooted) so the until-loop in ground.yml.erb reconnects
    source = f"tcpclientsrc host={args.host} port={port} timeout=5 ! application/x-rtp-stream,media=video,clock-rate=90000,encoding-name=H265,payload=96 ! rtpstreamdepay"
    cap = cv2.VideoCapture(f"{source} ! {decode}", cv2.CAP_GSTREAMER) # Opens only if the drone serves TCP (VIDEO_STREAM_TRANSPORT=tcp)
    if not cap.isOpened(): # Otherwise, listen for UDP (VIDEO_STREAM_TRANSPORT=udp)
        source = f"udpsrc port={port} ! application/x-rtp,media=video,clock-rate=90000,encoding-name=H265,payload=96 ! rtpjitterbuffer latency=100 drop-on-latency=true"
        signal.alarm(5) # SIGALRM ends this process if no video arrives within 5 s, so the until-loop tries TCP again
        cap = cv2.VideoCapture(f"{source} ! {decode}", cv2.CAP_GSTREAMER) # Blocks until the first packets arrive

    if not cap.isOpened():
        print(f"[{name}] Error: Could not open GStreamer pipeline on port {port}.")
        sys.exit(1)

    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    print(f"[{name}] Receiving stream: {source}")

    while True:
        signal.alarm(5) # Restart the 5 s countdown on every frame: no frame for 5 s (e.g. the drone stopped streaming) ends this process and the until-loop starts over
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
