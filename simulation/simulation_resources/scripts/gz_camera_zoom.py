"""
Publish a one-shot camera zoom command (gz.msgs.Double) on the per-model
zoom_cmd topic. The same topic is bridged to ROS2 in simulation.yml.erb as
/Drone<N>/camera/zoom_cmd, so external ROS2 nodes can drive the zoom too.

Use as:
    python3 gz_camera_zoom.py --model x500_0 --zoom 2.0
    python3 gz_camera_zoom.py --model iris_with_ardupilot_1 --zoom 1.0  # reset

Note: zoom is set at startup from sensor_config.yaml's camera_intrinsics.zoom_factor
(the static initial HFOV scaling). Runtime HFOV updates require a Gazebo plugin
that subscribes to the zoom_cmd topic and calls SetHFOV() on the camera sensor;
this script publishes on that topic so any such consumer (or the ros_gz_bridge)
can react.
"""
import time
import argparse
import gz.transport13
from gz.msgs10.double_pb2 import Double


def main():
    parser = argparse.ArgumentParser(description='Publish a camera zoom command on the model zoom_cmd topic')
    parser.add_argument('--model', type=str, required=True, help='Model name, e.g. x500_0 or iris_with_ardupilot_1')
    parser.add_argument('--zoom', type=float, default=1.0, help='Zoom factor (>1 zooms in, 1.0 resets)')
    args = parser.parse_args()

    if args.zoom <= 0.0:
        print(f"Zoom factor must be > 0 (got {args.zoom})")
        return

    topic = f"/model/{args.model}/camera/zoom_cmd"
    gz_node = gz.transport13.Node()
    pub = gz_node.advertise(topic, Double)

    timeout = 0
    while not pub.has_connections():
        if timeout > 5: # Give up after 5s
            print(f"No subscribers on {topic}! Is the ros_gz_bridge (or a zoom plugin) running?")
            return
        time.sleep(1.0)
        timeout += 1

    msg = Double()
    msg.data = args.zoom
    pub.publish(msg)
    print(f"Published zoom={args.zoom} on {topic}")
    time.sleep(0.5) # Wait for publication

if __name__ == "__main__":
    main()
