"""
This scripts allows to set wind velocity of plugin gz::sim::systems::WindEffects
- Wind reaches each LiftDrag/AdvancedLiftDrag/MulticopterMotorModel plugin, on every vehicle
- On the X500 and Iris, wind also applies a force on the base_link through tag <enable_wind>
- However, PX4's SENS_EN_ARSPDSIM, ArduPlane (SIM_)ARSPD_*, and gz::sim::systems::AirSpeed on VTOLs/tailsitters are blind to it
- Thus, this script is only recommended for use with QUADS

Use as:
    python3 gz_wind.py --from_west 0.0 --from_south 3.0
    python3 gz_wind.py --stop_wind
"""
import os
import time
import argparse
import gz.transport13
from gz.msgs10.wind_pb2 import Wind


def main():
    parser = argparse.ArgumentParser(description='Control Gazebo wind plugin')
    parser.add_argument('--from_west', type=float, default=0.0, help='Wind velocity from west toward east (m/s, Gazebo +X)')
    parser.add_argument('--from_south', type=float, default=0.0, help='Wind velocity from south toward north (m/s, Gazebo +Y)')
    parser.add_argument('--stop_wind', dest='stop_wind', action='store_true', help='Disable WindEffects (stop wind)')
    args = parser.parse_args()
    
    world_name = os.environ.get('WORLD', 'default')

    gz_node = gz.transport13.Node()

    pub = gz_node.advertise(f"/world/{world_name}/wind/", Wind)
    timeout = 0
    while not pub.has_connections():
        if timeout > 5:  # Give up after 5s
            print("No subscribers found! Is Gazebo running?")
            return
        time.sleep(1.0)
        timeout += 1

    wind_msg = Wind()
    if args.stop_wind:
        wind_msg.enable_wind = False
        print("Disabling WindEffects")
    else:
        wind_msg.linear_velocity.x = args.from_west
        wind_msg.linear_velocity.y = args.from_south
        # wind_msg.linear_velocity.z = 0.0 # Unused
        wind_msg.enable_wind = True
        print("Enabling WindEffects")

    pub.publish(wind_msg)
    time.sleep(0.5) # Wait for publication

if __name__ == "__main__":
    main()
