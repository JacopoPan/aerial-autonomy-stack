"""
Use as:
    python3 gz_step.py --step_sec 1.0
"""
import os
import argparse
import gz.transport13
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.boolean_pb2 import Boolean as GzBoolean


def main():
    parser = argparse.ArgumentParser(description='Control Gazebo world simulation stepping')
    parser.add_argument('--step_sec', type=float, help='Number of seconds to advance the simulation')
    args = parser.parse_args()
    
    world_name = os.environ.get('WORLD', 'default')
    autopilot = os.environ.get('AUTOPILOT', 'px4')

    gz_node = gz.transport13.Node()
    
    req = WorldControl()
    if autopilot == 'px4':
        req.multi_step = int(args.step_sec * 250) # PX4 SDF worlds use 250 steps per simulation step (4ms)
    elif autopilot == 'ardupilot':
        req.multi_step = int(args.step_sec * 500) # ArduPilot SDF worlds use 500 steps per simulation step (2ms)
    req.pause = True # Stops the simulation after the step

    result, response = gz_node.request(
        f"/world/{world_name}/control",
        req,
        WorldControl,
        GzBoolean,
        1000 # 1000ms = 1s
    )
    if result:
        pass
    else:
        print("Service call failed!")

if __name__ == "__main__":
    main()
