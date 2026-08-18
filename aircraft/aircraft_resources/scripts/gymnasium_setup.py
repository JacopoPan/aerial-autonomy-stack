import argparse

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from autopilot_interface_msgs.action import Takeoff, Offboard


class GymnasiumSetup(Node):
    def __init__(self, drone_id):
        super().__init__('gymnasium_setup_node')
        self.drone_id = drone_id
        
        self.takeoff_client = ActionClient(self, Takeoff, f'/Drone{drone_id}/takeoff_action')
        self.offboard_client = ActionClient(self, Offboard, f'/Drone{drone_id}/offboard_action')

    def wait_for_server(self, client, name):
        self.get_logger().info(f'Waiting for {name} action server...')
        while not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info(f'{name} not available yet. Retrying...')
        self.get_logger().info(f'{name} server is ready.')

    def send_takeoff(self):
        self.wait_for_server(self.takeoff_client, 'Takeoff')
        
        goal_msg = Takeoff.Goal()
        goal_msg.takeoff_altitude = 40.0
        goal_msg.vtol_transition_heading = 330.0
        goal_msg.vtol_loiter_north = 100.0
        goal_msg.vtol_loiter_east = 100.0
        goal_msg.vtol_loiter_alt = 60.0

        self.get_logger().info('Sending Takeoff Goal...')
        
        send_goal_future = self.takeoff_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Takeoff Goal Rejected! Retrying...')
            return False

        self.get_logger().info('Takeoff Goal Accepted. Waiting for result...')
        
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        return True

    def send_offboard(self):
        self.wait_for_server(self.offboard_client, 'Offboard')

        goal_msg = Offboard.Goal()
        goal_msg.controller_name = "ctbr-test" # See px4_offboard.cpp
        goal_msg.max_duration_sec = 1200.0 # 20' of offboard mode

        self.get_logger().info('Sending Offboard Goal...')
        send_goal_future = self.offboard_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Offboard Goal Rejected.')
            return False
        
        self.get_logger().info('Offboard Mode Active.')
        return True

def spin_wait(node, seconds):
    target_time = node.get_clock().now() + Duration(seconds=seconds)
    while node.get_clock().now() < target_time:
        rclpy.spin_once(node, timeout_sec=0.1) # Process callbacks to receive /clock updates

def main(args=None):
    rclpy.init(args=args)
    
    parser = argparse.ArgumentParser(description='Gymnasium Setup Node')
    parser.add_argument('--drone_id', type=str, required=True, help='The ID of the drone')
    parsed_args, _ = parser.parse_known_args()
    drone_id = parsed_args.drone_id
    
    node = GymnasiumSetup(drone_id)

    # Takeoff
    takeoff_success = False
    while not takeoff_success:
        takeoff_success = node.send_takeoff()
        if not takeoff_success:
            spin_wait(node, 2.0) # Simulation time

    # # Offboard
    # offboard_success = False
    # while not offboard_success:
    #     offboard_success = node.send_offboard()
    #     if not offboard_success:
    #         spin_wait(node, 1.0) # Simulation time

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
