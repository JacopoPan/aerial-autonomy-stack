#!/usr/bin/python3

# Based on: https://github.com/ros2/examples/blob/humble/rclpy/actions/minimal_cancellable_client/examples_rclpy_minimal_cancellable_client/client_cancel.py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rosidl_runtime_py import set_message_fields

import argparse
import shlex
import yaml
from threading import Thread

from autopilot_interface_msgs.action import Land, Offboard, Takeoff, Orbit

class CancellableClient(Node):
    def __init__(self, action_type, action_name):
        super().__init__('cancellable_client')
        self._cancellable_client = ActionClient(self, action_type, action_name)
        self._goal_handle = None
    
    def send_goal(self, goal_msg):
        self.get_logger().info('Waiting for action server...')
        self._cancellable_client.wait_for_server()
        self.get_logger().info('Sending goal request...')
        self._send_goal_future = self._cancellable_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def cancel_goal_from_input(self):
        if self._goal_handle:
            self.get_logger().info('Canceling goal...')
            future = self._goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_done)
        else:
            self.get_logger().warn('No active goal to cancel.')
            rclpy.shutdown()

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')
        rclpy.shutdown()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self._goal_handle = goal_handle
        self.get_logger().info('Goal accepted! Press Enter to cancel (or exit this script once the action is completed).')

    def feedback_callback(self, feedback):
        if hasattr(feedback.feedback, 'message'):
            self.get_logger().info(f'Received feedback: {feedback.feedback.message}')

def main(args=None):
    parser = argparse.ArgumentParser(description='Cancellable ROS2 Action Client')
    parser.add_argument('command', type=str, help='The full ros2 action command to execute, in quotes.')
    cli_args = parser.parse_args()

    # Parse the action command
    parts = shlex.split(cli_args.command)
    action_name = parts[3]
    action_type_str = parts[4]
    goal_str = parts[5]
    ACTION_TYPE_MAP = {
        'autopilot_interface_msgs/action/Takeoff': Takeoff,
        'autopilot_interface_msgs/action/Land': Land,
        'autopilot_interface_msgs/action/Offboard': Offboard,
        'autopilot_interface_msgs/action/Orbit': Orbit,
    }
    if action_type_str not in ACTION_TYPE_MAP:
        print(f"Error: Unsupported action type '{action_type_str}'")
        return
    action_type = ACTION_TYPE_MAP[action_type_str]
    goal_dict = yaml.safe_load(goal_str)
    goal_msg = action_type.Goal()
    set_message_fields(goal_msg, goal_dict)
    
    rclpy.init(args=args)

    cancellable_client = CancellableClient(action_type, action_name)
    cancellable_client.send_goal(goal_msg)

    # Listen for user cancellation in a separate thread
    def wait_for_input_and_cancel():
        input("Press Enter to cancel the action (or to recoup the terminal after the action ends)...\n")
        cancellable_client.cancel_goal_from_input()
    input_thread = Thread(target=wait_for_input_and_cancel)
    input_thread.start()

    rclpy.spin(cancellable_client)

if __name__ == '__main__':
    main()
