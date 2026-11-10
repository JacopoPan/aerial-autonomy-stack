#!/usr/bin/python3

# Based on: https://github.com/ros2/examples/tree/jazzy/rclpy/actions/minimal_action_client
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rosidl_runtime_py import set_message_fields
from rosidl_runtime_py.utilities import get_action

import argparse
import shlex
import yaml
import os
import signal
from threading import Thread

from action_msgs.msg import GoalStatus
STATUS = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
          GoalStatus.STATUS_CANCELED: 'CANCELED',
          GoalStatus.STATUS_ABORTED: 'ABORTED'}

class CancellableClient(Node):
    def __init__(self, action_type, action_name):
        super().__init__('cancellable_client')
        self._cancellable_client = ActionClient(self, action_type, action_name)
        self._goal_handle = None
    
    def send_goal(self, goal_msg):
        self.get_logger().info('Waiting for action server...')
        if not self._cancellable_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available')
            os._exit(1)
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
            return

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().warn(f'Cancel not accepted (code {cancel_response.return_code})')

    def get_result_callback(self, future):
        self._goal_handle = None
        status = STATUS.get(future.result().status, 'UNKNOWN')
        self.get_logger().info(f'Action finished with status: {status}')
        rclpy.shutdown()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            rclpy.shutdown()
            return
        self._goal_handle = goal_handle
        self.get_logger().info('Goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        if hasattr(feedback.feedback, 'message'):
            self.get_logger().info(f'Received feedback: {feedback.feedback.message}')

def main(args=None):
    parser = argparse.ArgumentParser(description='Cancellable ROS2 Action Client')
    parser.add_argument('command', type=str, help='The full ros2 action command to execute, in quotes.')
    cli_args = parser.parse_args()

    # Parse the action command
    parts = shlex.split(cli_args.command)
    if len(parts) != 6 or parts[1:3] != ['action', 'send_goal']:
        print("Error: expected \"ros2 action send_goal <name> <type> '<goal>'\", no flags")
        return
    action_name = parts[3]
    action_type_str = parts[4]
    goal_str = parts[5]
    try:
        action_type = get_action(action_type_str)
    except (AttributeError, ModuleNotFoundError, ValueError):
        print(f"Error: invalid action type '{action_type_str}'")
        return
    goal_dict = yaml.safe_load(goal_str)
    goal_msg = action_type.Goal()
    set_message_fields(goal_msg, goal_dict)
    
    rclpy.init(args=args)
    signal.signal(signal.SIGINT, signal.SIG_IGN) # Disable Ctrl+c on the script, press Enter to cancel the action (Ctrl+\ still kills the process)

    cancellable_client = CancellableClient(action_type, action_name)
    cancellable_client.send_goal(goal_msg)

    print("""
    -------------------------------------------
      Enter   cancel the action
      Ctrl+c  ignored
      Ctrl+\\  exit the script, NOT the action
    -------------------------------------------
    """, flush=True)

    # Listen for user cancellation in a separate thread
    def wait_for_input_and_cancel():
        input("Pressing Enter will cancel the action.\n")
        cancellable_client.cancel_goal_from_input()
    input_thread = Thread(target=wait_for_input_and_cancel)
    input_thread.start()

    rclpy.spin(cancellable_client)
    os._exit(0)

if __name__ == '__main__':
    main()
