import os, json, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from autopilot_interface_msgs.srv import SetReposition
from autopilot_interface_msgs.action import Takeoff

class DTCClient(Node):
    def __init__(self):
        super().__init__('dtc_client')
        self.drone_id = os.environ.get('DRONE_ID', '1')
        self.create_subscription(String, '/dtc_commands', self.cmd_cb, 10)

        self.repo_cli = self.create_client(SetReposition, f'/Drone{self.drone_id}/set_reposition')
        self.tkf_cli = ActionClient(self, Takeoff, f'/Drone{self.drone_id}/takeoff_action')

        # Enforcement Loop Variables
        self.target_action = None
        self.target_req = None
        self.action_accepted = True 

        # Runs at 1Hz to enforce the command until the autopilot accepts it
        self.enforcer_timer = self.create_timer(1.0, self.enforcement_loop)

    def cmd_cb(self, msg):
        try:
            cmd = json.loads(msg.data)
            if str(cmd.get('drone_id')) != self.drone_id: return

            # New command received! Set it as target and mark as not accepted
            self.target_action = cmd.get('action')
            self.target_req = cmd
            self.action_accepted = False
            self.get_logger().info(f"New Command Queued: {self.target_action}")

        except Exception as e:
            self.get_logger().error(f"Failed to process command: {e}")

    def enforcement_loop(self):
        """Continuously attempts to send the command if it hasn't been accepted yet."""
        if self.action_accepted or not self.target_action:
            return

        self.get_logger().info(f"Attempting to enforce {self.target_action}...")

        if self.target_action == 'takeoff' and self.tkf_cli.server_is_ready():
            goal = Takeoff.Goal(takeoff_altitude=float(self.target_req.get('alt', 40.0)))
            future = self.tkf_cli.send_goal_async(goal)
            future.add_done_callback(self.takeoff_response_cb)

        elif self.target_action == 'reposition' and self.repo_cli.service_is_ready():
            req = SetReposition.Request(
                east=float(self.target_req['east']),
                north=float(self.target_req['north']),
                altitude=float(self.target_req['alt'])
            )
            future = self.repo_cli.call_async(req)
            future.add_done_callback(self.reposition_response_cb)

    def takeoff_response_cb(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info("Autopilot ACCEPTED Takeoff!")
            self.action_accepted = True
        else:
            self.get_logger().warn("Autopilot REJECTED Takeoff. Will retry...")

    def reposition_response_cb(self, future):
        res = future.result()
        if res.success:
            self.get_logger().info("Autopilot ACCEPTED Reposition!")
            self.action_accepted = True
        else:
            self.get_logger().warn(f"Autopilot REJECTED Reposition: {res.message}. Will retry...")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DTCClient())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
