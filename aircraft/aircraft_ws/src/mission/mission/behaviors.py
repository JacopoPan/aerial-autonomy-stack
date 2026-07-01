import os
import py_trees

import rclpy
from action_msgs.msg import GoalStatus
from geographiclib.geodesic import Geodesic

from autopilot_interface_msgs.action import Takeoff, Land, Orbit, Offboard
from autopilot_interface_msgs.srv import SetSpeed, SetReposition

# TIMER BEHAVIOR
class WaitBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node, params):
        super().__init__(name)
        self.ros_node = ros_node
        self.duration = float(params.get('duration', 0.0))
        self.blackboard = py_trees.blackboard.Blackboard()
        # Wait tracking variable
        self.start_time = None

    def initialise(self):
        self.ros_node.get_logger().info(f"[{self.name}] Starting wait for {self.duration}s...")
        self.start_time = self.ros_node.get_clock().now()

    def update(self):
        elapsed = (self.ros_node.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed >= self.duration:
            self.ros_node.get_logger().info(f"[{self.name}] Wait complete.")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            self.ros_node.get_logger().warn(f"[{self.name}] Wait interrupted!")
        # Reset variable
        self.start_time = None

# CONDITION BEHAVIOR
class CheckBlackboardBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node, params):
        super().__init__(name)
        self.ros_node = ros_node
        self.key = params.get('key')
        self.expression = params.get('expression', None)
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        data = self.blackboard.get(self.key)
        if data is None:
            self.ros_node.get_logger().info(f"[{self.name}] Condition failed: No data for {self.key}")
            return py_trees.common.Status.FAILURE
        condition_met = False
        if self.expression:
            try:
                condition_met = bool(eval(self.expression, {}, {"data": data}))
            except Exception as e:
                self.ros_node.get_logger().error(f"[{self.name}] Expression eval error: {e}")
                return py_trees.common.Status.FAILURE
        else: # Fallback if no expression is provided: only check whether data for the key exist
            if self.key == "yolo_detections":
                condition_met = len(data.detections) > 0
            elif self.key == "ground_tracks":
                condition_met = len(data.tracks) > 0
            elif self.key == "drone_states":
                condition_met = len(data) > 0
            else:
                condition_met = bool(data)
        if condition_met:
            self.ros_node.get_logger().info(f"[{self.name}] Condition met for key: {self.key}")
            return py_trees.common.Status.SUCCESS
        self.ros_node.get_logger().info(f"[{self.name}] Condition for {self.key}")
        return py_trees.common.Status.FAILURE

# BASE ACTION CLIENT BEHAVIOR (NON-BLOCKING)
class BaseActionBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node, params, client_name):
        super().__init__(name)
        self.ros_node = ros_node
        self.params = params
        self.client_name = client_name
        self.blackboard = py_trees.blackboard.Blackboard()
        self.action_client = None
        # Action tracking variables
        self.goal_sent = False
        self.goal_future = None
        self.result_future = None
        self.goal = None

    def create_goal(self):
        raise NotImplementedError()

    def feedback_callback(self, feedback_msg):
        self.ros_node.get_logger().info(f"[{self.name}] Feedback: {feedback_msg.feedback.message}")

    def initialise(self):
        self.action_client = self.blackboard.get(self.client_name)
        self.goal_sent = False
        self.goal_future = None
        self.result_future = None
        self.goal = self.create_goal()

    def update(self):
        if self.goal == "SKIPPED": # E.g. non-supported actions like Offboard for ArduPilot VTOLs
            return py_trees.common.Status.SUCCESS
        if self.action_client is None:
            self.ros_node.get_logger().error(f"[{self.name}] Client {self.client_name} missing!")
            return py_trees.common.Status.FAILURE
        # 1: Wait for server and send goal (non-blocking)
        if not self.goal_sent:
            if not self.action_client.server_is_ready():
                self.ros_node.get_logger().info(f"[{self.name}] Waiting for action server...", throttle_duration_sec=2.0)
                return py_trees.common.Status.RUNNING
            self.ros_node.get_logger().info(f"[{self.name}] Sending goal...")
            self.goal_future = self.action_client.send_goal_async(self.goal, feedback_callback=self.feedback_callback)
            self.goal_sent = True
            return py_trees.common.Status.RUNNING
        # 2: Wait for goal acceptance
        if self.result_future is None:
            if self.goal_future.done():
                goal_handle = self.goal_future.result()
                if not goal_handle.accepted:
                    self.ros_node.get_logger().error(f"[{self.name}] Goal rejected.")
                    return py_trees.common.Status.FAILURE
                self.result_future = goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING
        # 3: Wait for result
        if self.result_future.done():
            result = self.result_future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.ros_node.get_logger().info(f"[{self.name}] Action successful!")
                return py_trees.common.Status.SUCCESS
            else:
                self.ros_node.get_logger().error(f"[{self.name}] Action failed. Status: {result.status}")
                return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        # If interrupted by a higher priority node, cancel the running goal
        if new_status == py_trees.common.Status.INVALID and self.goal_future and not self.result_future:
            self.ros_node.get_logger().warn(f"[{self.name}] Interrupted! Cancelling goal...")
            if self.goal_future.done():
                goal_handle = self.goal_future.result()
                if goal_handle.accepted:
                    goal_handle.cancel_goal_async()
        # Reset variables
        self.goal_sent = False
        self.goal_future = None
        self.result_future = None

# ACTION CLIENT BEHAVIORS
class TakeoffBehavior(BaseActionBehavior):
    def __init__(self, name, ros_node, params):
        super().__init__(name, ros_node, params, client_name="takeoff_client")

    def create_goal(self):
        goal = Takeoff.Goal()
        goal.takeoff_altitude = float(self.params.get('takeoff_altitude', 20.0))
        goal.vtol_transition_heading = float(self.params.get('vtol_transition_heading', 0.0))
        goal.vtol_loiter_nord = float(self.params.get('vtol_loiter_nord', 100.0))
        goal.vtol_loiter_east = float(self.params.get('vtol_loiter_east', 100.0))
        goal.vtol_loiter_alt = float(self.params.get('vtol_loiter_alt', 120.0))
        return goal

class LandBehavior(BaseActionBehavior):
    def __init__(self, name, ros_node, params):
        super().__init__(name, ros_node, params, client_name="land_client")

    def create_goal(self):
        goal = Land.Goal()
        goal.landing_altitude = float(self.params.get('landing_altitude', 20.0))
        goal.vtol_transition_heading = float(self.params.get('vtol_transition_heading', 0.0))
        return goal

class OrbitBehavior(BaseActionBehavior):
    def __init__(self, name, ros_node, params):
        super().__init__(name, ros_node, params, client_name="orbit_client")

    def create_goal(self):
        goal = Orbit.Goal()
        goal.east = float(self.params.get('east', 0.0))
        goal.north = float(self.params.get('north', 0.0))
        goal.altitude = float(self.params.get('altitude', 20.0))
        goal.radius = float(self.params.get('radius', 10.0))
        return goal

class OffboardBehavior(BaseActionBehavior):
    def __init__(self, name, ros_node, params):
        super().__init__(name, ros_node, params, client_name="offboard_client")

    def create_goal(self):
        autopilot, drone_type = os.getenv('AUTOPILOT', ''), os.getenv('DRONE_TYPE', '')
        if autopilot == 'ardupilot' and drone_type != 'quad':
            self.ros_node.get_logger().warn(f"[{self.name}] Offboard (GUIDED MODE) in ArduPilot is only supported for 'DRONE_TYPE=quad'. Skipping.")
            return "SKIPPED"
        goal = Offboard.Goal()
        default_controller = 'traj-test' if autopilot == 'px4' else 'vel-test' # Pick a default controller is not specified in the YAML
        goal.controller_name = str(self.params.get('controller_name', default_controller))
        goal.max_duration_sec = float(self.params.get('max_duration_sec', 10.0))
        return goal

# SIMPLE SERVICE BEHAVIOR WITH NO MONITORING
class SpeedBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node, params):
        super().__init__(name)
        self.ros_node = ros_node
        self.params = params
        self.blackboard = py_trees.blackboard.Blackboard()
        self.service_client = None
        # Service tracking variables
        self.req_sent = False
        self.future = None

    def initialise(self):
        self.service_client = self.blackboard.get("speed_client")
        self.req_sent = False
        self.future = None

    def update(self):
        if self.service_client is None: return py_trees.common.Status.FAILURE
        if not self.req_sent:
            if not self.service_client.service_is_ready():
                self.ros_node.get_logger().info(f"[{self.name}] Waiting for service...", throttle_duration_sec=2.0)
                return py_trees.common.Status.RUNNING
            self.ros_node.get_logger().info(f"[{self.name}] Requesting speed change...")
            req = SetSpeed.Request()
            req.speed = float(self.params.get('speed', 15.0))
            self.future = self.service_client.call_async(req)
            self.req_sent = True
            return py_trees.common.Status.RUNNING
        if self.future.done():
            response = self.future.result()
            if response.success:
                self.ros_node.get_logger().info(f"[{self.name}] Speed set successfully.")
                return py_trees.common.Status.SUCCESS
            else:
                self.ros_node.get_logger().error(f"[{self.name}] Speed service failed: {response.message}")
                return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            self.ros_node.get_logger().warn(f"[{self.name}] Speed request interrupted!")
        # Reset variables
        self.req_sent = False
        self.future = None

# COMPLEX SERVICE BEHAVIOR WITH BLACKBOARD MONITORING
class RepositionBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, ros_node, params):
        super().__init__(name)
        self.ros_node = ros_node
        self.params = params
        self.blackboard = py_trees.blackboard.Blackboard()
        self.service_client = None
        # Service and reposition tracking variables
        self.req_sent = False
        self.service_future = None
        self.reposition_active = False
        self.has_moved = False
        self.stable_ticks = 0
        self.reposition_wait_ticks = 0
        self.prev_lat = None
        self.prev_lon = None

    def initialise(self):
        self.service_client = self.blackboard.get("reposition_client")
        self.req_sent = False
        self.service_future = None
        self.reposition_active = False
        if os.getenv('DRONE_TYPE', '') != 'quad':
            self.ros_node.get_logger().warn(f"[{self.name}] Reposition is only supported for 'DRONE_TYPE=quad'. Skipping.")
            self.req_sent = "SKIPPED"

    def update(self):
        if self.req_sent == "SKIPPED": return py_trees.common.Status.SUCCESS
        if self.service_client is None: return py_trees.common.Status.FAILURE
        # 1: Wait for service and send request
        if not self.req_sent:
            if not self.service_client.service_is_ready():
                self.ros_node.get_logger().info(f"[{self.name}] Waiting for reposition service...", throttle_duration_sec=2.0)
                return py_trees.common.Status.RUNNING
            self.ros_node.get_logger().info(f"[{self.name}] Requesting reposition...")
            req = SetReposition.Request()
            req.east = float(self.params.get('east', 0.0))
            req.north = float(self.params.get('north', 0.0))
            req.altitude = float(self.params.get('altitude', 50.0))
            self.service_future = self.service_client.call_async(req)
            self.req_sent = True
            return py_trees.common.Status.RUNNING
        # 2: Wait for service response
        if not self.reposition_active:
            if self.service_future.done():
                response = self.service_future.result()
                if response.success:
                    self.ros_node.get_logger().info(f"[{self.name}] Service successful. Monitoring position...")
                    self.reposition_active = True
                    self.has_moved = False
                    self.stable_ticks = 0
                    self.reposition_wait_ticks = 0
                    self.prev_lat = self.blackboard.get("lat")
                    self.prev_lon = self.blackboard.get("lon")
                else:
                    self.ros_node.get_logger().error(f"[{self.name}] Service failed.")
                    return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING
        # 3: Monitor position stabilization
        lat = self.blackboard.get("lat")
        lon = self.blackboard.get("lon")
        if self.prev_lat is not None and self.prev_lon is not None and lat is not None and lon is not None:
            distance_moved = Geodesic.WGS84.Inverse(self.prev_lat, self.prev_lon, lat, lon)['s12']
            if distance_moved > 0.5: # Started moving (change between ticks @2Hz > 0.5m)
                self.has_moved = True
            if self.has_moved and distance_moved < 0.2: # Stabilized/stopped (change between ticks @2Hz < 0.2m)
                self.stable_ticks += 1
            elif self.has_moved:
                self.stable_ticks = 0
        self.prev_lat = lat
        self.prev_lon = lon
        self.reposition_wait_ticks += 1
        if (self.stable_ticks >= 6 # 6 ticks @2Hz = 3sec
                or (not self.has_moved and self.reposition_wait_ticks > 20)): # 20 ticks @2Hz = 10sec
            self.ros_node.get_logger().info(f"[{self.name}] Destination reached.")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            self.ros_node.get_logger().warn(f"[{self.name}] Reposition interrupted!")
        # Reset variables
        self.req_sent = False
        self.service_future = None
        self.reposition_active = False
        self.has_moved = False
        self.stable_ticks = 0
        self.reposition_wait_ticks = 0
        self.prev_lat = None
        self.prev_lon = None
