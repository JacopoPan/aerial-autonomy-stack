import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

import os
import argparse
import threading
import random
import time
import yaml

from action_msgs.msg import GoalStatus
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import VfrHud
from vision_msgs.msg import Detection2DArray
from px4_msgs.msg import VehicleGlobalPosition, AirspeedValidated

from ground_system_msgs.msg import SwarmObs
from state_sharing.msg import SharedState
from autopilot_interface_msgs.action import Land, Offboard, Takeoff, Orbit
from autopilot_interface_msgs.srv import SetSpeed, SetReposition

class MissionNode(Node):
    def __init__(self, mission_file):
        super().__init__('mission_node')

        self.mission_plan = []
        self.get_logger().info(f"Loading conops from: {mission_file}")

        if not os.path.isabs(mission_file):
            mission_file = os.path.join('/aas/aircraft_resources/missions/', mission_file)
        try:
            with open(mission_file, 'r') as f:
                data = yaml.safe_load(f)
                self.mission_plan = data.get('steps', [])
                self.get_logger().info(f"Loaded {len(self.mission_plan)} steps.")
        except Exception as e:
            self.get_logger().error(f"Failed to load mission file: {e}")
            self.mission_plan = []

        self.mission_step = 0 # Track the advancement of the mission
        self.last_executed_step = -1 # To avoid re-executing the same step
        self.active_mission_goal_handle = None # Hold the goal handle of the active action
        self.wait_start_time = None # For "wait" mission steps

        self.own_drone_id = None
        drone_id_str = os.environ.get('DRONE_ID') # Get id from ENV VAR
        if drone_id_str is None:
            self.get_logger().info("DRONE_ID environment variable not set.")
        else:
            try:
                self.own_drone_id = int(drone_id_str)
            except ValueError:
                self.get_logger().info(f"Could not parse DRONE_ID='{drone_id_str}' as an integer.")

        self.data_lock = threading.Lock()
        # MAVROS data
        self.lat = None
        self.lon = None
        self.alt_msl = None
        self.heading = None
        self.airspeed = None
        # Perception data
        self.yolo_detections = None
        # self.ground_tracks = None
        # State sharing
        self.active_state_sharing_subs = {}
        self.drone_states = {}
        self.STALE_DRONE_TIMEOUT_SEC = 5.0 # Time after which we prune a drone from drone_states

        # Create a reentrant callback groups to allow callbacks to run in parallel
        self.subscriber_callback_group = ReentrantCallbackGroup()
        self.timer_callback_group = ReentrantCallbackGroup()
        self.action_callback_group = ReentrantCallbackGroup()
        self.service_callback_group = ReentrantCallbackGroup()

        # Create a QoS profile for the subscribers
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        # PX4 subscribers
        self.create_subscription( # 100Hz
            VehicleGlobalPosition, 'fmu/out/vehicle_global_position', self.px4_global_position_callback,
            self.qos_profile, callback_group=self.subscriber_callback_group)
        self.create_subscription( # 10Hz, MESSAGE_VERSION = 1 -> _v1 since 1.17
            AirspeedValidated, '/fmu/out/airspeed_validated_v1', self.airspeed_validated_callback,
            self.qos_profile, callback_group=self.subscriber_callback_group)
        # MAVROS subscribers
        self.create_subscription( # 4Hz
            NavSatFix, '/mavros/global_position/global', self.mavros_global_position_callback,
            self.qos_profile, callback_group=self.subscriber_callback_group)
        self.create_subscription( # 4Hz
            VfrHud, '/mavros/vfr_hud', self.vfr_hud_callback,
            self.qos_profile, callback_group=self.subscriber_callback_group)
        # Perception subscribers
        self.create_subscription( # 15Hz
            Detection2DArray, '/detections', self.yolo_detections_callback,
            self.qos_profile, callback_group=self.subscriber_callback_group)
        # self.create_subscription( # 1Hz
        #     SwarmObs, '/tracks', self.ground_tracks_callback,
        #     self.qos_profile, callback_group=self.subscriber_callback_group)

        # Timed callbacks
        self.discover_drones_timer = self.create_timer(
            5.0, # 0.2Hz
            self.discover_drones_callback,
            callback_group=self.timer_callback_group
        )
        self.stale_check_timer = self.create_timer(
            2.0, # 0.5Hz
            self.check_stale_drones_callback,
            callback_group=self.timer_callback_group
        )
        self.printout_timer = self.create_timer(
            3.0, # 0.33Hz
            self.printout_callback,
            callback_group=self.timer_callback_group
        )
        self.conops_timer = self.create_timer(
            1.0, # 1Hz
            self.conops_callback,
            callback_group=self.timer_callback_group
        )

        # Actions
        self._takeoff_client = ActionClient(self, Takeoff, 'takeoff_action', callback_group=self.action_callback_group)
        self._land_client = ActionClient(self, Land, 'land_action', callback_group=self.action_callback_group)
        self._orbit_client = ActionClient(self, Orbit, 'orbit_action', callback_group=self.action_callback_group)
        self._offboard_client = ActionClient(self, Offboard, 'offboard_action', callback_group=self.action_callback_group)

        # Services
        if self.own_drone_id is not None:
            self._speed_client = self.create_client(
                SetSpeed, f'/Drone{self.own_drone_id}/set_speed',
                callback_group=self.service_callback_group
            )
            self._reposition_client = self.create_client(
                SetReposition, f'/Drone{self.own_drone_id}/set_reposition',
                callback_group=self.service_callback_group
            )
        else:
            self._speed_client = None
            self._reposition_client = None
            self.get_logger().info("DRONE_ID not set, service clients not created.")

    def px4_global_position_callback(self, msg): # Mutally exclusive with mavros_global_position_callback
        with self.data_lock:
            self.lat = msg.lat
            self.lon = msg.lon
            self.alt_msl = msg.alt

    def airspeed_validated_callback(self, msg): # Mutally exclusive with vfr_hud_callback
        with self.data_lock:
            self.airspeed = msg.true_airspeed_m_s

    def mavros_global_position_callback(self, msg):  # Mutally exclusive with px4_global_position_callback
        with self.data_lock:
            self.lat = msg.latitude
            self.lon = msg.longitude

    def vfr_hud_callback(self, msg): # Mutally exclusive with airspeed_validated_callback
        with self.data_lock:
            self.alt_msl = msg.altitude
            self.heading = msg.heading
            self.airspeed = msg.airspeed

    def yolo_detections_callback(self, msg):
        with self.data_lock:
            self.yolo_detections = msg

    def discover_drones_callback(self):
        topic_prefix = '/state_sharing_drone_'
        current_topics_and_types = self.get_topic_names_and_types() # This still re-discovers dead Zenoh topics but data won't be added to drone_states if they are not published
        for topic_name, msg_types in current_topics_and_types:
            if topic_name.startswith(topic_prefix) and topic_name not in self.active_state_sharing_subs:
                if 'state_sharing/msg/SharedState' in msg_types:
                    try:
                        topic_drone_id = int(topic_name.replace(topic_prefix, ''))
                        if topic_drone_id == self.own_drone_id:
                            continue # Ignore self
                    except ValueError:
                        continue # Skip if the topic name is malformed
                    self.get_logger().info(f"Discovered new drone: subscribing to {topic_name}")
                    sub = self.create_subscription( # 1Hz
                        SharedState,
                        topic_name,
                        self.state_sharing_callback,
                        self.qos_profile,
                        callback_group=self.subscriber_callback_group
                    )
                    self.active_state_sharing_subs[topic_name] = sub # Store the subscriber

    def check_stale_drones_callback(self):
        now = self.get_clock().now()
        stale_ids = []
        with self.data_lock:
            for drone_id, (last_msg, last_seen_time) in self.drone_states.items():
                duration = now - last_seen_time
                if duration.nanoseconds / 1e9 > self.STALE_DRONE_TIMEOUT_SEC:
                    stale_ids.append(drone_id)
            for drone_id in stale_ids:
                self.get_logger().info(f"Drone {drone_id} timed out. Removing.")
                # Remove the subscriber
                topic_name_to_remove = f"/state_sharing_drone_{drone_id}"
                if topic_name_to_remove in self.active_state_sharing_subs:
                    sub = self.active_state_sharing_subs.pop(topic_name_to_remove)
                    self.destroy_subscription(sub)
                # Remove the data
                self.drone_states.pop(drone_id, None)

    def state_sharing_callback(self, msg):
        # A single callback for all drone state topics
        with self.data_lock:
            now = self.get_clock().now()
            self.drone_states[msg.drone_id] = (msg, now)

    # def ground_tracks_callback(self, msg):
    #     with self.data_lock:
    #         self.ground_tracks = msg

    def printout_callback(self):
        with self.data_lock: # Copy with lock
            mission_step = self.mission_step
            lat = self.lat
            lon = self.lon
            alt_msl = self.alt_msl
            yolo_detections = self.yolo_detections
            states_copy = self.drone_states.copy()
            # ground_tracks = self.ground_tracks
        now_seconds = self.get_clock().now().nanoseconds / 1e9
        output = f"\nCurrent node time: {now_seconds:.2f} seconds\n"
        output += f"Mission step: {mission_step}\n"
        lat_str = f"{lat:.5f}" if lat is not None else "N/A"
        lon_str = f"{lon:.5f}" if lon is not None else "N/A"
        alt_str = f"{alt_msl:.2f}" if alt_msl is not None else "N/A"
        output += f"Global Position:\n  lat: {lat_str} lon: {lon_str} alt: {alt_str} (msl)\n"
        #
        if yolo_detections and yolo_detections.detections:
            output += "YOLO Detections:\n"
            for detection in yolo_detections.detections:
                for result in detection.results:
                    output += f"  Label: {result.hypothesis.class_id} - conf: {result.hypothesis.score:.2f}\n"
        else:
            output += "YOLO Detections: [No data]\n"
        #
        if not states_copy:
            output += "State Sharing: [No data]\n"
        else:
            now_seconds = self.get_clock().now().nanoseconds / 1e9
            output += "State Sharing:\n"
            for drone_id, (state_msg, last_seen_time) in sorted(states_copy.items()):
                seconds_ago = now_seconds - (last_seen_time.nanoseconds / 1e9)
                output += (f"  Id {drone_id}, lat: {state_msg.latitude_deg:.5f} lon: {state_msg.longitude_deg:.5f}, "
                        f"alt: {state_msg.altitude_m:.2f} (px4: msl, ap: ell.), hdg: {state_msg.heading_deg:.1f}deg, "
                        f"vel: [{state_msg.vx:.1f}, {state_msg.vy:.1f}, {state_msg.vz:.1f}]"
                        f"(seen {seconds_ago:.1f}s ago)\n")
        #
        # if ground_tracks and ground_tracks.tracks:
        #     output += "Ground Tracks:\n"
        #     for track in ground_tracks.tracks:
        #         output += f"  Id {track.id}, lat: {track.latitude_deg:.5f} lon: {track.longitude_deg:.5f} alt (msl): {track.altitude_m:.2f}\n"
        # else:
        #     output += "Ground Tracks: [No data]\n"
        #
        self.get_logger().info(output)

    def send_goal(self, client, goal_msg):
        if self.active_mission_goal_handle is not None:
            self.get_logger().info("An action is already in progress. Cannot send new goal.")
            return
        self.get_logger().info('Waiting for action server...')
        client.wait_for_server()
        self.get_logger().info('Sending goal request...')
        send_goal_future = client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.mission_step = -1
            return
        self.active_mission_goal_handle = goal_handle
        self.get_logger().info('Goal accepted! Waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.active_mission_goal_handle = None # Clear the handle
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Action succeeded! Result: {result.success}")
            self.mission_step += 1 # Advance the mission step
        else:
            self.get_logger().info(f"Action failed with status: {status}")
            self.mission_step = -1

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Received action feedback: {feedback_msg.feedback.message}")

    def call_service(self, server, request):
        if server is None or not server.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service not available.')
            return
        future = server.call_async(request)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(f"Service call failed: {response.message}")
                self.mission_step = -1
                return
            self.get_logger().info(f'Service call successful: {response.success}')
            self.mission_step += 1 # Advance the mission step
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self.mission_step = -1

    def conops_callback(self):
        # If an ROS Action is running, do nothing
        if self.active_mission_goal_handle is not None:
            return

        # If a "Wait" is active, check time: if still waiting, return. If done, clear wait and proceed
        if self.wait_start_time is not None:
            elapsed = (self.get_clock().now() - self.wait_start_time).nanoseconds / 1e9
            if elapsed < self.current_wait_duration:
                return # Still waiting
            else:
                self.get_logger().info(f"Wait complete")
                self.wait_start_time = None # Clear the wait flag
                self.mission_step += 1 # Advance to next step

        # End the mission
        if (self.mission_step >= len(self.mission_plan)) or (self.mission_step == -1):
            if self.mission_step == -1:
                self.get_logger().info("Mission Failed")
            else:
                self.get_logger().info("Mission Complete")
            self.conops_timer.cancel()
            rclpy.shutdown()
            return

        # Avoid executing the same step more than once
        if self.mission_step == self.last_executed_step:
            return

        # Continue the mission
        step = self.mission_plan[self.mission_step]
        action_type = step['action']
        params = step.get('params', {})
        self.get_logger().info(f"Executing step {self.mission_step}: {action_type}")
        self.last_executed_step = self.mission_step

        if action_type == 'wait':
            self.wait_start_time = self.get_clock().now()
            self.current_wait_duration = float(params.get('duration', 0.0))
            return

        elif action_type == 'takeoff':
            goal = Takeoff.Goal()
            goal.takeoff_altitude = float(params.get('takeoff_altitude', 20.0))
            goal.vtol_transition_heading = float(params.get('vtol_transition_heading', 0.0))
            goal.vtol_loiter_nord = float(params.get('vtol_loiter_nord', 100.0))
            goal.vtol_loiter_east = float(params.get('vtol_loiter_east', 100.0))
            goal.vtol_loiter_alt = float(params.get('vtol_loiter_alt', 120.0))
            self.send_goal(self._takeoff_client, goal)
        
        elif action_type == 'land':
            goal = Land.Goal()
            goal.landing_altitude = float(params.get('landing_altitude', 20.0))
            goal.vtol_transition_heading = float(params.get('vtol_transition_heading', 0.0))
            self.send_goal(self._land_client, goal)

        elif action_type == 'orbit':
            goal = Orbit.Goal()
            goal.east = float(params.get('east', 0.0))
            goal.north = float(params.get('north', 0.0))
            goal.altitude = float(params.get('altitude', 20.0))
            goal.radius = float(params.get('radius', 10.0))
            self.send_goal(self._orbit_client, goal)
            
        elif action_type == 'offboard':
            if (os.getenv('AUTOPILOT', '') == 'ardupilot') and (os.getenv('DRONE_TYPE', '') != 'quad'):
                self.get_logger().warn("Offboard action is not supported by Ardupilot VTOL. Skip.")
                self.mission_step += 1
                return
            default_setpoint_type = 2 if os.getenv('AUTOPILOT', '') == 'px4' else 3 # 2: PX4 trajectory reference, 3: ArduPilot velocity
            goal = Offboard.Goal()
            goal.offboard_setpoint_type = int(params.get('offboard_setpoint_type', default_setpoint_type))
            goal.max_duration_sec = float(params.get('max_duration_sec', 10.0))
            self.send_goal(self._offboard_client, goal)

        elif action_type == 'reposition':
            if os.getenv('DRONE_TYPE', '') != 'quad':
                self.get_logger().warn("Reposition action is only supported for 'quad' drone type. Skip.")
                self.mission_step += 1
                return
            req = SetReposition.Request()
            req.east = float(params.get('east', 0.0))
            req.north = float(params.get('north', 0.0))
            req.altitude = float(params.get('altitude', 50.0))
            self.call_service(self._reposition_client, req)
            
        elif action_type == 'speed':
            req = SetSpeed.Request()
            req.speed = float(params.get('speed', 15.0))
            self.call_service(self._speed_client, req)

        else:
            self.get_logger().error(f"Unknown action: {action_type}")
            self.mission_step = -1

def main(args=None):
    parser = argparse.ArgumentParser(description="Mission Node.")
    parser.add_argument('--conops', type=str, default='yalla.yaml', help="Path to the mission YAML file")

    cli_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    mission_node = MissionNode(mission_file=cli_args.conops)

    executor = MultiThreadedExecutor() # Or set MultiThreadedExecutor(num_threads=4)
    executor.add_node(mission_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        mission_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
