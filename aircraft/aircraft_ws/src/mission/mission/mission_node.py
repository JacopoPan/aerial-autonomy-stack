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
    def __init__(self, conops):
        super().__init__('mission_node')
        self.conops = conops
        self.get_logger().info(f"Missioning with CONOPS: {self.conops}")

        self.mission_step = 0 # Track the advancement of the mission
        self.active_mission_goal_handle = None # Hold the goal handle of the active action

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
        self.create_subscription( # 10Hz
            AirspeedValidated, '/fmu/out/airspeed_validated', self.airspeed_validated_callback,
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
                output += f"  Id {drone_id}, lat: {state_msg.latitude_deg:.5f} lon: {state_msg.longitude_deg:.5f} alt: {state_msg.altitude_m:.2f} (px4: msl, ap: ell.) (seen {seconds_ago:.1f}s ago)\n"
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
            self.get_logger().info(f'Service call successful: {response.success}')
            self.mission_step += 1 # Advance the mission step
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self.mission_step = -1

    def conops_callback(self):
        if self.active_mission_goal_handle is not None:
            return # Do nothing while an action is active

        ################################################################################
        if self.conops == 'plan_A':
            self.get_logger().info("[Plan A] Plan A is classified")
        ################################################################################
        elif self.conops == 'plan_B':
            self.get_logger().info("[Plan B] There is no Plan B")
        ################################################################################
        elif self.conops == 'yalla':
            if self.mission_step == -1:
                self.get_logger().info("[Yalla] Mission failed")
                self.conops_timer.cancel() # Stop this timer
                return
            elif self.mission_step == 0:
                self.get_logger().info("[Yalla] Taking off")
                self.mission_step = 1 # Dummy step to wait for takeoff completion
                takeoff_goal = Takeoff.Goal()
                takeoff_goal.takeoff_altitude = 40.0
                takeoff_goal.vtol_transition_heading = 300.0
                takeoff_goal.vtol_loiter_nord = 100.0
                takeoff_goal.vtol_loiter_east = 100.0
                takeoff_goal.vtol_loiter_alt = 120.0
                self.send_goal(self._takeoff_client, takeoff_goal)
            elif self.mission_step == 2:
                self.get_logger().info("[Yalla] Orbiting")
                self.mission_step = 3 # Dummy step to wait for orbit completion
                orbit_goal = Orbit.Goal()
                orbit_goal.east = -100.0
                orbit_goal.north = 50.0
                orbit_goal.altitude = 50.0
                orbit_goal.radius = 80.0
                self.send_goal(self._orbit_client, orbit_goal)
                self.orbit_start_time = self.get_clock().now()
            elif self.mission_step == 4:
                elapsed_time = self.get_clock().now() - self.orbit_start_time
                elapsed_sec = elapsed_time.nanoseconds / 1e9
                if elapsed_sec > 45.0: # Start landing 45 sec after the orbit
                    self.get_logger().info("[Yalla] Landing")
                    self.mission_step = 5 # Dummy step to wait for landing completion
                    land_goal = Land.Goal()
                    land_goal.landing_altitude = 60.0
                    land_goal.vtol_transition_heading = 60.0
                    self.send_goal(self._land_client, land_goal)
            elif self.mission_step == 6:
                self.get_logger().info("[Yalla] Mission complete")
                self.conops_timer.cancel() # Stop this timer
        ################################################################################
        elif self.conops == 'cat':
            if self.mission_step == -1:
                self.get_logger().info("[Cat] Mission failed")
                self.conops_timer.cancel() # Stop this timer
                return
            elif self.mission_step == 0:
                self.get_logger().info("[Cat] Taking off")
                self.mission_step = 1 # Dummy step to wait for takeoff completion
                takeoff_goal = Takeoff.Goal()
                takeoff_goal.takeoff_altitude = 20.0
                self.send_goal(self._takeoff_client, takeoff_goal)
            elif self.mission_step == 2:
                self.get_logger().info("[Cat] Repositioning")
                self.mission_step = 3 # Dummy step to wait for reposition completion
                repo_req = SetReposition.Request()
                repo_req.east = random.uniform(-100.0, 100.0)
                repo_req.north = random.uniform(-10.0, 10.0)
                repo_req.altitude = random.uniform(30.0, 60.0)
                self.call_service(self._reposition_client, repo_req)
                self.repo_start_time = self.get_clock().now()
            elif self.mission_step == 4:
                elapsed_time = self.get_clock().now() - self.repo_start_time
                elapsed_sec = elapsed_time.nanoseconds / 1e9
                if elapsed_sec > 15.0: # Start landing 15 sec after the reposition
                    self.get_logger().info("[Cat] Offboarding")
                    self.mission_step = 5 # Dummy step to wait for offboard completion
                    offboard_goal = Offboard.Goal()
                    offboard_goal.offboard_setpoint_type = 3
                    offboard_goal.max_duration_sec = 180.0
                    self.send_goal(self._offboard_client, offboard_goal)
                    # TODO: add termination
        ################################################################################
        elif self.conops == 'mouse':
            if self.mission_step == -1:
                self.get_logger().info("[Mouse] Mission failed")
                self.conops_timer.cancel() # Stop this timer
                return
            elif self.mission_step == 0:
                self.get_logger().info("[Mouse] Taking off")
                self.mission_step = 1 # Dummy step to wait for takeoff completion
                takeoff_goal = Takeoff.Goal()
                takeoff_goal.takeoff_altitude = 20.0
                self.send_goal(self._takeoff_client, takeoff_goal)
            elif self.mission_step == 2:
                self.get_logger().info("[Mouse] Orbiting")
                self.mission_step = 3 # Dummy step to wait for orbit completion
                orbit_goal = Orbit.Goal()
                orbit_goal.east = -50.0
                orbit_goal.north = 100.0
                orbit_goal.altitude = 50.0
                orbit_goal.radius = 40.0
                self.send_goal(self._orbit_client, orbit_goal)
                # TODO: add termination 
        ################################################################################
        else:
            self.get_logger().info(f"Unknown CONOPS: {self.conops}")

def main(args=None):
    parser = argparse.ArgumentParser(description="Mission Node.")
    parser.add_argument(
        '--conops',
        type=str,
        choices=['plan_A', 'plan_B', 'yalla', 'cat', 'mouse'], # TODO: add as needed
        default='yalla',
        help="Specify the concept of operations."
    )
        
    cli_args, ros_args = parser.parse_known_args()
    
    rclpy.init(args=ros_args)
    mission_node = MissionNode(conops=cli_args.conops)

    executor = MultiThreadedExecutor() # Or set MultiThreadedExecutor(num_threads=4)
    executor.add_node(mission_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        mission_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
