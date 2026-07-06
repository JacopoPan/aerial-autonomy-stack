import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

import os
import argparse
import threading
import yaml
import py_trees
import py_trees_ros

from mission import tree_builder

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
                self.mission_plan = yaml.safe_load(f)
                self.get_logger().info("Loaded mission plan.")
        except Exception as e:
            self.get_logger().error(f"Failed to load mission file: {e}")
            self.mission_plan = []

        self.own_drone_id = None
        drone_id_str = os.environ.get('DRONE_ID') # Get id from ENV VAR
        if drone_id_str is None:
            self.get_logger().info("DRONE_ID environment variable not set.")
        else:
            try:
                self.own_drone_id = int(drone_id_str)
            except ValueError:
                self.get_logger().info(f"Could not parse DRONE_ID='{drone_id_str}' as an integer.")

        self.blackboard = py_trees.blackboard.Blackboard()
        if self.own_drone_id is not None:
            self.blackboard.set("own_drone_id", self.own_drone_id)

        self.data_lock = threading.Lock()
        # Initialize Blackboard variables
        self.blackboard.set("lat", None)
        self.blackboard.set("lon", None)
        self.blackboard.set("alt_msl", None)
        self.blackboard.set("heading", None)
        self.blackboard.set("airspeed", None)
        # Perception
        self.blackboard.set("yolo_detections", None)
        self.blackboard.set("ground_tracks", None)
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
        self.create_subscription( # 10Hz
            NavSatFix, '/mavros/global_position/global', self.mavros_global_position_callback,
            self.qos_profile, callback_group=self.subscriber_callback_group)
        self.create_subscription( # 10Hz
            VfrHud, '/mavros/vfr_hud', self.vfr_hud_callback,
            self.qos_profile, callback_group=self.subscriber_callback_group)
        # Perception subscribers
        self.create_subscription( # 15Hz
            Detection2DArray, '/detections', self.yolo_detections_callback,
            self.qos_profile, callback_group=self.subscriber_callback_group)
        self.create_subscription( # 1Hz
            SwarmObs, '/tracks', self.ground_tracks_callback,
            self.qos_profile, callback_group=self.subscriber_callback_group)

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

        # Actions
        self.blackboard.set("takeoff_client", ActionClient(self, Takeoff, 'takeoff_action', callback_group=self.action_callback_group))
        self.blackboard.set("land_client", ActionClient(self, Land, 'land_action', callback_group=self.action_callback_group))
        self.blackboard.set("orbit_client", ActionClient(self, Orbit, 'orbit_action', callback_group=self.action_callback_group))
        self.blackboard.set("offboard_client", ActionClient(self, Offboard, 'offboard_action', callback_group=self.action_callback_group))

        # Services
        if self.own_drone_id is not None:
            self.blackboard.set("speed_client", self.create_client(SetSpeed, f'/Drone{self.own_drone_id}/set_speed', callback_group=self.service_callback_group))
            self.blackboard.set("reposition_client", self.create_client(SetReposition, f'/Drone{self.own_drone_id}/set_reposition', callback_group=self.service_callback_group))
        else:
            self.get_logger().info("DRONE_ID not set, service clients not created.")

        # Mission as a 2Hz behavior tree
        if self.mission_plan:
            self.root_node = tree_builder.create_mission_tree(self.mission_plan, self)
            self.behaviour_tree = py_trees_ros.trees.BehaviourTree(self.root_node)
            self.behaviour_tree.setup(node=self)
            self.tree_timer = self.create_timer(0.5, self.tick_tree, callback_group=self.timer_callback_group)
        else:
            self.get_logger().error("Mission plan is empty. Tree not built.")

    def px4_global_position_callback(self, msg): # Mutally exclusive with mavros_global_position_callback
        with self.data_lock:
            self.blackboard.set("lat", msg.lat)
            self.blackboard.set("lon", msg.lon)
            self.blackboard.set("alt_msl", msg.alt)

    def airspeed_validated_callback(self, msg): # Mutally exclusive with vfr_hud_callback
        with self.data_lock:
            self.blackboard.set("airspeed", msg.true_airspeed_m_s)

    def mavros_global_position_callback(self, msg):  # Mutally exclusive with px4_global_position_callback
        with self.data_lock:
            self.blackboard.set("lat", msg.latitude)
            self.blackboard.set("lon", msg.longitude)

    def vfr_hud_callback(self, msg): # Mutally exclusive with airspeed_validated_callback
        with self.data_lock:
            self.blackboard.set("alt_msl", msg.altitude)
            self.blackboard.set("heading", msg.heading)
            self.blackboard.set("airspeed", msg.airspeed)

    def yolo_detections_callback(self, msg):
        if msg.header.frame_id == "camera_frame_0": # Only process the primary camera
            with self.data_lock:
                self.blackboard.set("yolo_detections", msg)

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
            for drone_id, (_last_msg, last_seen_time) in self.drone_states.items():
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
                self.blackboard.set("drone_states", self.drone_states)

    def state_sharing_callback(self, msg):
        # A single callback for all drone state topics
        with self.data_lock:
            now = self.get_clock().now()
            self.drone_states[msg.drone_id] = (msg, now)
            self.blackboard.set("drone_states", self.drone_states)

    def ground_tracks_callback(self, msg):
        with self.data_lock:
            self.blackboard.set("ground_tracks", msg)

    def printout_callback(self):
        with self.data_lock: # Copy with lock
            lat = self.blackboard.get("lat")
            lon = self.blackboard.get("lon")
            alt_msl = self.blackboard.get("alt_msl")
            yolo_detections = self.blackboard.get("yolo_detections")
            ground_tracks = self.blackboard.get("ground_tracks")
            states_copy = self.drone_states.copy()
            try:
                active_node = self.behaviour_tree.root.tip().name if self.behaviour_tree.root else "None"
            except:
                active_node = "Initializing"
        now_seconds = self.get_clock().now().nanoseconds / 1e9
        output = f"\nCurrent node time: {now_seconds:.2f} seconds\n"
        output += f"Active step: {active_node}\n"
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

        if ground_tracks and ground_tracks.tracks:
            output += "Ground Tracks:\n"
            for track in ground_tracks.tracks:
                output += f"  Id {track.id}, lat: {track.latitude_deg:.5f} lon: {track.longitude_deg:.5f} alt (msl): {track.altitude_m:.2f}\n"
        else:
            output += "Ground Tracks: [No data]\n"
        
        self.get_logger().info(output)

    def tick_tree(self):
        # Ticks the behavior tree and monitors its overall status
        with self.data_lock: # Lock the data while the tree evaluates the blackboard
            self.behaviour_tree.tick()
        status = self.behaviour_tree.root.status
        if status == py_trees.common.Status.SUCCESS:
            self.get_logger().info("Mission Complete!")
            self.tree_timer.cancel()
            rclpy.shutdown()
        elif status == py_trees.common.Status.FAILURE:
            self.get_logger().info("Mission Failed!")
            self.tree_timer.cancel()
            rclpy.shutdown()

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
