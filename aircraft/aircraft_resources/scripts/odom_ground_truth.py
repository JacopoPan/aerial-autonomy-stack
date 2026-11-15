# Simulation only: subscribes to the drone's ground truth (/odom_ground_truth, Gazebo's pose bridged by simulation.yml.erb)
# and publishes /odom_ground_truth/path in frame_id (the ODOM backend's fixed frame in RViz), starting from the drone's pose at rest, at frame_id's origin
# With body_frame (the backend's TF frame of the drone), the path restarts, once, from the backend's pose of it, when that appears on TF
# With odom_topic (the backend's odometry, in frame_id), it also shows the backend's position error and its RMSE since takeoff on /odom_ground_truth/error as text above the drone in RViz
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener

def qmul(a, b): # Quaternion product, [x, y, z, w]
    (ax, ay, az, aw), (bx, by, bz, bw) = a, b
    return [aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz]

def rotate(q, v): # Vector v rotated by quaternion q
    return qmul(qmul(q, v + [0.0]), [-q[0], -q[1], -q[2], q[3]])[:3]

class OdomGroundTruth(Node):
    def __init__(self):
        super().__init__('odom_ground_truth')
        self.path, self.world_to_frame = Path(), None
        self.path.header.frame_id = self.declare_parameter('frame_id', 'map').value
        self.body_frame = self.declare_parameter('body_frame', '').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Path, '/odom_ground_truth/path', 1)
        self.pose_pub = self.create_publisher(PoseStamped, '/odom_ground_truth/pose', 1)
        self.error_pub = self.create_publisher(Marker, '/odom_ground_truth/error', 1)
        self.recent, self.pending, self.flying, self.error, self.sum_sq, self.count = [], [], False, 0.0, 0.0, 0 # Ground truth and backend positions (time, xyz); error metric
        odom_topic = self.declare_parameter('odom_topic', '').value
        if odom_topic:
            self.create_subscription(Odometry, odom_topic, self.odom_callback, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom_ground_truth', self.callback, 10)

    def start_pose(self): # Where the path restarts in frame_id: the backend's pose of body_frame, the first time it is on TF (else None)
        if not self.body_frame or not self.tf_buffer.can_transform(self.path.header.frame_id, self.body_frame, Time()):
            return None
        t = self.tf_buffer.lookup_transform(self.path.header.frame_id, self.body_frame, Time()).transform
        self.get_logger().info(f'Path restarted from TF {self.path.header.frame_id} -> {self.body_frame}')
        self.body_frame = '' # Only once
        return [t.translation.x, t.translation.y, t.translation.z], [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]

    def callback(self, msg):
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
        p, q = [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]
        start = self.start_pose()
        if start is None and self.world_to_frame is None and abs(v.x) + abs(v.y) + abs(v.z) < 0.05: # Else, at rest (i.e., after the drop at spawn), frame_id's origin
            start = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]
        if start:
            start_p, start_q = start
            rotation = qmul(start_q, [-q[0], -q[1], -q[2], q[3]]) # From Gazebo's world to frame_id: the drone's current orientation becomes start_q
            self.world_to_frame = (rotation, [s - r for s, r in zip(start_p, rotate(rotation, p), strict=True)]) # And its current position becomes start_p
            self.path.poses, self.recent, self.sum_sq, self.count = [], [], 0.0, 0
        if self.world_to_frame is None:
            return
        rotation, translation = self.world_to_frame
        x, y, z = [r + t for r, t in zip(rotate(rotation, p), translation, strict=True)]
        pose = PoseStamped(header=msg.header)
        pose.header.frame_id = self.path.header.frame_id
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
        o = pose.pose.orientation
        o.x, o.y, o.z, o.w = qmul(rotation, q)
        self.pose_pub.publish(pose) # Every pose, for RViz's axes
        if not self.flying and abs(v.x) + abs(v.y) + abs(v.z) > 0.5: # Takeoff: the RMSE starts here
            self.flying, self.sum_sq, self.count = True, 0.0, 0
        self.recent = self.recent[-99:] + [(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, [x, y, z])] # The last 2s, at Gazebo's 50Hz
        self.score()
        if self.count:
            text = f'error {self.error:.2f} m, RMSE {math.sqrt(self.sum_sq / self.count):.2f} m'
            marker = Marker(header=pose.header, type=Marker.TEXT_VIEW_FACING, text=text)
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z, marker.pose.orientation.w = x, y, z + 1.0, 1.0 # 1 m above the drone
            marker.scale.z, marker.color.r, marker.color.g, marker.color.a = 1.0, 1.0, 1.0, 1.0 # 1 m tall, yellow as the path
            self.error_pub.publish(marker)
            self.get_logger().info(text, throttle_duration_sec=5.0)
        last = self.path.poses[-1].pose.position if self.path.poses else None
        if last and (x - last.x) ** 2 + (y - last.y) ** 2 + (z - last.z) ** 2 < 0.01: # Ignore poses less than 10cm apart
            return
        self.path.header.stamp = msg.header.stamp
        self.path.poses.append(pose)
        self.pub.publish(self.path)

    def odom_callback(self, msg):
        if msg.header.frame_id != self.path.header.frame_id:
            self.get_logger().warn(f'{msg.header.frame_id} is not {self.path.header.frame_id}: no error metric', once=True)
            return
        p = msg.pose.pose.position
        self.pending.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, [p.x, p.y, p.z]))

    def score(self): # Errors of the queued backend positions against the ground truth, interpolated at their times (dropped if older than 2s)
        while self.pending and self.recent and self.pending[0][0] < self.recent[-1][0]:
            t, p = self.pending.pop(0)
            i = next(i for i, (ti, _) in enumerate(self.recent) if ti > t)
            if i:
                (t0, p0), (t1, p1) = self.recent[i - 1], self.recent[i]
                self.error = math.dist(p, [a + (b - a) * (t - t0) / (t1 - t0) for a, b in zip(p0, p1, strict=True)])
                self.sum_sq, self.count = self.sum_sq + self.error ** 2, self.count + 1

rclpy.init()
rclpy.spin(OdomGroundTruth())
