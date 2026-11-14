# Simulation only: subscribes to the drone's ground truth (/odom_ground_truth, Gazebo's pose bridged by simulation.yml.erb)
# and publishes /odom_ground_truth/path in frame_id (the ODOM backend's fixed frame in RViz), starting from the drone's pose at rest, at frame_id's origin;
# with body_frame (the backend's TF frame of the drone), the path restarts, once, from the backend's pose of it, when that appears on TF
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
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
            self.world_to_frame = (rotation, [s - r for s, r in zip(start_p, rotate(rotation, p))]) # And its current position becomes start_p
            self.path.poses = []
        if self.world_to_frame is None:
            return
        rotation, translation = self.world_to_frame
        x, y, z = [r + t for r, t in zip(rotate(rotation, p), translation)]
        last = self.path.poses[-1].pose.position if self.path.poses else None
        if last and (x - last.x) ** 2 + (y - last.y) ** 2 + (z - last.z) ** 2 < 0.01: # Ignore poses less than 10cm apart
            return
        pose = PoseStamped(header=msg.header)
        pose.header.frame_id = self.path.header.frame_id
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
        o = pose.pose.orientation
        o.x, o.y, o.z, o.w = qmul(rotation, q)
        self.path.header.stamp = msg.header.stamp
        self.path.poses.append(pose)
        self.pub.publish(self.path)

rclpy.init()
rclpy.spin(OdomGroundTruth())
