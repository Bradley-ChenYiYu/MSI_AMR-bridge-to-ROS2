import math
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np


class LaserMergerNode(Node):
    def __init__(self):
        super().__init__('laser_merger')

        # parameters
        self.declare_parameter('input_topic1', '/laser0')
        self.declare_parameter('input_topic2', '/laser1')
        self.declare_parameter('frame_id', 'base_link')

        self.input_topic1 = self.get_parameter('input_topic1').get_parameter_value().string_value
        self.input_topic2 = self.get_parameter('input_topic2').get_parameter_value().string_value
        self.target_frame = self.get_parameter('frame_id').get_parameter_value().string_value

        self.get_logger().info(f'Listening to {self.input_topic1} and {self.input_topic2}, publishing /scan in frame {self.target_frame}')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.scan1: Optional[LaserScan] = None
        self.scan2: Optional[LaserScan] = None
        self.scan_lock = threading.Lock()

        self.sub1 = self.create_subscription(LaserScan, self.input_topic1, self.scan1_cb, 10)
        self.sub2 = self.create_subscription(LaserScan, self.input_topic2, self.scan2_cb, 10)

        self.pub = self.create_publisher(LaserScan, '/scan', 10)

        # timer to periodically attempt merge
        self.timer = self.create_timer(0.1, self.timer_cb)

    def scan1_cb(self, msg: LaserScan):
        with self.scan_lock:
            self.scan1 = msg

    def scan2_cb(self, msg: LaserScan):
        with self.scan_lock:
            self.scan2 = msg

    def timer_cb(self):
        with self.scan_lock:
            s1 = self.scan1
            s2 = self.scan2

        if s1 is None and s2 is None:
            return

        try:
            merged = self.merge_scans(s1, s2)
            if merged is not None:
                self.pub.publish(merged)
        except Exception as e:
            self.get_logger().error(f'Error merging scans: {e}')

    def transform_scan_to_target(self, scan: LaserScan) -> Optional[np.ndarray]:
        # Convert scan ranges into (x,y) points in target_frame
        if scan is None:
            return None

        try:
            # Use the scan timestamp for lookup to handle motion correctly
            # We need to convert the message time to rclpy Time if needed, 
            # but lookup_transform accepts the message type in recent versions.
            # To be safe, we use the message directly.
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                self.target_frame, scan.header.frame_id, scan.header.stamp, timeout=rclpy.duration.Duration(seconds=0.5))
        except Exception:
            # maybe no tf yet
            return None

        # Extract translation and rotation
        tx = trans.transform.translation.x
        ty = trans.transform.translation.y
        tz = trans.transform.translation.z
        q = trans.transform.rotation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

        # Construct 4x4 transformation matrix
        # Rotation matrix from quaternion
        # 1 - 2y^2 - 2z^2   2xy - 2zw       2xz + 2yw
        # 2xy + 2zw         1 - 2x^2 - 2z^2 2yz - 2xw
        # 2xz - 2yw         2yz + 2xw       1 - 2x^2 - 2y^2
        
        R = np.array([
            [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
        ])
        
        T = np.array([tx, ty, tz])

        # Vectorized point generation
        ranges = np.array(scan.ranges)
        # Angles corresponding to each range
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        
        # Filter valid ranges
        valid_mask = (ranges >= scan.range_min) & (ranges <= scan.range_max) & np.isfinite(ranges)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        if len(valid_ranges) == 0:
            return np.empty((0, 2))

        # Convert to Cartesian in scan frame
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        z = np.zeros_like(x)
        
        # Stack into (3, N) array
        points = np.vstack((x, y, z))
        
        # Apply rotation and translation: P_target = R * P_source + T
        # R is (3,3), points is (3,N), T is (3,) broadcasted
        points_transformed = R @ points + T[:, np.newaxis]
        
        # Return (x, y) columns
        return points_transformed[:2, :].T

    def merge_scans(self, s1: Optional[LaserScan], s2: Optional[LaserScan]) -> Optional[LaserScan]:
        # Strategy: sample combined angular resolution and pick nearest range per angle
        scans = [s for s in (s1, s2) if s is not None]
        if not scans:
            return None

        # Transform all valid points into target frame with their angles
        pts_per_scan = []
        angles_per_scan = []
        for s in scans:
            pts = self.transform_scan_to_target(s)
            if pts is None:
                return None
            pts_per_scan.append(pts)

            # reconstruct angle for each point relative to target frame
            angles = []
            ang = s.angle_min
            for r in s.ranges:
                if math.isfinite(r) and r >= s.range_min and r <= s.range_max:
                    # compute angle in source frame then rotate by transform yaw
                    angles.append(ang)
                ang += s.angle_increment
            angles_per_scan.append(np.array(angles))

        # Build polar representation in target frame: compute angle and range for each point
        merged_angles = []
        merged_ranges = []
        for pts in pts_per_scan:
            for (x, y) in pts:
                a = math.atan2(y, x)
                r = math.hypot(x, y)
                merged_angles.append(a)
                merged_ranges.append(r)

        if not merged_angles:
            return None

        # choose resolution from first available scan
        ref = scans[0]
        ang_min = -math.pi
        ang_max = math.pi
        ang_inc = ref.angle_increment
        num = int(math.ceil((ang_max - ang_min) / ang_inc))
        grid_angles = ang_min + np.arange(num) * ang_inc

        grid_ranges = np.full((num,), float('inf'))
        for a, r in zip(merged_angles, merged_ranges):
            # find nearest bin
            idx = int(round((a - ang_min) / ang_inc))
            if 0 <= idx < num:
                if r < grid_ranges[idx]:
                    grid_ranges[idx] = r

        # clip inf to max range
        max_range = max((s.range_max for s in scans))
        grid_ranges = np.where(np.isfinite(grid_ranges), grid_ranges, float('nan'))

        out = LaserScan()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.target_frame
        out.angle_min = ang_min
        out.angle_max = ang_max - ang_inc
        out.angle_increment = ang_inc
        out.time_increment = ref.time_increment
        out.scan_time = ref.scan_time
        out.range_min = min((s.range_min for s in scans))
        out.range_max = max((s.range_max for s in scans))
        out.ranges = list(grid_ranges)
        out.intensities = []

        return out


def main(args=None):
    rclpy.init(args=args)
    node = LaserMergerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
