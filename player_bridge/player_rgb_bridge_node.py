#!/usr/bin/env python3
#
#ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.2}}"
#
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import math
import numpy as np
import time
import ctypes
import os
import sys
from ament_index_python.packages import get_package_share_directory
import threading
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import ctypes

# Import Player binding
from playercpp import *
from playerc import *

class PlayerRgbBridgeNode(Node):
    def __init__(self):
        super().__init__('PlayerRgbBridgeNode')
        self.get_logger().info("Test node for PlayerRgbBridgeNode has been started!")

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('enable_cam0', True)
        self.declare_parameter('enable_cam1', True)
        self.declare_parameter('enable_cam2', True)

        self.declare_parameter('player_host', '192.168.88.8')
        self.enable_cam0 = self.get_parameter('enable_cam0').get_parameter_value().bool_value
        self.enable_cam1 = self.get_parameter('enable_cam1').get_parameter_value().bool_value
        self.enable_cam2 = self.get_parameter('enable_cam2').get_parameter_value().bool_value

        host = self.get_parameter('player_host').get_parameter_value().string_value
        self.get_logger().info(f'Connecting to RGB Player server at {host}')

        self.robot1 = PlayerClient(host, 6667)
        self.robot1.SetRequestTimeout(30) #Set the timeout for client requests

        self.cam0 = CameraProxy(self.robot1, 4) if self.enable_cam0 else None
        self.cam1 = CameraProxy(self.robot1, 14) if self.enable_cam1 else None
        self.cam2 = CameraProxy(self.robot1, 24) if self.enable_cam2 else None

        self.static_transforms = []
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        if self.enable_cam0:
            self.cam0.RequestGeom()  
            self.cam0.RequestIntrinsics()
            camGeom=self.cam0.GetPoseVect()
            camIntrinsics=self.cam0.GetIntrinsicsVect()
            self.cam0_intrinsics = camIntrinsics
            camIndex = self.cam0.GetIndex()
            self.get_logger().info("Camera[%d] camGeom px=%f py=%f pz=%f proll=%f ppitch=%f pyaw=%f" % (camIndex,camGeom[0],camGeom[1],camGeom[2],camGeom[3],camGeom[4],camGeom[5]) )
            self.get_logger().info("Camera[%d] camIntrinsics ppx=%f ppy=%f fx=%f fy=%f " % (camIndex,camIntrinsics[0],camIntrinsics[1],camIntrinsics[2],camIntrinsics[3]))
            self._publish_static_tf2(camGeom[0], camGeom[1], camGeom[2], camGeom[3], camGeom[4], camGeom[5], child_frame_id=f"camera{camIndex}", frame_id=self.get_parameter('base_frame').get_parameter_value().string_value)
        
        
        if self.enable_cam1:
            self.cam1.RequestGeom()  
            self.cam1.RequestIntrinsics()
            camGeom=self.cam1.GetPoseVect()
            camIntrinsics=self.cam1.GetIntrinsicsVect()
            self.cam1_intrinsics = camIntrinsics
            camIndex = self.cam1.GetIndex()
            self.get_logger().info("Camera[%d] camGeom px=%f py=%f pz=%f proll=%f ppitch=%f pyaw=%f" % (camIndex,camGeom[0],camGeom[1],camGeom[2],camGeom[3],camGeom[4],camGeom[5]) )
            self.get_logger().info("Camera[%d] camIntrinsics ppx=%f ppy=%f fx=%f fy=%f " % (camIndex,camIntrinsics[0],camIntrinsics[1],camIntrinsics[2],camIntrinsics[3]))
            self._publish_static_tf2(camGeom[0], camGeom[1], camGeom[2], camGeom[3], camGeom[4], camGeom[5], child_frame_id=f"camera{camIndex}", frame_id=self.get_parameter('base_frame').get_parameter_value().string_value)
        
            
        if self.enable_cam2:
            self.cam2.RequestGeom()  
            self.cam2.RequestIntrinsics()
            camGeom=self.cam2.GetPoseVect()
            camIntrinsics=self.cam2.GetIntrinsicsVect()
            self.cam2_intrinsics = camIntrinsics
            camIndex = self.cam2.GetIndex()
            self.get_logger().info("Camera[%d] camGeom px=%f py=%f pz=%f proll=%f ppitch=%f pyaw=%f" % (camIndex,camGeom[0],camGeom[1],camGeom[2],camGeom[3],camGeom[4],camGeom[5]) )
            self.get_logger().info("Camera[%d] camIntrinsics ppx=%f ppy=%f fx=%f fy=%f " % (camIndex,camIntrinsics[0],camIntrinsics[1],camIntrinsics[2],camIntrinsics[3]))
            self._publish_static_tf2(camGeom[0], camGeom[1], camGeom[2], camGeom[3], camGeom[4], camGeom[5], child_frame_id=f"camera{camIndex}", frame_id=self.get_parameter('base_frame').get_parameter_value().string_value)
        
        if self.enable_cam0:
            self.cam0_pub = self.create_publisher(Image, 'camera0', 10)
            self.cam0_info_pub = self.create_publisher(CameraInfo, 'camera0/camera_info', 10)
        if self.enable_cam1:
            self.cam1_pub = self.create_publisher(Image, 'camera1', 10)
            self.cam1_info_pub = self.create_publisher(CameraInfo, 'camera1/camera_info', 10)
        if self.enable_cam2:
            self.cam2_pub = self.create_publisher(Image, 'camera2', 10)
            self.cam2_info_pub = self.create_publisher(CameraInfo, 'camera2/camera_info', 10)

        self.my_callback_group = ReentrantCallbackGroup()
        if self.enable_cam0:
            self.timer_cam0 = self.create_timer(0.01, self.timer_callback_cam0,
                                       callback_group=self.my_callback_group)
        if self.enable_cam1:
            self.timer_cam1 = self.create_timer(0.01, self.timer_callback_cam1,
                                       callback_group=self.my_callback_group)
        if self.enable_cam2:
            self.timer_cam2 = self.create_timer(0.01, self.timer_callback_cam2,
                                       callback_group=self.my_callback_group)

        self.get_logger().info("Player RGB ROS2 bridge started.")

    def timer_callback_cam0(self):
        if self.robot1.Peek(100) and self.enable_cam0:
            self.robot1.Read()
            self.publish_cam(self.cam0,self.cam0_pub, self.cam0_info_pub, self.cam0_intrinsics)
            
    def timer_callback_cam1(self):
        if self.robot1.Peek(100) and self.enable_cam1:
            self.robot1.Read()
            self.publish_cam(self.cam1,self.cam1_pub, self.cam1_info_pub, self.cam1_intrinsics)
            
    def timer_callback_cam2(self):
        if self.robot1.Peek(100) and self.enable_cam2:
            self.robot1.Read()
            self.publish_cam(self.cam2,self.cam2_pub, self.cam2_info_pub, self.cam2_intrinsics)
            
    def publish_cam(self, cam: CameraProxy, pub: rclpy.publisher.Publisher, info_pub: rclpy.publisher.Publisher, intrinsics):
        if cam.IsFresh()==True:
            cam.NotFresh()
            
            img_msg = Image()
            
            img_msg.header.stamp = self.get_clock().now().to_msg()
            # img_msg.header.frame_id = "camera"
            img_msg.header.frame_id = f"camera{cam.GetIndex()}"
            
            img_msg.height = int(cam.GetHeight())
            img_msg.width = int(cam.GetWidth())
            img_msg.encoding = "rgb8"
            img_msg.is_bigendian = 0
            img_msg.step = img_msg.width * 3   # 假設 3 channel
            
            cam.Decompress()
            imgAdd=cam.GetImage()
            imgRaw = (ctypes.c_uint8 * img_msg.height * img_msg.width  * 3).from_address(int(imgAdd))
            img_msg.data = bytes(imgRaw)     # numpy frame -> byte array
            
            
            
            pub.publish(img_msg)

            # Publish CameraInfo
            info_msg = CameraInfo()
            info_msg.header = img_msg.header
            info_msg.height = img_msg.height
            info_msg.width = img_msg.width
            info_msg.distortion_model = "plumb_bob"
            
            # intrinsics: [ppx, ppy, fx, fy]
            ppx = intrinsics[0]
            ppy = intrinsics[1]
            fx = intrinsics[2]
            fy = intrinsics[3]
            
            # K: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            info_msg.k = [fx, 0.0, ppx, 0.0, fy, ppy, 0.0, 0.0, 1.0]
            
            # D: [0, 0, 0, 0, 0] (assuming no distortion)
            info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            
            # R: Identity
            info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            
            # P: [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
            info_msg.p = [fx, 0.0, ppx, 0.0, 0.0, fy, ppy, 0.0, 0.0, 0.0, 1.0, 0.0]
            
            info_pub.publish(info_msg)
    
    def _publish_static_tf2(self, x, y, z, roll, pitch, yaw, frame_id, child_frame_id):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id

        quat = self.euler_to_quaternion(roll, pitch, yaw)

        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # Remove existing transform for this child frame if it exists to avoid duplicates
        self.static_transforms = [tr for tr in self.static_transforms if tr.child_frame_id != child_frame_id]
        self.static_transforms.append(t)

        # Publish the accumulated list of transforms
        self.tf_static_broadcaster.sendTransform(self.static_transforms)
        self.get_logger().info(f'Published static transform from {t.header.frame_id} to {t.child_frame_id}')

    def euler_to_quaternion(self, roll, pitch, yaw):
        cr = math.cos(roll/2)
        sr = math.sin(roll/2)
        cp = math.cos(pitch/2)
        sp = math.sin(pitch/2)
        cy = math.cos(yaw/2)
        sy = math.sin(yaw/2)

        w = cr*cp*cy + sr*sp*sy
        x = sr*cp*cy - cr*sp*sy
        y = cr*sp*cy + sr*cp*sy
        z = cr*cp*sy - sr*sp*cy

        return x, y, z, w

def main(args=None):
    # TODO: Use MultiThreadedExecutor to handle multiple cameras
    rclpy.init(args=args)
    # executor = rclpy.executors.MultiThreadedExecutor()
    node = PlayerRgbBridgeNode()
    # executor.add_node(node)
    try:
        rclpy.spin(node)
        # executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # executor.shutdown()
        # executor.destroy_node() 
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()

