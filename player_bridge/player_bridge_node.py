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




# Import Player binding
from playercpp import *
from playerc import *



###################################################################################################
def euler_to_quaternion(roll, pitch, yaw):
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
###################################################################################################
class PlayerBridgeNode(Node):
    def __init__(self):
        super().__init__('player_bridge_node')

        # --- ROS 參數 ---
        self.declare_parameter('player_host', '192.168.0.2')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')

        # --- Player 連線 ---
        host = self.get_parameter('player_host').get_parameter_value().string_value
        self.get_logger().info(f'Connecting to Player server at {host}')

        self.robot = PlayerClient(host, 6665)
        self.p2d = Position2dProxy(self.robot, 0)
        self.slam = Position2dProxy(self.robot, 2)
        self.dis = DispatcherProxy(self.robot,0);
        self.lp0 = LaserProxy(self.robot, 0);
        self.lp1 = LaserProxy(self.robot, 1);


        # Retrieve the pose of the laser with respect to its parent
        self.lp0.RequestConfigure();
        self.lp0.RequestGeom();
        pose = self.lp0.GetPose();
        maxRange = self.lp0.GetMaxRange();
        angleRes = self.lp0.GetScanRes();
        minAngle = self.lp0.GetMinAngle();
        maxAngle = self.lp0.GetMaxAngle();
        self.get_logger().info('Laser[%d] maxRange:%3.3f<m> angleRes:%.3f<rad>  minAngle:%2.3f<rad> maxAngle:%2.3f<rad> pose:(px=%.3f,py=%.3f,pz=%.3f,proll=%.3f,ppitch=%.3f,pyaw=%.3f)' %  \
            (0, maxRange, angleRes, minAngle, maxAngle, pose.px, pose.py, pose.pz, pose.proll, pose.ppitch, pose.pyaw))

        self.lp1.RequestConfigure();
        self.lp1.RequestGeom();
        pose = self.lp1.GetPose();
        maxRange = self.lp1.GetMaxRange();
        angleRes = self.lp1.GetScanRes();
        minAngle = self.lp1.GetMinAngle();
        maxAngle = self.lp1.GetMaxAngle();
        self.get_logger().info('Laser[%d] maxRange:%3.3f<m> angleRes:%.3f<rad>  minAngle:%2.3f<rad> maxAngle:%2.3f<rad> pose:(px=%.3f,py=%.3f,pz=%.3f,proll=%.3f,ppitch=%.3f,pyaw=%.3f)' %  \
            (1, maxRange, angleRes, minAngle, maxAngle, pose.px, pose.py, pose.pz, pose.proll, pose.ppitch, pose.pyaw))




        # --- ROS Publisher / Subscriber ---
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.pose_pub = self.create_publisher(Odometry, 'pose', 10)
        self.laser0_pub = self.create_publisher(LaserScan, 'laser0', 10)
        self.laser1_pub = self.create_publisher(LaserScan, 'laser1', 10)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- 速度命令 ---
        self.cmd_v = 0.0
        self.cmd_w = 0.0

        #not allow MSI AI 控制底盤
        self.dis.SetAiCmd(ctypes.c_uint(PLAYER_DISPATCH_INIT_DEST_RD).value,ctypes.c_uint(PLAYER_DISPATCH_AI_NOT_CONTROL_CHASSIS).value) 
        
        # --- Thread for Player loop ---
        self.loop_thread = threading.Thread(target=self.loop_thread_func)
        self.loop_thread.daemon = True
        self.loop_thread.start()

        self.get_logger().info("Player ROS2 bridge started.")

    # -------------------------------------------------------------------------
    def cmd_callback(self, msg: Twist):
        #self.get_logger().info("linear.x=%f angular.z=%f" % (msg.linear.x,msg.angular.z))
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z
        # 發送到 Player
        self.p2d.SetSpeed(self.cmd_v, self.cmd_w, 0.0, 0.0)



    # -------------------------------------------------------------------------
    def publish_laser(self, lp: LaserProxy, pub: rclpy.publisher.Publisher):
        if lp.IsFresh()==True:
            lp.NotFresh()
            
            r = np.array(lp.GetRangeVec())
            i = np.array(lp.GetIntensityVec())
            # 保證角度數量與距離數量一致
            th = np.linspace(lp.GetMinAngle(), lp.GetMaxAngle(), len(r))
            scan = LaserScan()
            scan.header.stamp = self.get_clock().now().to_msg()
            scan.header.frame_id = f"laser{lp.GetIndex()}"
            scan.angle_min = lp.GetMinAngle()
            scan.angle_max = lp.GetMaxAngle()
            scan.angle_increment = lp.GetScanRes()
            scan.time_increment = 0.0
            scan.scan_time = 1.0 / lp.GetScanningFrequency()
            scan.range_min = 0.0
            scan.range_max = lp.GetMaxRange()
            scan.ranges = r.tolist()
            scan.intensities = i.tolist()
            
            pub.publish(scan)
    # -------------------------------------------------------------------------
    def publish_odom(self, p2d: Position2dProxy, pub: rclpy.publisher.Publisher):
        if p2d.IsFresh()==True:
            p2d.NotFresh()
            
            
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = self.get_parameter('odom_frame').get_parameter_value().string_value
            odom_msg.child_frame_id = self.get_parameter('base_frame').get_parameter_value().string_value

            x = p2d.GetXPos()
            y = p2d.GetYPos()
            yaw = p2d.GetYaw()

            #self.get_logger().info('x=%4.3f y=%4.3f  yaw=%4.3f' % (x,y,yaw))
                
            quat = euler_to_quaternion(0, 0, yaw)
            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation.x = quat[0]
            odom_msg.pose.pose.orientation.y = quat[1]
            odom_msg.pose.pose.orientation.z = quat[2]
            odom_msg.pose.pose.orientation.w = quat[3]
            timestamp = self.get_clock().now().to_msg()
            self._tf2_publish(x, y, quat, timestamp)
            odom_msg.twist.twist.linear.x = p2d.GetXSpeed()
            odom_msg.twist.twist.linear.y = p2d.GetYSpeed()
            odom_msg.twist.twist.angular.z = p2d.GetYawSpeed()
            
            pub.publish(odom_msg)

    def _tf2_publish(self, x, y, quat, timestamp):
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = self.get_parameter('odom_frame').get_parameter_value().string_value
        t.child_frame_id = self.get_parameter('base_frame').get_parameter_value().string_value

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)
    
    # -------------------------------------------------------------------------
    def publish_slam(self, p2d: Position2dProxy, pub: rclpy.publisher.Publisher):
        if p2d.IsFresh()==True:
            p2d.NotFresh()
            
            
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = self.get_parameter('map_frame').get_parameter_value().string_value
            odom_msg.child_frame_id = self.get_parameter('base_frame').get_parameter_value().string_value

            x = p2d.GetXPos()
            y = p2d.GetYPos()
            yaw = p2d.GetYaw()

            #self.get_logger().info('x=%4.3f y=%4.3f  yaw=%4.3f' % (x,y,yaw))
                
            quat = euler_to_quaternion(0, 0, yaw)
            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation.x = quat[0]
            odom_msg.pose.pose.orientation.y = quat[1]
            odom_msg.pose.pose.orientation.z = quat[2]
            odom_msg.pose.pose.orientation.w = quat[3]

            odom_msg.twist.twist.linear.x = p2d.GetXSpeed()
            odom_msg.twist.twist.linear.y = p2d.GetYSpeed()
            odom_msg.twist.twist.angular.z = p2d.GetYawSpeed()
            
            pub.publish(odom_msg)
    # -------------------------------------------------------------------
    def loop_thread_func(self):
        while rclpy.ok():
            try:
                self.loop()
            except Exception as e:
                self.get_logger().error(f"Player loop error: {e}")


    # -------------------------------------------------------------------------
    def loop(self):
        if not self.robot.Peek(100):
            return
        self.robot.Read()
            

         # Publish Odometry
        self.publish_odom(self.p2d, self.odom_pub)

         # Publish Slam
        self.publish_slam(self.slam, self.pose_pub)
        
        # Publish lasers
        self.publish_laser(self.lp0, self.laser0_pub)
        self.publish_laser(self.lp1, self.laser1_pub)
        



    # -------------------------------------------------------------------------
    def destroy_node(self):
        self.get_logger().info("Stopping robot...")
        try:
            self.p2d.SetSpeed(0.0, 0.0, 0.0, 0.0)
            # allow MSI AI 控制底盤
            self.dis.SetAiCmd(ctypes.c_uint(PLAYER_DISPATCH_INIT_DEST_RD).value,ctypes.c_uint(PLAYER_DISPATCH_AI_CONTROL_CHASSIS).value) 
        except Exception as e:
            self.get_logger().warn(str(e))


        
        del self.lp1
        del self.lp0
        del self.dis
        del self.slam
        del self.p2d
        del self.robot

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PlayerBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
