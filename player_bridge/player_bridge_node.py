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
import ctypes




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
        
        self.robot1 = PlayerClient(host, 6667)
        self.robot1.SetRequestTimeout(30) #Set the timeout for client requests
        
        self.p2d = Position2dProxy(self.robot, 0)
        self.slam = Position2dProxy(self.robot, 2)
        self.dis = DispatcherProxy(self.robot,0)
        self.lp0 = LaserProxy(self.robot, 0)
        self.lp1 = LaserProxy(self.robot, 1)
        

        self.cam0 = CameraProxy(self.robot1, 4)
        self.cam1 = CameraProxy(self.robot1, 14)

        
        
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
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
        self._publish_static_tf2(pose.px, pose.py, pose.pz, pose.proll, pose.ppitch, pose.pyaw, child_frame_id='laser0', frame_id=self.get_parameter('base_frame').get_parameter_value().string_value)

        self.lp1.RequestConfigure();
        self.lp1.RequestGeom();
        pose = self.lp1.GetPose();
        maxRange = self.lp1.GetMaxRange();
        angleRes = self.lp1.GetScanRes();
        minAngle = self.lp1.GetMinAngle();
        maxAngle = self.lp1.GetMaxAngle();
        self.get_logger().info('Laser[%d] maxRange:%3.3f<m> angleRes:%.3f<rad>  minAngle:%2.3f<rad> maxAngle:%2.3f<rad> pose:(px=%.3f,py=%.3f,pz=%.3f,proll=%.3f,ppitch=%.3f,pyaw=%.3f)' %  \
            (1, maxRange, angleRes, minAngle, maxAngle, pose.px, pose.py, pose.pz, pose.proll, pose.ppitch, pose.pyaw))
        self._publish_static_tf2(pose.px, pose.py, pose.pz, pose.proll, pose.ppitch, pose.pyaw, child_frame_id='laser1', frame_id=self.get_parameter('base_frame').get_parameter_value().string_value)


        self.cam0.RequestGeom()  
        self.cam0.RequestIntrinsics()
        camGeom=self.cam0.GetPoseVect()
        camIntrinsics=self.cam0.GetIntrinsicsVect()
        print("Camera[%d] camGeom px=%f py=%f pz=%f proll=%f ppitch=%f pyaw=%f" % (0,camGeom[0],camGeom[1],camGeom[2],camGeom[3],camGeom[4],camGeom[5]) )
        print("Camera[%d] camIntrinsics ppx=%f ppy=%f fx=%f fy=%f " % (0,camIntrinsics[0],camIntrinsics[1],camIntrinsics[2],camIntrinsics[3]))
        
        
        self.cam1.RequestGeom()  
        self.cam1.RequestIntrinsics()
        camGeom=self.cam1.GetPoseVect()
        camIntrinsics=self.cam1.GetIntrinsicsVect()
        print("Camera[%d] camGeom px=%f py=%f pz=%f proll=%f ppitch=%f pyaw=%f" % (1,camGeom[0],camGeom[1],camGeom[2],camGeom[3],camGeom[4],camGeom[5]) )
        print("Camera[%d] camIntrinsics ppx=%f ppy=%f fx=%f fy=%f " % (1,camIntrinsics[0],camIntrinsics[1],camIntrinsics[2],camIntrinsics[3]))        self._publish_static_tf2(pose.px, pose.py, pose.pz, pose.proll, pose.ppitch, pose.pyaw, child_frame_id='laser1', frame_id=self.get_parameter('base_frame').get_parameter_value().string_value)




        
        # --- ROS Publisher / Subscriber ---
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.pose_pub = self.create_publisher(Odometry, 'pose', 10)
        self.laser0_pub = self.create_publisher(LaserScan, 'laser0', 10)
        self.laser1_pub = self.create_publisher(LaserScan, 'laser1', 10)
        self.cam0_pub = self.create_publisher(Image, 'camera0', 10)
        self.cam1_pub = self.create_publisher(Image, 'camera1', 10)
        
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Save odom data for tf publishing ---
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_timestamp = None

        # --- 速度命令 ---
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        

        #not allow MSI AI 控制底盤
        self.dis.SetAiCmd(ctypes.c_uint(PLAYER_DISPATCH_INIT_DEST_RD).value,ctypes.c_uint(PLAYER_DISPATCH_AI_NOT_CONTROL_CHASSIS).value) 
        
        # --- Thread for Player loop ---
        self.loop_thread = threading.Thread(target=self.loop_thread_func)
        self.loop_thread.daemon = True
        self.loop_thread.start()

        # --- Thread for Player camera loop ---
        self.loop1_thread = threading.Thread(target=self.loop1_thread_func)
        self.loop1_thread.daemon = True
        self.loop1_thread.start()
        
        # --- Thread for publishing tf2 ---
        self.tf2_thread = threading.Thread(target=self.publish_tf2_func)
        self.tf2_thread.daemon = True
        self.tf2_thread.start()

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
        # self.get_logger().info(f"Laser data {lp.IsFresh()}")
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
            # scan.ranges = r.tolist()
            # scan.intensities = i.tolist()
            
            # Sanitize ranges: ensure finite floats and within [range_min, range_max]
            ranges = []
            for val in r:
                try:
                    f = float(val)
                except Exception:
                    f = float('nan')
                if math.isnan(f) or math.isinf(f):
                    # replace invalid range with max range (interpreted as no return)
                    f = float(scan.range_max)
                # clamp to [range_min, range_max]
                if f < scan.range_min:
                    f = float(scan.range_min)
                if f > scan.range_max:
                    f = float(scan.range_max)
                ranges.append(f)

            # Sanitize intensities: ensure floats; if missing or invalid use 0.0
            intensities = []
            # If intensity vector is shorter or longer than ranges, align by index
            for idx in range(len(ranges)):
                if idx < len(i):
                    try:
                        iv = float(i[idx])
                        if math.isnan(iv) or math.isinf(iv):
                            iv = 0.0
                    except Exception:
                        iv = 0.0
                else:
                    iv = 0.0
                intensities.append(iv)

            scan.ranges = ranges
            scan.intensities = intensities
            
            
            pub.publish(scan)
            # self.get_logger().info("Sending Laser scan")
    # -------------------------------------------------------------------------
    def publish_odom(self, p2d: Position2dProxy, pub: rclpy.publisher.Publisher):
        if p2d.IsFresh()==True:
            p2d.NotFresh()
            
            
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = self.get_parameter('odom_frame').get_parameter_value().string_value
            odom_msg.child_frame_id = self.get_parameter('base_frame').get_parameter_value().string_value

            self.x = p2d.GetXPos()
            self.y = p2d.GetYPos()
            self.yaw = p2d.GetYaw()
            self.odom_timestamp = self.get_clock().now().to_msg()

            #self.get_logger().info('x=%4.3f y=%4.3f  yaw=%4.3f' % (x,y,yaw))
                
            quat = euler_to_quaternion(0, 0, self.yaw)
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation.x = quat[0]
            odom_msg.pose.pose.orientation.y = quat[1]
            odom_msg.pose.pose.orientation.z = quat[2]
            odom_msg.pose.pose.orientation.w = quat[3]            
            odom_msg.twist.twist.linear.x = p2d.GetXSpeed()
            odom_msg.twist.twist.linear.y = p2d.GetYSpeed()
            odom_msg.twist.twist.angular.z = p2d.GetYawSpeed()
            
            pub.publish(odom_msg)

    def publish_tf2(self):
        if self.odom_timestamp is None:
            return
        t = TransformStamped()
        t.header.stamp = self.odom_timestamp
        t.header.frame_id = self.get_parameter('odom_frame').get_parameter_value().string_value
        t.child_frame_id = self.get_parameter('base_frame').get_parameter_value().string_value

        quat = euler_to_quaternion(0, 0, self.yaw)
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

    def _publish_static_tf2(self, x, y, z, roll, pitch, yaw, frame_id, child_frame_id):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id

        quat = euler_to_quaternion(roll, pitch, yaw)

        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_static_broadcaster.sendTransform(t)
        self.get_logger().info(f'Published static transform from {t.header.frame_id} to {t.child_frame_id}')
    
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
            
    # -------------------------------------------------------------------------
    def publish_cam(self, cam: CameraProxy, pub: rclpy.publisher.Publisher):
        if cam.IsFresh()==True:
            cam.NotFresh()
            
            img_msg = Image()
            
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "camera"
            
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
    # -------------------------------------------------------------------
    def loop_thread_func(self):
        while rclpy.ok():
            try:
                self.loop()
            except Exception as e:
                self.get_logger().error(f"Player loop error: {e}")

    def loop1_thread_func(self):
        while rclpy.ok():
            try:
                self.loop1()
            except Exception as e:
                self.get_logger().error(f"Player loop1 error: {e}")

    def publish_tf2_func(self):
        while rclpy.ok():
            try:
                self.publish_tf2()
            except Exception as e:
                self.get_logger().error(f"TF2 publish error: {e}")
            # time.sleep(0.05)  # 20 Hz


    # -------------------------------------------------------------------------
    def loop(self):
        if self.robot.Peek(100):
            self.robot.Read()
            
             # Publish Odometry
            self.publish_odom(self.p2d, self.odom_pub)
    
             # Publish Slam
            self.publish_slam(self.slam, self.pose_pub)
            
            # Publish lasers
            self.publish_laser(self.lp0, self.laser0_pub)
            self.publish_laser(self.lp1, self.laser1_pub)
        
    # -------------------------------------------------------------------------
    def loop1(self):
        if self.robot1.Peek(100):
            self.robot1.Read()
            
            # Publish camera
            self.publish_cam(self.cam0,self.cam0_pub)
            self.publish_cam(self.cam1,self.cam1_pub)
    # -------------------------------------------------------------------------
    def destroy_node(self):
        self.get_logger().info("Stopping robot...")
        try:
            self.p2d.SetSpeed(0.0, 0.0, 0.0, 0.0)
            # allow MSI AI 控制底盤
            self.dis.SetAiCmd(ctypes.c_uint(PLAYER_DISPATCH_INIT_DEST_RD).value,ctypes.c_uint(PLAYER_DISPATCH_AI_CONTROL_CHASSIS).value) 
        except Exception as e:
            self.get_logger().warn(str(e))


        
        del self.cam1
        del self.cam0
        del self.lp1
        del self.lp0
        del self.dis
        del self.slam
        del self.p2d
        del self.robot1
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
