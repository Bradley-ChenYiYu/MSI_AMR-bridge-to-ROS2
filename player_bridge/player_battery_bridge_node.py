#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from msi_amr_interface.msg import BatteryState

# Import Player binding
from playercpp import *
from playerc import *


class PlayerBatteryBridgeNode(Node):
    def __init__(self):
        super().__init__('player_battery_bridge_node')

        # --- ROS Parameters ---
        self.declare_parameter('player_host', '192.168.88.8')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate', 1.0)  # Hz

        host = self.get_parameter('player_host').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # --- Player Connection ---
        self.get_logger().info(f'Connecting to Power server at {host}')
        self.robot = PlayerClient(host)

        # Power proxy for battery data
        self.power = PowerProxy(self.robot, 0)

        # --- ROS Publishers ---
        self.battery_pub = self.create_publisher(BatteryState, 'battery_state', 10)

        # --- Timer ---
        # self._lock = threading.Lock()
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

        self.get_logger().info('PlayerBatteryBridgeNode started.')

    def timer_callback(self):
        try:
            self.robot.Read()

            msg = BatteryState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.base_frame

            msg.percent = self.power.GetPercent()               # simulate draining battery
            msg.charge = self.power.GetCharge()                 # V
            msg.watts = self.power.GetWatts()                   # W
            msg.runtime = self.power.GetRunTime()/3600.0        # hour
            msg.residuallife = self.power.GetResidualLife()     # 

            self.battery_pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'Battery read error: {e}')

    def destroy_node(self):
        self.get_logger().info('Shutting down PlayerBatteryBridgeNode.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PlayerBatteryBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

