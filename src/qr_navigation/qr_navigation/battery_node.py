"""
@file battery_node.py
@author Abubakarsiddiq Navid shaikh
@date 2024-10-05
@brief Auto-generated author information
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32

class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery_node')
        self.battery_percentage = 100.0
        self.charging = False
        self.charge_sub = self.create_subscription(Int32, 'charge_battery', self.charge_callback, 10)
        self.charge_pub = self.create_publisher(Float32, 'battery', 10)
        self.timer = self.create_timer(0.25, self.publish_percentage)

    def charge_callback(self, msg):
        self.charging = (msg.data == 1)

    def publish_percentage(self):
        if self.charging:
            if self.battery_percentage <= 100.0: self.battery_percentage += 0.01
        else:
            if self.battery_percentage >= 0.0: self.battery_percentage -= 0.01
        
        msg = Float32()
        msg.data = float(self.battery_percentage)
        self.charge_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

