#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random

class PublishContinously(Node):
    def __init__(self):
        super().__init__("publish_joystick_continously")
        self.get_logger().info("publish_joystick_continously node started")
        self.subscriber_ = self.create_subscription(Twist,"cmd_vel",self.callback_joystick,10)
        self.publisher_ = self.create_publisher(Twist, "cmd_vel_continous", 10)
        self.timer_ = self.create_timer(0.05,self.publish_continously)

        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.throttle = 0.0
        self.change_mode = 0.0

    def callback_joystick(self, data):
        self.pitch = data.linear.x
        self.roll = data.linear.y
        self.change_mode = data.linear.z
        self.throttle = data.angular.x
        self.yaw = data.angular.y    
    
    def publish_continously(self):
        msg = Twist()
        msg.linear.x = self.pitch
        msg.linear.y = self.roll
        msg.linear.z = self.change_mode
        msg.angular.x = self.throttle
        msg.angular.y = self.yaw
        msg.angular.z = random.uniform(0, 1)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PublishContinously()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()