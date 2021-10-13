#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore

import threading


class CloudCommander(Node):
    def __init__(self):
        super().__init__("publish_cloud_commands")
        self.get_logger().info("publish_cloud_commands node started")
        self.web_app_publisher_ = self.create_publisher(Twist, "cloud_web_app_commands", 10)
        self.speech_publisher_ = self.create_publisher(Twist, "cloud_speech_commands", 10)
        self.timer_ = self.create_timer(0.05,self.publish_continously)

        self.db = firestore.client()

        self.joystick_doc_ref = self.db.collection(u'controller').document(u'joystick')
        # Watch the document
        self.joystick_doc_watch = self.joystick_doc_ref.on_snapshot(self.joystick_on_snapshot)

        self.flightmode_doc_ref = self.db.collection(u'controller').document(u'flightmode')
        # Watch the document
        self.flightmode_doc_watch = self.flightmode_doc_ref.on_snapshot(self.flightmode_on_snapshot)

        self.speech_doc_ref = self.db.collection(u'controller').document(u'speech')
        # Watch the document
        self.speech_doc_watch = self.speech_doc_ref.on_snapshot(self.speech_on_snapshot)

        self.pitch_pwm = 0.0
        self.roll_pwm = 0.0
        self.yaw_pwm = 0.0
        self.throttle_pwm = 0.0
        self.arm_mode = 0.0
        self.flight_mode = 0.0

        self.speech_activated = False
        self.offboard_h_disp = 0.0
        self.offboard_v_disp = 0.0
        self.offboard_yaw = 0.0

    # Create a callback on_snapshot function to capture changes
    def joystick_on_snapshot(self, doc_snapshot, changes, read_time):
        for doc in doc_snapshot:
            # print(f'Received document snapshot: {doc.to_dict()}')
            self.pitch_pwm = doc.to_dict()['ry']
            self.roll_pwm = doc.to_dict()['rx']
            self.throttle_pwm = doc.to_dict()['ly']
            self.yaw_pwm = doc.to_dict()['lx']

    # Create a callback on_snapshot function to capture changes
    def flightmode_on_snapshot(self, doc_snapshot, changes, read_time):
        for doc in doc_snapshot:
            # print(f'Received document snapshot: {doc.to_dict()}')
            self.arm_mode = doc.to_dict()['arm_status']
            self.flight_mode = doc.to_dict()['mode']

    # Create a callback on_snapshot function to capture changes
    def speech_on_snapshot(self, doc_snapshot, changes, read_time):
        for doc in doc_snapshot:
            # print(f'Received document snapshot: {doc.to_dict()}')
            self.speech_activated = doc.to_dict()['speech_control_activated']
            self.offboard_h_disp = doc.to_dict()['h_displacement']
            self.offboard_v_disp = doc.to_dict()['v_displacement']
            self.offboard_yaw = doc.to_dict()['yaw']
    
    def publish_continously(self):
        web_app_msg = Twist()
        web_app_msg.linear.x = float(self.pitch_pwm)
        web_app_msg.linear.y = float(self.roll_pwm)
        web_app_msg.linear.z = float(self.arm_mode)
        web_app_msg.angular.x = float(self.throttle_pwm)
        web_app_msg.angular.y = float(self.yaw_pwm)
        web_app_msg.angular.z = float(self.flight_mode)
        self.web_app_publisher_.publish(web_app_msg)

        speech_msg = Twist()
        speech_msg.linear.x = float(self.offboard_h_disp)
        speech_msg.linear.z = float(self.offboard_v_disp)
        speech_msg.angular.z = float(self.offboard_yaw)
        self.speech_publisher_.publish(speech_msg)

def main(args=None):
    # Use the application default credentials
    cred = credentials.Certificate("/home/hashim/Downloads/cloud-based--drone-firebase-adminsdk-rrbb3-b42c518c95.json")
    firebase_admin.initialize_app(cred)

    rclpy.init(args=args)
    node = CloudCommander()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()