#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore

import datetime
import time

class CloudInformer(Node):
    def __init__(self):
        super().__init__("cloud_informer")
        self.get_logger().info("cloud_informer node started")
        self.db = firestore.client()
        self.telemetryCollectionRef = self.db.collection(u'telemetry')

        self.ts = time.time()
        self.latitude = 0.0
        self.longitude = 0.0
        self.amsl = 0.0
        self.groundspeed = 0.0
        self.yaw = 0.0
        self.subscription_started = False

        self.subscriber_ = self.create_subscription(Twist,"cloud_telemetry",self.callback_cloud_telemetry,10)
        self.timer_ = self.create_timer(2,self.publish_to_cloud_continously) # publish data to cloud every 2 seconds
        
    def callback_cloud_telemetry(self, msg):
        self.latitude = msg.linear.x
        self.longitude = msg.linear.y
        self.amsl = msg.linear.z
        self.groundspeed = msg.angular.x
        self.yaw = msg.angular.y
        self.subscription_started = True

    def publish_to_cloud_continously(self):
        if self.subscription_started:
            self.ts = time.time()
            self.telemetryCollectionRef.document(str(int(self.ts))).set({
                u'id' :int(self.ts),
                u'gps_location': firebase_admin.firestore.GeoPoint(self.latitude, self.longitude),
                u'altitude_amsl': self.amsl,
                u'groundspeed': self.groundspeed,
                u'yaw': self.yaw,
                u'timestamp': datetime.datetime.now()
            })



def main(args=None):
    # Use the application default credentials
    cred = credentials.Certificate("/home/hashim/Downloads/cloud-based--drone-firebase-adminsdk-rrbb3-b42c518c95.json")
    firebase_admin.initialize_app(cred)

    rclpy.init(args=args)
    node = CloudInformer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()