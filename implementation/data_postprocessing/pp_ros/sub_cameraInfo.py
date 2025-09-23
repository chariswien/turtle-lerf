#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo

import json
import os

class CameraInfoSubscriber(Node):
    def __init__(self):
        super().__init__('camera_info_subscriber')

        self.out_file = 'intrinsics.json' #output file
        self.topic = '/oakd/rgb/camera_info'

        self.subscription = self.create_subscription(
            CameraInfo,
            self.topic,
            self.listener_callback,
            10)
        self.subscription

        self.get_logger().info(f"Listening to '{self.topic}' - will save to '{self.out_file}'")

        
    def listener_callback(self, msg):
        #timestamp handling
        timestamp_sec = msg.header.stamp.sec
        timestamp_nanosec = msg.header.stamp.nanosec
        timestamp_combined = timestamp_sec + timestamp_nanosec * 1e-9

        # Daten aus cameramatrix
        fx = msg.k[0]
        fy = msg.k[4]
        cx = msg.k[2]
        cy = msg.k[5]

        data = {
            "timestamp": {
                "sec": timestamp_sec,
                "nanosec": timestamp_nanosec,
                "combined": timestamp_combined
            },
            "width": msg.width,
            "height": msg.height,
            "fx": fx,
            "fy": fy,
            "cx": cx,
            "cy": cy,
            "K": [msg.k[i] for i in range(9)],   # 3x3-Matrix
            "distortion_model": msg.distortion_model,
            "D": list(msg.d)                     
        }

        os.makedirs(os.path.dirname(os.path.abspath(self.out_file)) or ".", exist_ok=True)

        with open(self.out_file, "w") as f:
            json.dump(data, f, indent=2)

        self.get_logger().info(f"Saved intrinsics to {self.out_file}.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    camera_info_subscriber = CameraInfoSubscriber()

    rclpy.spin(camera_info_subscriber)
    camera_info_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
