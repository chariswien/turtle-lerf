#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import csv
import os


class ImageSaver(Node):

    def __init__(self):
        super().__init__('image_saver')
        
        self.topic = '/oakd/rgb/image_raw'
        self.out_dir = 'images'
        self.csv_file_name = 'timestamps.csv'
        
        self.bridge = CvBridge()
        self.image_count = 0
        
        os.makedirs(self.out_dir, exist_ok=True)
        
        # open and write header of csv
        self.csv_file = open(os.path.join(self.out_dir, self.csv_file_name), 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['filename', 'timestamp_sec', 'timestamp_nanosec'])
        
        # sub images topic
        self.subscription = self.create_subscription(
            Image,
            self.topic,
            self.listener_callback,
            10)
        self.subscription
        
        self.get_logger().info(f"Listening to '{self.topic}' and saving to '{self.out_dir}/'")

    def listener_callback(self, msg):

        filename = f"frame_{self.image_count:06d}.png"
        filepath = os.path.join(self.out_dir, filename)
        
        # ROS -> openCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
        
        # save image
        cv2.imwrite(filepath, cv_image)
        
        # write timestand 
        self.csv_writer.writerow([
            filename,
            msg.header.stamp.sec,
            msg.header.stamp.nanosec
        ])
        self.csv_file.flush()  # writes immediately
        
        self.image_count += 1
        
        # log 
        if self.image_count % 10 == 0:
            self.get_logger().info(f"Saved {self.image_count} images")

    def __del__(self):
        if hasattr(self, 'csv_file'):
            self.csv_file.close()
            self.get_logger().info(f'CSV file closed. Total images saved: {self.image_count}')


def main(args=None):
    rclpy.init(args=args)
    
    image_saver = ImageSaver()
    
    rclpy.spin(image_saver)
    
    image_saver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()