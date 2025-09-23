#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import csv

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')

        self.out_file_name = 'baselink_pose.csv' #output file
        self.topic = '/pose'
        
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self.topic,
            self.listener_callback,
            10)
        self.subscription
               
        # open and write header of csv
        self.out_file = open(self.out_file_name, 'w', newline='')
        self.csv_writer = csv.writer(self.out_file)

        self.csv_writer.writerow([
            'timestamp', 'frame_id', 
            'pos_x', 'pos_y', 'pos_z',
            'or_x', 'or_y', 'or_z', 'or_w'
        ])
        
        self.get_logger().info(f'Lstening to {self.topic}, saving to {self.out_file_name}')
        self.message_count = 0

    def listener_callback(self, msg):
        #timestamp handling
        timestamp = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
        
        frame_id = msg.header.frame_id
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        pos_z = msg.pose.pose.position.z
        or_x = msg.pose.pose.orientation.x
        or_y = msg.pose.pose.orientation.y
        or_z = msg.pose.pose.orientation.z
        or_w = msg.pose.pose.orientation.w
        
        self.csv_writer.writerow([
            timestamp, frame_id,
            pos_x, pos_y, pos_z,
            or_x, or_y, or_z, or_w
        ])
        #gets written immediately
        self.out_file.flush()
        
        #debug-->löschen
        self.message_count += 1
        self.get_logger().info(
            f'[{self.message_count}] position at timestamp {timestamp}: '
            f'pos=({pos_x:.3f}, {pos_y:.3f}, {pos_z:.3f})'
        )

    def __del__(self):
        # csv closing when finished
        if hasattr(self, 'out_file'):
            self.out_file.close()
            self.get_logger().info(f'CSV file {self.out_file_name} exited')

def main(args=None):
    rclpy.init(args=args)
    
    pose_subscriber = PoseSubscriber()
    
    rclpy.spin(pose_subscriber)
    pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()