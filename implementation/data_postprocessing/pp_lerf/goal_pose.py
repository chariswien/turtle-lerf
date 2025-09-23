#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import numpy as np
import struct


class CenterPointPublisher(Node):
    def __init__(self):
        super().__init__('center_point_publisher')
        
        self.center_x = 0.0
        self.center_y = 0.0
        self.got_pointcloud = False
        
        #sub to pointcloud
        self.subscription = self.create_subscription(
            PointCloud2,
            '/pointcloud',
            self.pointcloud_callback,
            10
        )
        
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        timer_period = 2.0 
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def pointcloud_callback(self, msg):
        points = []
        
        for i in range(msg.width):
            offset = i * msg.point_step
            x, y, z = struct.unpack_from('fff', msg.data, offset)
            points.append([x, y, z])
        
        if len(points) > 0:
            points = np.array(points)
            
            #calc center point
            self.center_x = np.mean(points[:, 0])
            self.center_y = np.mean(points[:, 1])
            
            if not self.got_pointcloud:
                self.get_logger().info(f'Center: x={self.center_x:.3f}, y={self.center_y:.3f}')
                self.got_pointcloud = True
    
    def timer_callback(self):
        if not self.got_pointcloud:
            return
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Set position to center point at floor
        msg.pose.position.x = self.center_x
        msg.pose.position.y = self.center_y
        msg.pose.position.z = 0.0 ##projected on floor
        msg.pose.orientation.w = 1.0
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing goal: ({self.center_x:.3f}, {self.center_y:.3f})')


def main(args=None):
    rclpy.init(args=args)
    center_point_publisher = CenterPointPublisher()
    rclpy.spin(center_point_publisher)
    center_point_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()