#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import numpy as np
import open3d as o3d
import cv2
import yaml
import struct
import sys

class PointcloudMapPublisher(Node):
    def __init__(self, ply_path, yaml_path, pgm_path):
        super().__init__('pointcloud_map_publisher')
        
        # pointcloud loaiding
        pcd = o3d.io.read_point_cloud(ply_path)
        self.points = np.asarray(pcd.points, dtype=np.float32)
        
        # map loading
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)
        
        img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        img = cv2.flip(img, 0)  # flip vertically
        
        # map message
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = 'map'
        self.map_msg.info.resolution = float(config['resolution'])
        self.map_msg.info.width = img.shape[1]
        self.map_msg.info.height = img.shape[0]
        self.map_msg.info.origin = Pose()
        self.map_msg.info.origin.position.x = float(config['origin'][0])
        self.map_msg.info.origin.position.y = float(config['origin'][1])
        
        # image --> occupancy
        data = []
        for y in range(img.shape[0]):
            for x in range(img.shape[1]):
                p = img[y, x]
                if p < 128:
                    data.append(100) 
                elif p > 200:
                    data.append(0)   
                else:
                    data.append(-1)
        self.map_msg.data = data
        
        # pointcloud message
        self.cloud_msg = PointCloud2()
        self.cloud_msg.header.frame_id = 'map'
        self.cloud_msg.height = 1
        self.cloud_msg.width = len(self.points)
        self.cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        self.cloud_msg.point_step = 12
        self.cloud_msg.row_step = self.cloud_msg.point_step * len(self.points)
        self.cloud_msg.is_bigendian = False
        self.cloud_msg.is_dense = True
        
        # Pack point data
        data = []
        for point in self.points:
            data.append(struct.pack('fff', point[0], point[1], point[2]))
        self.cloud_msg.data = b''.join(data)
        
        #publishers
        self.cloud_publisher = self.create_publisher(PointCloud2, '/pointcloud', 10)
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)
        
        timer_period = 5.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        
        now = self.get_clock().now().to_msg()
        self.cloud_msg.header.stamp = now
        self.map_msg.header.stamp = now
        
        self.cloud_publisher.publish(self.cloud_msg)
        self.map_publisher.publish(self.map_msg)
        self.get_logger().info('Published pointcloud and map')

def main(args=None):
    if len(sys.argv) < 4:
        print("Usage: python3 publisher.py <pointcloud.ply> <map.yaml> <map.pgm>")
        return
    
    rclpy.init(args=args)
    
    publisher = PointcloudMapPublisher(sys.argv[1], sys.argv[2], sys.argv[3])
    
    rclpy.spin(publisher)
    
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()