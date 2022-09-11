#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 10 09:58:26 2022

@author: nvidiaorin
"""

import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
import open3d as o3d
import os
import threading

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan,PointCloud2

folder = 'sampleset1'
os.makedirs(os.path.join(folder,"velodyne"),exist_ok=True)

def savePCD(X,cnt):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(X[:,:3])   
#        pcd.colors = o3d.utility.Vector3dVector(X[:,3]) 
    o3d.io.write_point_cloud(os.path.join(folder,"velodyne","velodyne_%d.pcd"%cnt), pcd)
    

class VeloSubscriber(Node):

    def __init__(self):
        super().__init__('velosave_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            'velodyne_points',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.cnt=0
        
    def listener_callback(self, msg):
        
        X = rnp.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=True)
        thrd = threading.Thread(target=savePCD, args=(X,self.cnt))
        thrd.daemon=True
        thrd.start()
        
        self.cnt+=1
        print(X.shape)
        
        
def main(args=None):
    rclpy.init(args=args)

    velo_subscriber = VeloSubscriber()

    rclpy.spin(velo_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    velo_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()