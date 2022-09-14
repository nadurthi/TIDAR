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
import time
import queue
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan,PointCloud2

folder = 'sampleset1'
os.makedirs(os.path.join(folder,"velodyne"),exist_ok=True)

q = queue.Queue()

#vis = o3d.visualization.Visualizer()
#vis.create_window()
pcd = o3d.geometry.PointCloud()
#vis.add_geometry(pcd)
#    
doneflg=False

def savePCD(X,cnt):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(X[:,:3])   
#        pcd.colors = o3d.utility.Vector3dVector(X[:,3]) 
    o3d.io.write_point_cloud(os.path.join(folder,"velodyne","velodyne_%d.pcd"%cnt), pcd)
    
def plotpcd(q):
    
    
    while 1:
        X = q.get()
        print(X.shape)
        pcd.points = o3d.utility.Vector3dVector(X[:,:3])  
        o3d.visualization.draw_geometries([pcd])
#        vis.update_geometry(pcd)
#        vis.poll_events()
#        vis.update_renderer()
        time.sleep(0.05)
        q.task_done()
        if q.empty() is True and doneflg is True:
            break


     
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
#        q.put(X)
        
        print("pushded X= ",X.shape)

        
        self.cnt+=1

        
def main(args=None):
#    thrd = threading.Thread(target=plotpcd, args=(q,))
#    thrd.start()

    rclpy.init(args=args)

    velo_subscriber = VeloSubscriber()

    
    rclpy.spin(velo_subscriber)
#    while rclpy.ok():
#        rclpy.spin_once(velo_subscriber)
#        vis.add_geometry(velo_subscriber.pcd)
#        vis.update_geometry(velo_subscriber.pcd)
#        vis.poll_events()
#        vis.update_renderer()
        
        
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    doneflg=True
    thrd.join()
    velo_subscriber.vis.destroy_window()
    velo_subscriber.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()