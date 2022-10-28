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
import numpy as np
import sys

folder = 'sampleset1'
os.makedirs(os.path.join(folder,"velodyne"),exist_ok=True)





def savePCD(X,cnt):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(X[:,:3])   
#        pcd.colors = o3d.utility.Vector3dVector(X[:,3]) 
    o3d.io.write_point_cloud(os.path.join(folder,"velodyne","velodyne_%d.pcd"%cnt), pcd)
    
def plotpcd(q,doneflg):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    pcd = o3d.geometry.PointCloud()
    vis.add_geometry(pcd)
    voxel_size=0.01
    max_correspondence_distance_fine = 1
    
    cnt=0
    while 1:
        
        try:
            X = q.get(block=True, timeout=0.1)
        except queue.Empty:
            X =None
        
        if X is not None:
            pcd.points = o3d.utility.Vector3dVector(X[:,:3])  
            
    #        pcd0 = o3d.geometry.PointCloud()
    #        pcd0.points = o3d.utility.Vector3dVector(X[:,:3])  
    #        if cnt==0:
    #            pcd=pcd0.voxel_down_sample(voxel_size=voxel_size)
    #        else:
    #            icp_fine = o3d.pipelines.registration.registration_generalized_icp(
    #            pcd0, pcd, max_correspondence_distance_fine,
    #            np.identity(4),
    #            o3d.pipelines.registration.
    #            TransformationEstimationForGeneralizedICP(),
    #            o3d.pipelines.registration.ICPConvergenceCriteria(
    #                relative_fitness=1e-6,
    #                relative_rmse=1e-6,
    #                max_iteration=30))
    #            
    #            pcd0.transform(icp_fine.transformation)
    #            pcd=pcd+pcd0
    #            pcd=pcd.voxel_down_sample(voxel_size=voxel_size)
            
            cnt+=1
            print(cnt)
    #        o3d.visualization.draw_geometries([pcd])
            vis.add_geometry(pcd)
            vis.update_geometry(pcd)
            ctr = vis.get_view_control()
            ctr.set_lookat([-2, 3,3])
            ctr.set_up([0, 0, 3])
            ctr.set_front([0, 3, 0])
                                          
            vis.poll_events()
            vis.update_renderer()
            time.sleep(0.01)
            q.task_done()
    
            o3d.io.write_point_cloud(os.path.join(folder,"velodyne","velodyne_%d.pcd"%cnt), pcd)
        if q.empty() and doneflg.is_set():
            print("done flag set...quitting thread")
#            o3d.io.write_point_cloud("velo-combinedfile.pcd",pcd)
            break

    vis.destroy_window()
    print("left thread")
     
class VeloSubscriber(Node):

    def __init__(self,q):
        super().__init__('velosave_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            'velodyne_points',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.q=q
        self.cnt=0


        

        
    def listener_callback(self, msg):
        
        X = rnp.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=True)
#        thrd = threading.Thread(target=savePCD, args=(X,self.cnt))
#        thrd.daemon=True
#        thrd.start()
#        self.q.put(X)
        np.save(os.path.join(folder,"velodyne","velodyne_%d"%self.cnt), X)
        print("pushded cnt= ",self.cnt)

        
        self.cnt+=1

        
def main(args=None):

    q = queue.Queue()
    doneflg = threading.Event()
    doneflg.clear()
#    thrd = threading.Thread(target=plotpcd, args=(q,doneflg))
#    thrd.start()

    rclpy.init(args=args)

    velo_subscriber = VeloSubscriber(q)

    try:
        rclpy.spin(velo_subscriber)
    except KeyboardInterrupt:
        print('server stopped cleanly')
    except BaseException:
        print('exception in server:', file=sys.stderr)
        raise
    finally:
        # Destroy the node explicitly
        # (optional - Done automatically when node is garbage collected)
        doneflg.set()
#        thrd.join()
        velo_subscriber.destroy_node()
        rclpy.shutdown() 

#    while rclpy.ok():
#        rclpy.spin_once(velo_subscriber)
#        vis.add_geometry(velo_subscriber.pcd)
#        vis.update_geometry(velo_subscriber.pcd)
#        vis.poll_events()
#        vis.update_renderer()
        
        
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    
#    velo_subscriber.vis.destroy_window()
    print("shutting down viewer")
    

    
    

if __name__ == '__main__':
    main()