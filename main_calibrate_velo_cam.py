#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov  3 18:32:28 2022

@author: nagnanmus
"""

import open3d as o3d
import numpy as np
import os

folder='simulations/cam_velo_stepper/2022-11-03-19-03-10'
X=[]
for step in range(0,1500,100):
    i=3
    fname=os.path.join(os.path.join(folder,'velo_%05d_%02d.bin'%(step,i)))
    X.append(np.fromfile(fname, dtype=np.float32).reshape(-1,4))

def filter(X):
    ind = X[:,1]>0
    ind2 = np.abs(np.arctan(X[:,1]/X[:,0]))<60*np.pi/180
    return X[ind,:]
# filter(X[2][:,:3])

 # Pass xyz to Open3D.o3d.geometry.PointCloud and visualize
source_pcd = o3d.geometry.PointCloud()
source_pcd.points = o3d.utility.Vector3dVector(filter(X[2][:,:3]))
source_pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

target_pcd = o3d.geometry.PointCloud()
target_pcd.points = o3d.utility.Vector3dVector(filter(X[3][:,:3]))
target_pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

o3d.visualization.draw_geometries([source_pcd,target_pcd])

threshold = 0.02
trans_init = np.asarray([[np.cos(2.25*np.pi/180), 0, -np.sin(2.25*np.pi/180), 0.02],
                         [0, 1,0, 0],
                         [np.sin(2.25*np.pi/180), 0, np.cos(2.25*np.pi/180), 0.02], 
                         [0.0, 0.0, 0.0, 1.0]])
reg_p2l = o3d.pipelines.registration.registration_icp(
    source_pcd, target_pcd, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPlane())
print(reg_p2l)
print("Transformation is:")
print(reg_p2l.transformation)

o3d.visualization.draw_geometries([source_pcd.transform(reg_p2l.transformation),target_pcd])

