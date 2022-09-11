#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Aug  1 14:58:44 2022

@author: nvidiaorin
"""

import open3d as o3d
import os

pcd = o3d.io.read_point_cloud(os.path.join("sampleset1","velodyne","velodyne_10.pcd") )
o3d.visualization.draw_geometries([pcd])


color_raw = o3d.io.read_image(os.path.join("sampleset1","left","image_6.png"))
depth_raw = o3d.io.read_image(os.path.join("sampleset1","depth","depth_6.png"))
rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    color_raw, depth_raw,convert_rgb_to_intensity=False)
print(rgbd_image)

fx = 1400.67
fy = 1400.22
cx = 947.67
cy = 569.468
width=1920
height = 1080

pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
    rgbd_image,
    o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy))
# Flip it, otherwise the pointcloud will be upside down
pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
o3d.visualization.draw_geometries([pcd])




#downpcd = pcd.voxel_down_sample(voxel_size=0.01)
#o3d.visualization.draw_geometries([downpcd])


#downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
#        radius=0.1, max_nn=30))
#o3d.visualization.draw_geometries([downpcd])