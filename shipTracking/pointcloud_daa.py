#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Aug  1 14:58:44 2022

@author: nvidiaorin
"""

import open3d as o3d
import os
import numpy as np
import cv2
from PIL import Image

ffpng=os.path.join('/home/nvidiaorin/Documents/TIDAR/shipTracking','simulations','sampleset1','left','image_1.png')
ffpcd=os.path.join('/home/nvidiaorin/Documents/TIDAR/shipTracking','simulations','sampleset1','pointcloud','image_1.npz.npy')
ffjpg=os.path.join('/home/nvidiaorin/Documents/TIDAR/shipTracking','simulations','ff.jpg')

Limg = cv2.imread(ffpng, cv2.IMREAD_COLOR)
X = np.load(ffpcd)
PP=np.ravel(X[:, :, 3]).view('uint8').reshape((X.shape[0],X.shape[1], 4))

cv2.imshow("Left",Limg)
cv2.waitKey(0)
cv2.destroyAllWindows()
 
#.astype(np.float32)/255.0

#pcd = o3d.io.read_point_cloud(ff )
#X=np.asarray(pcd.points)
pcd = o3d.geometry.PointCloud()
XX=X[:,:,:3].reshape(-1,3)/1000
ind=np.isfinite(XX[:,0])
pcd.points = o3d.utility.Vector3dVector(XX[ind])
cc=PP[:,:,:3].reshape(-1,3)[ind]
pcd.colors = o3d.utility.Vector3dVector(cc/255.0) #
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


import open3d as o3d

mesh = o3d.io.read_triangle_mesh("/home/na0043/Insync/n.adurthi@gmail.com/Google Drive/repos/TIDAR/shipTracking/data_0.ply")
o3d.visualization.draw_geometries([mesh])

pcd = o3d.geometry.PointCloud()
pcd.points = mesh.vertices
pcd.colors = mesh.vertex_colors
o3d.visualization.draw_geometries([pcd])