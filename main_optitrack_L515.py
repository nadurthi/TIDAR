# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""


import numpy.linalg
import pandas as pd
import numpy as np
import open3d as o3d
import os
import cv2 
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from camerahardware import calib_codes,object_points 


from mpl_toolkits import mplot3d



path=r"/home/nagnanmus/Downloads/L515-optitrackData"
pose_opt = pd.read_csv(os.path.join(path,"Poses.csv"),header=None,
                       names=['sec','nsec','x','y','z','qx','qy','qz','qw'])


K=	np.reshape([
		609.29248046875,
		0.0,
		0.0,
		0.0,
		609.54571533203125,
		0.0,
		317.69757080078125,
		247.73855590820312,
		1.0
	],(3,3),order='F')
    

x = (u-cx)/fx
y= (v-cy)/fy

actual (X,Y,Z)
X= x * depth.at(u,v)
Y= y * depth.at(u,v)
Z = depth.at(u,v)

# H_f_c?
# H_c_0*H_0_1*H_1_c
Hpose_w_f=[]

for ind in range(414):
    r = R.from_quat(pose_opt.loc[ind,['qx','qy','qz','qw']])
    RR=r.as_matrix()
    H=np.identity(4)
    H[0:3,0:3]=RR
    H[0:3,3]=pose_opt.loc[ind,['x','y','z']]
    Hpose_w_f.append(H)
    
    
Hpose_rel=[np.identity(4)]
for i in range(len(Hpose_w_f)-1):
    H_w_0=Hpose_w_f[i]
    H_w_1=Hpose_w_f[i+1]
    H_0_1 = np.linalg.inv(H_w_0).dot(H_w_1)
    Hpose_rel.append(H_0_1)    

cv2.namedWindow('Camera', cv2.WINDOW_NORMAL)
for ind in range(415):
    print(ind)
    imgpath=os.path.join(path,"color_%05d.png"%ind)
    dpath=os.path.join(path,"depth_%05d.png"%ind)
    Hwf=Hpose_w_f[ind]
    Limg=cv2.imread(imgpath)
    Dimg=cv2.imread(dpath,-1)
    gray = cv2.cvtColor(Limg, cv2.COLOR_BGR2GRAY)
    retL,cor=calib_codes.getchesspattern(gray,fast=False)
    cv2.drawChessboardCorners(Limg, (4,5), cor[0], retL)
    
    cv2.imshow('Camera', Limg)                
    lastkey = cv2.waitKey(10)
    
    if retL:
        break


color_raw = o3d.io.read_image(imgpath)
depth_raw = o3d.io.read_image(dpath)
rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    color_raw, depth_raw,depth_scale=4000,convert_rgb_to_intensity=False)
print(rgbd_image)
# o3d.io.read_pinhole_camera_intrinsic()


pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
    rgbd_image,
    o3d.camera.PinholeCameraIntrinsic(640, 480, K[0,0],K[1,1], K[0,2], K[1,2])
)
# Flip it, otherwise the pointcloud will be upside down
# pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
o3d.visualization.draw_geometries([pcd])
S=np.array(pcd.points)
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot(S[:,0],S[:,1],S[:,2],'r.')


x=cor.reshape(20,2)
X=np.zeros((x.shape[0],3))
z=np.zeros(x.shape[0])
for i in range(x.shape[0]):
    a=int(x[i,0])
    b=int(x[i,1])
    Z=[]
    for j in range(-11,12):
        for k in range(-11,12):
            Z.append(np.mean(Dimg[a+j,b+k]/4000))
    Z=np.array(Z)
    Z=Z[Z>0]
    z[i]=np.max(Z)

# z=np.mean(Dimg[x[:,0].astype(int),x[:,1].astype(int)]/4000,axis=1)
X[:,0:2]=z.reshape(-1,1)*(x-K[0:2,2])/K[[0,1],[0,1]]
X[:,2]=z
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot(X[:,0],X[:,1],X[:,2],'ro')



a=0.175
optipoints=np.array([
    [0,0,3*a],
    [0,0,2*a],
    [0,0,1*a],
    [0,0,0*a],
    
    [1*a,0,3*a],
    [1*a,0,2*a],
    [1*a,0,1*a],
    [1*a,0,0*a],
    
    [2*a,0,3*a],
    [2*a,0,2*a],
    [2*a,0,1*a],
    [2*a,0,0*a],
    
    [3*a,0,3*a],
    [3*a,0,2*a],
    [3*a,0,1*a],
    [3*a,0,0*a],
    
    [4*a,0,3*a],
    [4*a,0,2*a],
    [4*a,0,1*a],
    [4*a,0,0*a],
    
    ])

Hfw=np.linalg.inv(Hwf)
optipoints_f = Hfw[0:3,0:3].dot(optipoints.T).T+Hfw[0:3,3]
# Relevant links:
#   - http://stackoverflow.com/a/32244818/263061 (solution with scale)
#   - "Least-Squares Rigid Motion Using SVD" (no scale but easy proofs and explains how weights could be added)


# Rigidly (+scale) aligns two point clouds with know point-to-point correspondences
# with least-squares error.
# Returns (scale factor c, rotation matrix R, translation vector t) such that
#   Q = P*cR + t
# if they align perfectly, or such that
#   SUM over point i ( | P_i*cR + t - Q_i |^2 )
# is minimised if they don't align perfectly.
def umeyama(P, Q):
    assert P.shape == Q.shape
    n, dim = P.shape

    centeredP = P - P.mean(axis=0)
    centeredQ = Q - Q.mean(axis=0)

    C = np.dot(np.transpose(centeredP), centeredQ) / n

    V, S, W = np.linalg.svd(C)
    d = (np.linalg.det(V) * np.linalg.det(W)) < 0.0

    if d:
        S[-1] = -S[-1]
        V[:, -1] = -V[:, -1]

    R = np.dot(V, W)

    varP = np.var(P, axis=0).sum()
    c = 1/varP * np.sum(S) # scale factor

    t = Q.mean(axis=0) - P.mean(axis=0).dot(c*R)

    return c, R, t



fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot(X[:,0],X[:,1],X[:,2],'ro')
# ax.plot(optipoints_f[:,0],optipoints_f[:,1],optipoints_f[:,2],'bo')

c, R, t=umeyama(X,optipoints_f)