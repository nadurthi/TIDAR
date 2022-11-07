#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov  3 18:32:28 2022

@author: nagnanmus
"""

import open3d as o3d
import numpy as np
import os
from scipy.spatial.transform import Rotation as RR
import copy
import scipy.optimize
import pickle as pkl

def Hmat(eulangles,t):
     R=RR.from_euler('zyx', eulangles, degrees=True)
     H=np.identity(4)
     H[0:3,0:3]=R.as_matrix()
     H[0:3,3]=t
     return H


Hest=np.linalg.multi_dot([Hmat([0,0,0],[0,0.01,0]),Hmat([0,0,-85],[0,0,0]),Hmat([0,0,0],[0,-0.055,0]),Hmat([0,0,0],[0,0,0.04])])

folder='simulations/cam_velo_stepper/2022-11-07-13-18-46'
X=[]
for step in range(0,8000,100):
    i=3
    fname=os.path.join(os.path.join(folder,'velo_%05d_%02d.bin'%(step,i)))
    x=np.fromfile(fname, dtype=np.float32).reshape(4,-1,order='F').T
    R=np.array([[0,-1,0],[1,0,0],[0,0,1]])
    x[:,:3]=R.dot(x[:,:3].T).T
    X.append(x)
    


# filter(X[2][:,:3])

#%%
def filter(X):
    ind = (X[:,0]<0) & (X[:,0]>-1.5)
    # ind2 = np.abs(np.arctan(X[:,1]/X[:,0]))<60*np.pi/180
    ind2= (X[:,1]>0) & (X[:,1]<0.75)
    # return X[ind & ind2,:]
    return X
    
HH=[]
N=80
pairs=[(p1,p1+1) for p1 in range(N-1)]
pairs=pairs+[(p1,p1+2) for p1 in range(N-2)]
pairs=pairs+[(p1,p1+3) for p1 in range(N-3)]


for p1,p2 in pairs:
    print(p1,p2)
     # Pass xyz to Open3D.o3d.geometry.PointCloud and visualize
    source_pcd = o3d.geometry.PointCloud()
    source_pcd.points = o3d.utility.Vector3dVector(filter(X[p1][:,:3]))
    source_pcd=source_pcd.voxel_down_sample(voxel_size=0.025)
    source_pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
    source_pcd.orient_normals_consistent_tangent_plane(15)
    
    target_pcd = o3d.geometry.PointCloud()
    target_pcd.points = o3d.utility.Vector3dVector(filter(X[p2][:,:3]))
    target_pcd=target_pcd.voxel_down_sample(voxel_size=0.025)
    target_pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
    target_pcd.orient_normals_consistent_tangent_plane(15)
    
    
    # o3d.visualization.draw_geometries([source_pcd],point_show_normal=True)
    # o3d.visualization.draw_geometries([target_pcd],point_show_normal=True)
    
    threshold=0.1
    ang= (p2-p1)*2.25
    Hr = Hmat([-ang,0,0],[0,0,0])
    Hrv=Hest
    Hvr=np.linalg.inv(Hrv)
    H=np.linalg.multi_dot([Hvr,Hr,Hrv])
    # H=np.linalg.multi_dot([Hrv,Hr,Hvr])
    Hinv = np.linalg.inv(H)

    
    # threshold = 0.1
    # ang= (p2-p1)*2.25
    # trans_init = np.asarray([[np.cos(ang*np.pi/180), 0, -np.sin(ang*np.pi/180), 0.01],
    #                          [0, 1,0, 0.01],
    #                          [np.sin(ang*np.pi/180), 0, np.cos(ang*np.pi/180), 0.01], 
    #                          [0, 0, 0, 1.0]])
    crti=o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=0.000001,
                                           relative_rmse=0.000001,
                                           max_iteration=500)
    reg_p2l = o3d.pipelines.registration.registration_icp(
        source_pcd, target_pcd, threshold, Hinv,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),crti)
    
    # reg_p2l = o3d.pipelines.registration.registration_generalized_icp(
    #     source, target, params['max_corr_dist_fine'],
    #     icp_coarse.transformation,
    #     o3d.pipelines.registration.
    #     TransformationEstimationForGeneralizedICP(),
    #     o3d.pipelines.registration.ICPConvergenceCriteria(
    #         relative_fitness=1e-6,
    #         relative_rmse=1e-6,
    #         max_iteration=30))
    
    HH.append([p1,p2,np.array(reg_p2l.transformation)])

    # o3d.visualization.draw_geometries([target_pcd,copy.deepcopy(source_pcd).transform(reg_p2l.transformation)])

with open(os.path.join(folder,'poses.pkl'),'wb') as F:
    pkl.dump(HH,F)
#%%
p1=0
p2=5
source_pcd = o3d.geometry.PointCloud()
source_pcd.points = o3d.utility.Vector3dVector(filter(X[p1][:,:3]))
source_pcd=source_pcd.voxel_down_sample(voxel_size=0.0025)
source_pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
source_pcd.orient_normals_consistent_tangent_plane(15)

target_pcd = o3d.geometry.PointCloud()
target_pcd.points = o3d.utility.Vector3dVector(filter(X[p2][:,:3]))
target_pcd=target_pcd.voxel_down_sample(voxel_size=0.0025)
target_pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
target_pcd.orient_normals_consistent_tangent_plane(15)

ang= (p2-p1)*2.25
Hr = Hmat([-ang,0,0],[0,0,0])
Hrv=Hest_opt
Hvr=np.linalg.inv(Hrv)
H=np.linalg.multi_dot([Hvr,Hr,Hrv])
# H=np.linalg.multi_dot([Hrv,Hr,Hvr])
Hinv = np.linalg.inv(H)

# o3d.visualization.draw_geometries([source_pcd,target_pcd])


o3d.visualization.draw_geometries([target_pcd,copy.deepcopy(source_pcd).transform(Hinv)])

#%%
# 
# print(reg_p2l)
# print("Transformation is:")
# print(reg_p2l.transformation)
# print(reg_p2l.transformation[:3,3])
# r = RR.from_matrix(np.array(reg_p2l.transformation[0:3,0:3]))
# r.as_euler('zyx', degrees=True)

# o3d.visualization.draw_geometries([copy.deepcopy(source_pcd).transform(reg_p2l.transformation),target_pcd])



p1,p2,Ht=HH[5]
print(Ht.round(3))
ang= (p2-p1)*2.25
Hr = Hmat([-ang,0,0],[0,0,0])
Hrv=Hest
Hvr=np.linalg.inv(Hrv)
# H=np.linalg.multi_dot([Hrv,Hr,Hvr])
H=np.linalg.multi_dot([Hvr,Hr,Hrv])
Hinv = np.linalg.inv(H)
print(Hinv.round(3))

#%%
def err_lsq(x,HH):
    
    eulang = x[:3]
    t=x[3:]
    Hrv=Hmat(eulang,t)
    Hvr=np.linalg.inv(Hrv)
    err=np.zeros(len(HH))
    i=0
    for p1,p2,Htr in HH:
        ang= (p2-p1)*2.25
        Hr = Hmat([-ang,0,0],[0,0,0])
        # H=np.linalg.multi_dot([Hvr,Hr,Hrv])
        H=np.linalg.multi_dot([Hvr,Hr,Hrv])
        # H=np.linalg.multi_dot([Hrv,Hr,Hvr])
        Hinv = np.linalg.inv(H)
        
        # Httt = np.linalg.inv(Htr)

        eultr=RR.from_matrix(Htr[0:3,0:3]).as_euler('zyx',degrees=False)
        eulest=RR.from_matrix(Hinv[0:3,0:3]).as_euler('zyx',degrees=False)
        
        err[i]=np.sum(np.abs(eultr-eulest))+10*np.sum(np.abs(Hinv[0:3,3]-Htr[0:3,3]))
        i+=1
    return err

def err_min(x,HH):
    
    err=err_lsq(x,HH)
    return np.sum(err)

x0=np.zeros(6)
x0[:3]=RR.from_matrix(Hest[0:3,0:3]).as_euler('zyx',degrees=True)
x0[3:]=Hest[0:3,3]
res1=scipy.optimize.minimize(err_min,x0,args=(HH[:3],),bounds=[(-30,30),(-30,30),(-90,90),(-0.1,1),(-0.1,1),(-1,1)] ,options={'maxiter':5000})
eulang = res1.x[:3]
t=res1.x[3:]
Hest_opt1=Hmat(eulang,t)
print(Hest_opt1.round(3))
print(Hest.round(3))


res2=scipy.optimize.least_squares(err_lsq,x0,args=(HH,),bounds=[(-30,-30,-90,-0.1,-1,-1),(30,30,90,0.1,1,1)])
eulang = res2.x[:3]
t=res2.x[3:]
Hest_opt=Hmat(eulang,t)
print(Hest_opt.round(3))
print(Hest.round(3))

# Optimed
# [[ 1.     0.022 -0.002  0.008]
#  [ 0.     0.086  0.996  0.05 ]
#  [ 0.023 -0.996  0.086  0.058]
#  [ 0.     0.     0.     1.   ]]
# rough est
# [[ 1.     0.     0.     0.   ]
#  [ 0.     0.087  0.996  0.045]
#  [ 0.    -0.996  0.087  0.058]
#  [ 0.     0.     0.     1.   ]]

with open(os.path.join(folder,'velo_step_calib.pkl'),'wb') as F:
    pkl.dump([Hest,Hest_opt],F)
#%%

Hest_opt


# o3d.visualization.draw_geometries([source_pcd,target_pcd])
X=[]
p1=0
pcd=None
for p2,step in enumerate(range(0,15999,100)):
    i=3
    print(p2,step)
    fname=os.path.join(os.path.join(folder,'velo_%05d_%02d.bin'%(step,i)))
    x=np.fromfile(fname, dtype=np.float32).reshape(4,-1,order='F').T
    R=np.array([[0,-1,0],[1,0,0],[0,0,1]])
    x[:,:3]=R.dot(x[:,:3].T).T
    # X.append(x)
    
    ang= (p2-p1)*2.25
    Hr = Hmat([-ang,0,0],[0,0,0])
    Hrv=Hest_opt
    Hvr=np.linalg.inv(Hrv)
    H=np.linalg.multi_dot([Hvr,Hr,Hrv])
    # H=np.linalg.multi_dot([Hrv,Hr,Hvr])
    Hinv = np.linalg.inv(H)
    

    source_pcd = o3d.geometry.PointCloud()
    source_pcd.points = o3d.utility.Vector3dVector(x[:,:3])
    source_pcd=source_pcd.transform(H)
    # source_pcd=source_pcd.voxel_down_sample(voxel_size=0.025)
    # source_pcd.estimate_normals(
        # search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
    # source_pcd.orient_normals_consistent_tangent_plane(15)
    
    if pcd is None:
        pcd=source_pcd
    else:
        pcd=pcd+source_pcd
    pcd=pcd.voxel_down_sample(voxel_size=0.01)
    
o3d.visualization.draw_geometries([pcd])
pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.25, max_nn=30))

# with o3d.utility.VerbosityContextManager(
#         o3d.utility.VerbosityLevel.Debug) as cm:
#     mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
#         pcd, depth=20)

radii = [0.005,0.01, 0.02, 0.04]
rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    pcd, o3d.utility.DoubleVector(radii))
o3d.visualization.draw_geometries([rec_mesh])



# print(mesh)
# o3d.visualization.draw_geometries([mesh])
