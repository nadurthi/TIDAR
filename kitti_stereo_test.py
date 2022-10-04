#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 24 20:09:26 2022

@author: nvidiaorin
"""
import cv2
import open3d as o3d
import configparser

from scipy.spatial import cKDTree
from dataloader import KITTIloader2012 as lk12
from dataloader import MiddleburyLoader as DA
from dataloader import kittimot_dataset
import time
import numpy as np
import json 

import stereocodes.SGMpython.gpu_library  as gpu_library

with open('multicam_config.json','r') as F:
    config_dict = json.load(F)

def get_points(disp,Q):
    points = cv2.reprojectImageTo3D(disp, Q)
    
    colors = cv2.cvtColor(Limg, cv2.COLOR_BGR2RGB)
    mask = disp > disp.min()
    out_points = points[mask]
    out_colors = colors[mask]
    
    ind = np.linalg.norm(out_points,axis=1)<=100
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(out_points[ind])
    pcd.colors = o3d.utility.Vector3dVector(out_colors[ind]/255)

    
    return pcd

def disp_error(disp_true,disp_pred):
    e=[]
    m1 = disp_pred>disp_pred.min()
    m2 = disp_true>disp_true.min()
    m = m1 & m2
    e = disp_true[m]-disp_pred[m]
    return e


img = np.random.randint(0,255,size=(500,250,3))
img2 = gpu_library.flipcvMat(img)

libsgm = gpu_library.Algo_libsgm(json.dumps(config_dict))
data=np.load('test22.npz')
Limg=cv2.imread("stereocodes/SGMpython/1.png")
Rimg=cv2.imread("stereocodes/SGMpython/2.png")
grayL = cv2.cvtColor(Limg, cv2.COLOR_BGR2GRAY);
grayR = cv2.cvtColor(Rimg, cv2.COLOR_BGR2GRAY);
    
disp_libsgm=libsgm.getDisparity_gpu(grayL,grayR)


libopencv = gpu_library.Algo_openCV(json.dumps(config_dict))
disp_bm_libopencv=libopencv.compute_stereo_bm(grayL,grayR)
disp_csgm_libopencv=libopencv.compute_stereo_csgm(grayL,grayR)

print("done")
#%% load config




all_left_img, all_right_img, all_left_disp,calib_train = lk12.dataloader('/run/user/1000/gvfs/sftp:host=192.168.68.73,user=n.adurthi/home/research/SLAMData/Kitti/stereo2012/training/')
loader_kitti12 = DA.myImageFloder(all_left_img,all_right_img,all_left_disp,calib=calib_train, mode='test',rand_scale=None, order=0)
(left_img, right_img, dataL,(left,right,calib))=loader_kitti12[0]

seq='0000'
kittimot = kittimot_dataset.KittiMOT('/run/user/1000/gvfs/sftp:host=192.168.68.73,user=n.adurthi/home/research/SLAMData/Kitti/tracking/mot/training',seq)

kittimot[0]


#%% OPencv Python based stereo


(left_img, right_img, dataL,(left,right,calib))=loader_kitti12[0]
Limg = cv2.imread(left)
Rimg = cv2.imread(right)
minDisparity=0
stereo_sgbm_pythoncpu = cv2.StereoSGBM_create(minDisparity=0,
                                            numDisparities=128,
                                            blockSize=3,
                                            disp12MaxDiff=10,
                                            P1=20,
                                            P2=84,
                                            mode=3,
                                            uniquenessRatio=9
                                            )

libsgm = gpu_library.Algo_libsgm(json.dumps(config_dict))
grayL = cv2.cvtColor(Limg, cv2.COLOR_BGR2GRAY)
grayR = cv2.cvtColor(Rimg, cv2.COLOR_BGR2GRAY)
st=time.time()
disp_libsgm_cpu=libsgm.getDisparity_cpu(grayL,grayR)
et=time.time()
print("libsgm cpu",et-st)
err_ligsgm_cpu=disp_error(disp_libsgm_cpu,dataL)

st=time.time()
disp_libsgm_gpu=libsgm.getDisparity_gpu(grayL,grayR)
et=time.time()
print("libsgm gpu",et-st)
err_ligsgm_gpu=disp_error(disp_libsgm_gpu,dataL)

libopencv = gpu_library.Algo_openCV(json.dumps(config_dict))

st=time.time()
disp_bm_libopencv=libopencv.compute_stereo_bm(grayL,grayR)
et=time.time()
print("opencv cuda bm",et-st)

st=time.time()
disp_csgm_libopencv=libopencv.compute_stereo_csgm(grayL,grayR)
et=time.time()
print("opencv cuda csgm",et-st)
disp_csgm_libopencv=disp_csgm_libopencv/16
disp_csgm_libopencv[disp_csgm_libopencv<=0]=0


err_bm_cppopencv=disp_error(disp_bm_libopencv/16,dataL)
err_csgm_cppopencv=disp_error(disp_csgm_libopencv,dataL)


st=time.time()
disp = stereo_sgbm_pythoncpu.compute(Limg,Rimg).astype(np.float32)/16
#disp[disp<0]=0
et = time.time()
print("python cv2 sgbm = ",et-st)
err=disp_error(disp,dataL)





pred_pcd = get_points(disp_csgm_libopencv.astype(np.int16),calib['Q'])
true_pcd = get_points(dataL,calib['Q'])
evaluation = o3d.pipelines.registration.evaluate_registration(
    pred_pcd, true_pcd, 0.5, np.identity(4))

kdt = cKDTree(np.asarray(true_pcd.points),leafsize=5)
d,i=kdt.query(np.asarray(pred_pcd.points),k=1,distance_upper_bound=0.5)


o3d.visualization.draw_geometries([pred_pcd])


cv2.namedWindow("gg", cv2.WINDOW_NORMAL) 
cv2.imshow("gg",Limg)
cv2.waitKey(330)

cv2.destroyAllWindows()
#%%

