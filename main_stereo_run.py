#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 24 20:09:26 2022

@author: nvidiaorin
"""

import open3d as o3d

import cv2


from camerahardware import TIS

import torch
import torchvision.transforms as trchtrsnf
import torch.backends.cudnn as cudnn
import time
cudnn.benchmark = False
import torchvision
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor

from scipy.spatial import cKDTree
from dataloader import KITTIloader2012 as lk12
from dataloader import MiddleburyLoader as DA
from dataloader import kittimot_dataset
import time
import numpy as np
import json 
import pickle as pkl
import os
from stereocodes.high_res_stereo_master import getmodel
from main_helpers import *

#from stereocodes.high_res_stereo_master import getevalmodel

import stereocodes.SGMpython.gpu_library  as gpu_library



#%% testing calibrated stereo projection
with open('multicam_config.json','r') as F:
    config_dict = json.load(F)


with open("calibrated_camera_parameters.pkl","rb") as F:
    calib_parameters = pkl.load(F)
    
config_dict['libSGM']["P1"]= 5
config_dict['libSGM']["P2"]= 20
config_dict['libSGM']["max_disparity"]= 128
config_dict['libSGM']["min_disparity"]= 0
config_dict['libSGM']["uniqueness"]= 0.98
config_dict['libSGM']["num_paths"]= 8
config_dict['libSGM']["LR_max_diff"]= 2

hsmmodel = getmodel.getevalmodel(max_disp=384)

# SGM models
libsgm = gpu_library.Algo_libsgm(json.dumps(config_dict))
libopencv = gpu_library.Algo_openCV(json.dumps(config_dict))
    


#%% setup cameras
mTis = TIS.MultiTis("camerahardware/cameras.json")
mTis.start_cameras()


#%% configure target detection

yolomodel = torch.hub.load('ultralytics/yolov5', 'yolov5m', pretrained=True)
yolomodel.cuda()
yolomodel.eval()


#%% run simulation
LEFT=0
MID=1
RIGHT=2
for i in range(len(mTis)):
    cv2.namedWindow('Camera_%d' % i, cv2.WINDOW_NORMAL)
cv2.namedWindow('Depth', cv2.WINDOW_NORMAL)

c1=0
c2=1
(Lmapx, Lmapy), (Rmapx, Rmapy) = calib_parameters['stereo_rect'][(c1, c2)][7:9]

while 1:
    images = mTis.snapImages()
        
    if images is None:
        continue

    st=time.time()
    Ldst = cv2.remap(images[c1][:,:,:3], Lmapx, Lmapy, cv2.INTER_LINEAR)
    Rdst = cv2.remap(images[c2][:,:,:3], Rmapx, Rmapy, cv2.INTER_LINEAR)
    et = time.time()
    print("remap undistort = ", et - st)

    st = time.time()
    Ldstgray = cv2.cvtColor(Ldst, cv2.COLOR_BGR2GRAY)
    Rdstgray = cv2.cvtColor(Rdst, cv2.COLOR_BGR2GRAY)
    et = time.time()
    print("gray conversion = ",et-st)

    # deep
    st = time.time()
    disp_hsm = getmodel.runmodel(hsmmodel, Ldst.astype('float32'), Rdst.astype('float32'))
    et=time.time()




    # disp_csgm_libopencv=libopencv.compute_stereo_csgm(Ldstgray,Rdstgray)
    # disp_csgm_libopencv=disp_csgm_libopencv/16
    # disp_csgm_libopencv[disp_csgm_libopencv<=0]=0

    # st=time.time()
    # disp_libsgm_gpu = libsgm.getDisparity_gpu(Ldstgray,Rdstgray)
    # et=time.time()
    # print("libsgm time = ",et-st)

    Q = calib_parameters['stereo_rect'][(c1, c2)][4]
    depths, depthcolor, pcd = get_colored_depth(Ldst, disp_hsm, Q, dmax=100, returnpcd=False)


    #% target detection---------------------------------
    st=time.time()
    predictions = yolomodel([cv2.resize(Ldst, None,fx=0.5, fy=0.5,interpolation = cv2.INTER_AREA) ])
    preds = []
    for dd in predictions.pandas().xyxy:
        b={'boxes': 2*dd[['xmin', 'ymin', 'xmax', 'ymax']].values.astype(int),
                            'scores': dd['confidence'].values.astype(np.float32),
                            'labels': dd['name'].values}
        preds.append(b)

    torch.cuda.empty_cache()


    for j in range(len(preds)):
        if j>0:
            break
        boxes_f = preds[j]['boxes']
        labels_f = preds[j]['labels']
        scores_f = preds[j]['scores']
        for i in range(len(boxes_f)):
            d = depths[boxes_f[i][1]:boxes_f[i][3],boxes_f[i][0]:boxes_f[i][2]].reshape(-1)
            d=np.round(d[np.isfinite(d)],2)
            cv2.rectangle(Ldst,(boxes_f[i][0],boxes_f[i][1]),(boxes_f[i][2],boxes_f[i][3]),(0,255,0),2)
            cv2.putText(Ldst,str(labels_f[i])+" : "+str(scores_f[i])+" || {0:.2f},{0:.2f},{0:.2f}".format(np.min(d),np.mean(d),np.max(d)),(boxes_f[i][0],boxes_f[i][1]-10),0,1,(0,255,0),2)


    et = time.time()
    print("yolo time = ",et-st)

    #% plotting ------------------------------
    imagesdst=[Ldst,Rdst]
    for i in range(len(imagesdst)):
        cv2.imshow('Camera_%d' % i, imagesdst[i])


    cv2.imshow("Depth", depthcolor)
    params = {}
    params['name'] = "Depth"
    params['img'] = depthcolor
    params['depths'] = depths
    cv2.setMouseCallback("Depth", click_event_depth, params)

    lastkey = cv2.waitKey(10)
    if lastkey == 27 or lastkey == 113:
        break
