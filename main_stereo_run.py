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
import threading
#from stereocodes.high_res_stereo_master import getevalmodel

import stereocodes.SGMpython.gpu_library  as gpu_library


c1=0
c2=2
scale=3


#%% testing calibrated stereo projection
with open('multicam_config.json','r') as F:
    config_dict = json.load(F)

with open("calibrated_camera_parameters.pkl","rb") as F:
    calib_parameters = pkl.load(F)

config_dict['cameras']['height']=calib_parameters[scale]['size'][0]
config_dict['cameras']['width']=calib_parameters[scale]['size'][1]


    
config_dict['libSGM']["P1"]= 5
config_dict['libSGM']["P2"]= 20
config_dict['libSGM']["max_disparity"]= 256
config_dict['libSGM']["min_disparity"]= 0
config_dict['libSGM']["uniqueness"]= 0.98
config_dict['libSGM']["num_paths"]= 8
config_dict['libSGM']["LR_max_diff"]= 2

config_dict['opencv_stereo']["P1"]= 5
config_dict['opencv_stereo']["P2"]= 20
config_dict['opencv_stereo']["max_disparity"]= 256
config_dict['opencv_stereo']["min_disparity"]= 16
config_dict['opencv_stereo']["uniquenessRatio"]= 5



hsmmodel = getmodel.getevalmodel(max_disp=384)

# SGM models
libsgm = gpu_library.Algo_libsgm(json.dumps(config_dict))
libopencv = gpu_library.Algo_openCV(json.dumps(config_dict))
    


#%% setup cameras
mTis = TIS.MultiTis("camerahardware/cameras.json")
mTis.start_cameras()


#%% configure target detection

# yolomodel = torch.hub.load('ultralytics/yolov5', 'yolov5m', pretrained=True)
# yolomodel.cuda()
# yolomodel.eval()


#%% run simulation
LEFT=0
MID=1
RIGHT=2
for i in range(len(mTis)):
    cv2.namedWindow('Camera_%d' % i, cv2.WINDOW_NORMAL)
cv2.namedWindow('Depth', cv2.WINDOW_NORMAL)

class TIDAR:
    def __init__(self):
        self.libsgm = gpu_library.Algo_libsgm(json.dumps(config_dict))
        self.libopencv = gpu_library.Algo_openCV(json.dumps(config_dict))

    def on_trackbar_P1(self,val):
        config_dict['opencv_stereo']["P1"]=val
        config_dict['libSGM']["P1"] = val

        print(val)
        self.libsgm = gpu_library.Algo_libsgm(json.dumps(config_dict))
        self.libopencv = gpu_library.Algo_openCV(json.dumps(config_dict))
    def on_trackbar_P2(self,val):
        config_dict['opencv_stereo']["P2"]=val
        config_dict['libSGM']["P2"] = val
        print(val)
        self.libsgm = gpu_library.Algo_libsgm(json.dumps(config_dict))
        self.libopencv = gpu_library.Algo_openCV(json.dumps(config_dict))

sps=TIDAR()

cv2.createTrackbar("P1", 'Depth' , 2, 300, sps.on_trackbar_P1)
cv2.createTrackbar("P2", 'Depth' , 2, 500, sps.on_trackbar_P2)

(Lmapx, Lmapy), (Rmapx, Rmapy) = calib_parameters[scale]['stereo_rect'][(c1, c2)][7:9]

Lmapx, Lmapy = cv2.convertMaps(Lmapx, Lmapy, dstmap1type=cv2.CV_16SC2,nninterpolation=False)
Rmapx, Rmapy = cv2.convertMaps(Rmapx, Rmapy, dstmap1type=cv2.CV_16SC2,nninterpolation=False)
while 1:
    images = mTis.snapImages()

    if scale>1:
        for i in range(len(mTis)):
            images[i] = cv2.resize(images[i], None, fx=1 / scale, fy=1 / scale, interpolation=cv2.INTER_AREA)

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
    disp = getmodel.runmodel(hsmmodel, Ldst.astype('float32'), Rdst.astype('float32'))
    et=time.time()




    # disp=sps.libopencv.compute_stereo_csgm(Ldstgray,Rdstgray)
    # disp=disp/16
    # disp[disp<=0]=0

    # st=time.time()
    # disp = sps.libsgm.getDisparity_gpu(Ldstgray,Rdstgray)
    # et=time.time()
    # print("libsgm time = ",et-st)

    Q = calib_parameters[scale]['stereo_rect'][(c1, c2)][4]
    depths, depthcolor, pcd = get_colored_depth(Ldst, disp, Q, dmax=100, returnpcd=False)


    #% target detection---------------------------------
    # st=time.time()
    # predictions = yolomodel([cv2.resize(Ldst, None,fx=0.5, fy=0.5,interpolation = cv2.INTER_AREA) ])
    # preds = []
    # for dd in predictions.pandas().xyxy:
    #     b={'boxes': 2*dd[['xmin', 'ymin', 'xmax', 'ymax']].values.astype(int),
    #                         'scores': dd['confidence'].values.astype(np.float32),
    #                         'labels': dd['name'].values}
    #     preds.append(b)
    # 
    # torch.cuda.empty_cache()
    # 
    # 
    # for j in range(len(preds)):
    #     if j>0:
    #         break
    #     boxes_f = preds[j]['boxes']
    #     labels_f = preds[j]['labels']
    #     scores_f = preds[j]['scores']
    #     for i in range(len(boxes_f)):
    #         d = depths[boxes_f[i][1]:boxes_f[i][3],boxes_f[i][0]:boxes_f[i][2]].reshape(-1)
    #         if len(d)==0:
    #             d=[np.inf]
    #         else:
    #             d=np.round(d[np.isfinite(d)],2)
    #         cv2.rectangle(Ldst,(boxes_f[i][0],boxes_f[i][1]),(boxes_f[i][2],boxes_f[i][3]),(0,255,0),2)
    #         cv2.putText(Ldst,str(labels_f[i])+" : "+str(scores_f[i])+" || {0:.2f},{0:.2f},{0:.2f}".format(np.min(d),np.mean(d),np.max(d)),(boxes_f[i][0],boxes_f[i][1]-10),0,1,(0,255,0),2)
    # et = time.time()
    # print("yolo time = ",et-st)

    #% plotting ------------------------------
    imagesdst=[Ldst,Rdst]
    for i in range(len(imagesdst)):
        line_thickness = 2
        h,w=imagesdst[i].shape[:2]
        for x in np.linspace(0 + 10, h - 10, 30):
            cv2.line(imagesdst[i], (0, int(x)), (w, int(x)), (0, 255, 0), thickness=line_thickness)
            cv2.line(imagesdst[i], (0, int(x)), (w, int(x)), (255, 0, 0), thickness=line_thickness)

    cv2.imshow('Camera_%d' % 0, np.hstack(imagesdst[:2]))


    cv2.imshow("Depth", depthcolor)
    params = {}
    params['name'] = "Depth"
    params['img'] = depthcolor
    params['depths'] = depths
    cv2.setMouseCallback("Depth", click_event_depth, params)

    lastkey = cv2.waitKey(10)
    if lastkey == 27 or lastkey == 113:
        break
