#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 24 20:09:26 2022

@author: nvidiaorin
"""
import cv2
import open3d as o3d

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
from stereocodes.high_res_stereo_master import getevalmodel

import stereocodes.SGMpython.gpu_library  as gpu_library


#%% setup cameras
mTis = TIS.MultiTis("camerahardware/cameras.json")
mTis.start_cameras()



#%% configure stereo
with open('multicam_config.json','r') as F:
    config_dict = json.load(F)

# SGM models    
libsgm = gpu_library.Algo_libsgm(json.dumps(config_dict))
libopencv = gpu_library.Algo_openCV(json.dumps(config_dict))

# deep learning models
hsm_model = getevalmodel()
hsm_model.eval()
#%% configure target detection

model = torch.hub.load('ultralytics/yolov5', 'yolov5m', pretrained=True)
model.cuda()
model.eval()


#%% run simulation
LEFT=0
MID=1
RIGHT=2

while 1:
    images = mTis.snapImages()
        
    if images is None:
        continue
    
    gray = [cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) for img in images]

#    disp_libsgm_gpu=libsgm.getDisparity_gpu(gray[LEFT],gray[RIGHT])
    disp_csgm_libopencv=libopencv.compute_stereo_csgm(gray[LEFT],gray[RIGHT])
    disp_csgm_libopencv=disp_csgm_libopencv/16
    disp_csgm_libopencv[disp_csgm_libopencv<=0]=0
    
    with torch.no_grad():
        pred_disp,entropy = hsm_model(images[LEFT],images[RIGHT])

    predictions = model([images[0]])
    preds = []
    for dd in predictions.pandas().xyxy:
        preds.append({'boxes': dd[['xmin', 'ymin', 'xmax', 'ymax']].values.astype(np.float32),
                            'scores': dd['confidence'].values.astype(np.float32),
                            'labels': dd['name'].values})
    torch.cuda.empty_cache()
    
    for j in range(len(preds)):
        if j>0:
            break
        boxes_f = preds[j]['boxes']
        labels_f = preds[j]['labels']
        scores_f = preds[j]['scores']
        for i in range(len(boxes_f)):
            cv2.rectangle(images[j],(boxes_f[i][0],boxes_f[i][1]),(boxes_f[i][2],boxes_f[i][3]),(0,255,0),2)
            cv2.putText(images[j],str(labels_f[i])+" : "+str(scores_f[i]),(boxes_f[i][0],boxes_f[i][1]-10),0,0.3,(0,255,0))

