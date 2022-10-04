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
from scipy.spatial import cKDTree
from dataloader import KITTIloader2012 as lk12
from dataloader import MiddleburyLoader as DA
from dataloader import kittimot_dataset
import time
import numpy as np
import json 
from stereocodes.high_res_stereo_master import getevalmodel

import stereocodes.SGMpython.gpu_library  as gpu_library



mTis = TIS.MultiTis("camerahardware/cameras.json")
mTis.start_cameras()


with open('multicam_config.json','r') as F:
    config_dict = json.load(F)

# SGM models    
libsgm = gpu_library.Algo_libsgm(json.dumps(config_dict))
libopencv = gpu_library.Algo_openCV(json.dumps(config_dict))

# deep learning models
hsm_model = getevalmodel()


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
        hsm_model.eval()
        pred_disp,entropy = hsm_model(images[LEFT],images[RIGHT])

