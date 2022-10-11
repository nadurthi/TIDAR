#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct  9 00:28:44 2022

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

folder = 'camerahardware/camera_saved_data/2022-10-08-18-12-02'
Ncam=3
ff=os.listdir(folder)
Nset = int(len(ff)/Ncam)
    
for si in range(Nset):
    for cj in range(Ncam): #Ncam
        print(si,cj)
        fname = "%05d_%02d.png"%(0,0)
        Limg = cv2.imread(os.path.join(folder,fname))
        fname = "%05d_%02d.png"%(0,1)
        Mimg = cv2.imread(os.path.join(folder,fname))
        fname = "%05d_%02d.png"%(0,2)
        Rimg = cv2.imread(os.path.join(folder,fname))
        Images=[Limg,Mimg,Rimg]
        c1=0
        c2=1
        
        (Lmapx, Lmapy),(Rmapx, Rmapy)=calib_parameters['stereo_rect'][(c1,c2)][7:9]
        Ldst = cv2.remap(Images[c1], Lmapx, Lmapy, cv2.INTER_LINEAR)
        Rdst = cv2.remap(Images[c2], Rmapx, Rmapy, cv2.INTER_LINEAR)
        

        
        h,  w = Images[c1].shape[:2]

        grayL = cv2.cvtColor(Ldst, cv2.COLOR_BGR2GRAY)
        grayR = cv2.cvtColor(Rdst, cv2.COLOR_BGR2GRAY)

        disp_libsgm_gpu=libsgm.getDisparity_gpu(grayL,grayR)
        
        h,w=grayL.shape
        down_width, down_height = int(w/2),int(h/2)

        grayL_down = cv2.resize(grayL, (down_width, down_height), interpolation= cv2.INTER_LINEAR)
        grayR_down = cv2.resize(grayR, (down_width, down_height), interpolation= cv2.INTER_LINEAR)

        disp_csgm_libopencv=libopencv.compute_stereo_csgm(grayL_down,grayR_down)
        disp_csgm_libopencv = cv2.resize(disp_csgm_libopencv, (w, h), interpolation= cv2.INTER_CUBIC)
        disp_csgm_libopencv=disp_csgm_libopencv/16
        disp_csgm_libopencv[disp_csgm_libopencv<=0]=0


        disp_hsm = getmodel.runmodel(hsmmodel, Ldst.astype('float32'), Rdst.astype('float32'))

        Q = calib_parameters['stereo_rect'][(c1, c2)][4]
        depths,depthcolor,pcd=get_colored_depth(Ldst, disp_hsm, Q, dmax=100, returnpcd=False)

        plotcv([["depth",depthcolor,click_event_depth,{'depths':depths}]])


        break
    break
