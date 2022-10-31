# -*- coding: utf-8 -*-
import pdb
import argparse
import os
import cv2
import random
import torch
import torch.nn as nn
import torch.backends.cudnn as cudnn
import torch.optim as optim
import torch.utils.data
from torch.autograd import Variable
import torch.nn.functional as F
import numpy as np
import time
from .models.submodule import disparityregression
from .models import hsm
#from .utilshsm import logger 
torch.backends.cudnn.benchmark=True
from .utilshsm.preprocess import get_transform

max_disp=383
16 * np.round(max_disp/16).astype(int)
processed = get_transform()
def getevalmodel(max_disp=384):
    if max_disp % 16 != 0:
        max_disp = 16 * np.round(max_disp/16).astype(int)
    max_disp = int(max_disp)
#    
#    if max_disp % 64 != 0:
#        max_disp = 64 * np.round(max_disp/64).astype(int)
#    max_disp = int(max_disp)
#    
    tmpdisp = int(max_disp//64*64)
    if (max_disp/64*64) > tmpdisp:
        max_disp = tmpdisp + 64
    else:
        max_disp = tmpdisp
    if max_disp ==64: 
        max_disp=128
    
     
    model = hsm.HSMNet(max_disp,clean=1,level=1)
    model = nn.DataParallel(model)
    model.cuda()
    
    
    loadmodel = os.path.join(os.path.dirname(os.path.realpath(__file__)),'bestmodels','kitti.tar')
    pretrained_dict = torch.load(loadmodel)
    pretrained_dict['state_dict'] =  {k:v for k,v in pretrained_dict['state_dict'].items() if ('disp' not in k) }
    model.load_state_dict(pretrained_dict['state_dict'],strict=False)
    
    model.eval()    

    ## change max disp
    
    
    model.module.disp_reg8 =  disparityregression(model.module.maxdisp,16).cuda()
    model.module.disp_reg16 = disparityregression(model.module.maxdisp,16).cuda()
    model.module.disp_reg32 = disparityregression(model.module.maxdisp,32).cuda()
    model.module.disp_reg64 = disparityregression(model.module.maxdisp,64).cuda()
    
    
    return model


def runmodel(model, Limg, Rimg):
    testres=1
    # imgL_o = (skimage.io.imread(leftfile).astype('float32'))[:, :, :3]
    # imgR_o = (skimage.io.imread(rightfile).astype('float32'))[:, :, :3]
    #
    # imgsize = imgL_o.shape[:2]

    # Limg = cv2.imread(leftfile).astype('float32')
    # Rimg = cv2.imread(rightfile).astype('float32')
    # Limg = cv2.cvtColor(Limg, cv2.COLOR_BGR2RGB)
    # Rimg = cv2.cvtColor(Rimg, cv2.COLOR_BGR2RGB)
    imgL_o=Limg
    imgR_o = Rimg
    imgsize = imgL_o.shape[:2]

    # resize
    # imgL_2 = cv2.resize(imgL_o, None, fx=testres, fy=testres, interpolation=cv2.INTER_CUBIC)
    # imgR_2 = cv2.resize(imgR_o, None, fx=testres, fy=testres, interpolation=cv2.INTER_CUBIC)
    imgL = processed(imgL_o).numpy()
    imgR = processed(imgR_o).numpy()

    imgL = np.reshape(imgL, [1, 3, imgL.shape[1], imgL.shape[2]])
    imgR = np.reshape(imgR, [1, 3, imgR.shape[1], imgR.shape[2]])

    ##fast pad
    max_h = int(imgL.shape[2] // 64 * 64)
    max_w = int(imgL.shape[3] // 64 * 64)
    if max_h < imgL.shape[2]: max_h += 64
    if max_w < imgL.shape[3]: max_w += 64

    top_pad = max_h - imgL.shape[2]
    left_pad = max_w - imgL.shape[3]
    imgL = np.lib.pad(imgL, ((0, 0), (0, 0), (top_pad, 0), (0, left_pad)), mode='constant', constant_values=0)
    imgR = np.lib.pad(imgR, ((0, 0), (0, 0), (top_pad, 0), (0, left_pad)), mode='constant', constant_values=0)

    # # test
    imgL = Variable(torch.FloatTensor(imgL).cuda())
    imgR = Variable(torch.FloatTensor(imgR).cuda())

    # show(drawn_boxes)

    with torch.no_grad():
        torch.cuda.synchronize()
        start_time = time.time()
        pred_disp, entropy = model(imgL, imgR)
        torch.cuda.synchronize()
        ttime = (time.time() - start_time);
        print('time = %.2f' % (ttime * 1000))
    pred_disp = torch.squeeze(pred_disp).data.cpu().numpy()

    top_pad = max_h - imgL_o.shape[0]
    left_pad = max_w - imgL_o.shape[1]
    entropy = entropy[top_pad:, :pred_disp.shape[1] - left_pad].cpu().numpy()
    pred_disp = pred_disp[top_pad:, :pred_disp.shape[1] - left_pad]


    pred_disp = cv2.resize(pred_disp / testres, (imgsize[1], imgsize[0]), interpolation=cv2.INTER_LINEAR)

    # clip while keep inf
    invalid = np.logical_or(pred_disp == np.inf, pred_disp != pred_disp)
    pred_disp[invalid] = 0

    return pred_disp
