# -*- coding: utf-8 -*-
import pdb
import argparse
import os
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
from .models import hsm
from .utilshsm import logger 
torch.backends.cudnn.benchmark=True
from utilshsm.preprocess import get_transform


def getevalmodel():
    maxdisp = 384
     
    model = hsm(maxdisp,clean=False,level=1)
    model = nn.DataParallel(model)
    model.cuda()
    
    
    loadmodel = os.path.join(os.path.dirname(os.path.realpath(__file__)),'bestmodels','kitti.tar')
    pretrained_dict = torch.load(loadmodel)
    pretrained_dict['state_dict'] =  {k:v for k,v in pretrained_dict['state_dict'].items() if ('disp' not in k) }
    model.load_state_dict(pretrained_dict['state_dict'],strict=False)
    
    model.eval()
    
    if max_disp>0:
        if max_disp % 16 != 0:
            max_disp = 16 * math.floor(max_disp/16)
        max_disp = int(max_disp)
    else:
        with open(leftfile.replace('im0.png','calib.txt')) as f:
            lines = f.readlines()
            max_disp = int(int(lines[6].split('=')[-1]))

    ## change max disp
    tmpdisp = int(max_disp//64*64)
    if (max_disp*testres/64*64) > tmpdisp:
        model.module.maxdisp = tmpdisp + 64
    else:
        model.module.maxdisp = tmpdisp
    if model.module.maxdisp ==64: model.module.maxdisp=128
    
    model.module.disp_reg8 =  disparityregression(model.module.maxdisp,16).cuda()
    model.module.disp_reg16 = disparityregression(model.module.maxdisp,16).cuda()
    model.module.disp_reg32 = disparityregression(model.module.maxdisp,32).cuda()
    model.module.disp_reg64 = disparityregression(model.module.maxdisp,64).cuda()
    
    
    return model