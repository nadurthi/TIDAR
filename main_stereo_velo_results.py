#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 24 20:09:26 2022

@author: nvidiaorin
"""
import copy

import open3d as o3d

import cv2

from camerahardware import TIS
from camerahardware import object_points
import torch
import torchvision.transforms as trchtrsnf
import torch.backends.cudnn as cudnn
import time
from velohardware import velodynecalib
cudnn.benchmark = False
import torchvision
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from camerahardware import calib_codes
from scipy.spatial import cKDTree
from dataloader import KITTIloader2012 as lk12
from dataloader import MiddleburyLoader as DA
from dataloader import kittimot_dataset
import time
import numpy as np
import json
import pickle as pkl
import os
from stereocodes.high_res_stereo import getmodel
from main_helpers import *
import threading
import importlib
importlib.reload(calib_codes)
# from stereocodes.high_res_stereo_master import getevalmodel


c1 = 0
c2 = 2
scale = 1

with open("calibfiles/velo_cam_calib.pkl",'rb') as F:
    Rcv,tcv = pkl.load(F)
    Hcv = np.identity(4)
    Hcv[0:3,0:3]=Rcv
    Hcv[0:3, 3] = tcv

# %% testing calibrated stereo projection
with open('calibfiles/multicam_config.json', 'r') as F:
    config_dict = json.load(F)

with open("calibrated_camera_parameters.pkl", "rb") as F:
    calib_parameters = pkl.load(F)

folder='/media/na0043/misc/DATA/cam_velo_stepper/1m/2022-11-15_outside/2022-11-15-14-34-13_calib'

#
# pcd_crop = o3d.io.read_point_cloud(os.path.join(folder,'cropped_1.ply'))
# o3d.visualization.draw_geometries([pcd_crop])




Limg = cv2.imread(os.path.join(folder,"cam_velo_cam_calib_00.png"))
Rimg = cv2.imread(os.path.join(folder,"cam_velo_cam_calib_01.png"))

if scale>1:
    Limg_scaled = cv2.resize(Limg, None, fx=1 / scale, fy=1 / scale, interpolation=cv2.INTER_AREA)
    Rimg_scaled = cv2.resize(Rimg, None, fx=1 / scale, fy=1 / scale, interpolation=cv2.INTER_AREA)
else:
    Limg_scaled=Limg.copy()
    Rimg_scaled = Rimg.copy()

importlib.reload(calib_codes)

def get_or_calibrate_2cam(Limages,Rimages,calib_parameters,returnold=True):
    if returnold:
        return calib_codes.calib_dict_2_tuple(calib_parameters)
    else:
        objp = object_points.objp_checker
        imgpoints = [{}, {}]  # 2d points in image plane.
        for si in range(len(Limages)):
            for cj in range(2):
                if cj==0:
                    img =Limages[si]
                else:
                    img = Rimages[si]
                # ret, cor = cv2.findCirclesGrid(Limg_o, (4, 11), cv2.CALIB_CB_ASYMMETRIC_GRID, blobDetector, None)
                grayimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                ret, cor = calib_codes.getchesspattern(grayimg)
                if ret:
                    imgpoints[cj][si] = cor
        ho, wo = Limages[0].shape[:2]
        calib_parameters_new = calib_codes.calibrate_Cameras_2cams(Limages[0], ho, wo, imgpoints, objp,
                                                                   scales=None,
                                                                       calib_parameters_old=copy.deepcopy(calib_parameters))
        return calib_codes.calib_dict_2_tuple(calib_parameters_new)

CCold=get_or_calibrate_2cam([Limg],[Rimg],calib_parameters,returnold=True)
CC=get_or_calibrate_2cam([Limg],[Rimg],calib_parameters,returnold=False)





(Lmapx, Lmapy), (Rmapx, Rmapy) = CCold[scale]['stereo_rect'][(c1, c2)].map1,CCold[scale]['stereo_rect'][(c1, c2)].map2

Lmapx, Lmapy = cv2.convertMaps(Lmapx, Lmapy, dstmap1type=cv2.CV_16SC2,nninterpolation=False)
Rmapx, Rmapy = cv2.convertMaps(Rmapx, Rmapy, dstmap1type=cv2.CV_16SC2,nninterpolation=False)

st = time.time()
Ldst = cv2.remap(Limg_scaled[:, :, :3], Lmapx, Lmapy, cv2.INTER_LINEAR)
Rdst = cv2.remap(Rimg_scaled[:,:,:3], Rmapx, Rmapy, cv2.INTER_LINEAR)
et = time.time()
print("remap undistort = ", et - st)
st = time.time()
Ldstgray = cv2.cvtColor(Ldst, cv2.COLOR_BGR2GRAY)
Rdstgray = cv2.cvtColor(Rdst, cv2.COLOR_BGR2GRAY)
et = time.time()
print("gray conversion = ", et - st)



hsmmodel = getmodel.getevalmodel(max_disp=384)



# %% configure target detection

yolomodel = torch.hub.load('ultralytics/yolov5', 'yolov5m', pretrained=True)
yolomodel.cuda()
yolomodel.eval()


imagesdst=[Ldst.copy(),Rdst.copy()]
for i in range(len(imagesdst)):
    line_thickness = 2
    h,w=imagesdst[i].shape[:2]
    for x in np.linspace(0 + 10, h - 10, 30):
        cv2.line(imagesdst[i], (0, int(x)), (w, int(x)), (0, 255, 0), thickness=line_thickness)
        cv2.line(imagesdst[i], (0, int(x)), (w, int(x)), (255, 0, 0), thickness=line_thickness)

plotcv([['distort',np.hstack(imagesdst[:2]),None,None]])



st = time.time()
disp = getmodel.runmodel(hsmmodel, Ldst.astype('float32'), Rdst.astype('float32'))
et=time.time()
Q = CCold[scale]['stereo_rect'][(0, 2)].Q
depths, depthcolor, pcd = get_colored_depth(Ldst, disp, Q, dmax=100, returnpcd=False)


params = {}
params['name'] = "Depth"
params['img'] = depthcolor
params['depths'] = depths
# cv2.setMouseCallback("Depth", click_event_depth, params)

plotcv([['Depth',depthcolor,click_event_depth,params]])


# % target detection---------------------------------

# predictions = yolomodel([cv2.resize(Ldst, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)])
predictions = yolomodel(Ldst)
preds = []
for dd in predictions.pandas().xyxy:
    b = {'boxes': dd[['xmin', 'ymin', 'xmax', 'ymax']].values.astype(int),
         'scores': dd['confidence'].values.astype(np.float32),
         'labels': dd['name'].values}
    preds.append(b)

torch.cuda.empty_cache()




#------------------
pcdfile = os.path.join(folder,'cummulative_pcd.pcd')
pcd = o3d.io.read_point_cloud(pcdfile)
pcd_cam=copy.deepcopy(pcd).transform(Hcv)
velodynecalib.plotpcds([pcd])


Xv,Colv = velodynecalib.pcd2points(pcd)
Xcv,Colcv = velodynecalib.pcd2points(pcd_cam)
indz=Xcv[:,2]>0
Xcv_front=Xcv[indz]
Colcv_front=Colcv[indz]

velodynecalib.plotpcds([velodynecalib.points2pcd(Xcv_front,index=None,colmat=Colcv_front,colvec=None)])



Lmtx=CCold[scale]['mono'][0].mtx
Ldts=CCold[scale]['mono'][0].dst

Xcv_imgpts,_=cv2.projectPoints(	Xcv_front, np.zeros(3), np.zeros(3), Lmtx, Ldts)
Xcv_imgpts=Xcv_imgpts[:,0,:]

h,w=Ldst.shape[:2]
ind_f = (Xcv_imgpts[:,0]>=0) & (Xcv_imgpts[:,0]<=w-1) & (Xcv_imgpts[:,1]>=0) & (Xcv_imgpts[:,1]<=h-1)
Xcv_imgpts_inside = Xcv_imgpts[ind_f]
Xcv_imgpts_inside_coords = Xcv_imgpts_inside.round().astype(int)
Xcv_front_inside = Xcv_front[ind_f]
Colcv_front_inside = Colcv_front[ind_f]

img_vf = np.ones((h,w),dtype=int)*np.nan
mx=Colcv_front[:,0].max()
mn=Colcv_front[:,0].min()
for i in range(len(Xcv_imgpts_inside_coords)):
    a = (Colcv_front_inside[i, 0] - mn) / mx
    img_vf[Xcv_imgpts_inside_coords[i, 1], Xcv_imgpts_inside_coords[i, 0]] = np.round(255 * a).astype(int)

img_vf_cp = img_vf.copy()
img_vf_cp[np.isnan(img_vf_cp)]=0
img_vf_scaled = cv2.resize(img_vf_cp, None, fx=1 / 3, fy=1 / 3, interpolation=cv2.INTER_AREA)

mask=np.zeros_like(img_vf_scaled,dtype=np.uint8)
mask[img_vf_scaled==0]=1
img_vf_scaled[np.isnan(img_vf_scaled)]=0
img_vf_scaled=img_vf_scaled.astype(np.uint8)
dst = cv2.inpaint(img_vf_scaled,mask,3,cv2.INPAINT_TELEA)
velo_cam_intensity_img = cv2.resize(dst, (w,h), fx=None, fy=None, interpolation=cv2.INTER_AREA)

# velo_cam_intensity_img = cv2.medianBlur(velo_cam_intensity_img,1)
# dstblur = cv2.adaptiveThreshold(dstblur,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,21,5)
cv2.namedWindow('gg', cv2.WINDOW_NORMAL)
while 1:
    cv2.imshow("gg", np.hstack([Ldstgray,velo_cam_intensity_img]))
    lastkey = cv2.waitKey(10)
    if lastkey == 27 or lastkey == 113:
        break
        time.sleep(0.2)
cv2.destroyAllWindows()




Ldst2=copy.deepcopy(Ldst)

for j in range(len(preds)):
    if j > 0:
        break
    boxes_f = preds[j]['boxes']
    labels_f = preds[j]['labels']
    scores_f = preds[j]['scores']
    for i in range(len(boxes_f)):

        cv2.rectangle(Ldst2, (boxes_f[i][0], boxes_f[i][1]), (boxes_f[i][2], boxes_f[i][3]), (0, 255, 0), 2)
        bx=boxes_f[i].copy()
        bcx=0.5*(bx[0]+bx[2])
        bcy = 0.5 * (bx[1] + bx[3])
        bcw = (bx[2]-bx[0])/2
        bch = (bx[3] - bx[1])/2
        bx[0]=bcx-bcw/2
        bx[2] = bcx + bcw / 2
        bx[1] = bcy - bch / 2
        bx[3] = bcy + bch / 2

        ii1=(Xcv_imgpts_inside_coords[:,0]>boxes_f[i][0]) &  (Xcv_imgpts_inside_coords[:,0]<boxes_f[i][2])
        ii2 = (Xcv_imgpts_inside_coords[:, 1] > boxes_f[i][1]) & (Xcv_imgpts_inside_coords[:, 1] < boxes_f[i][3])
        for x1,y1 in Xcv_imgpts_inside_coords[ii1 & ii2,:].astype(int):
            image = cv2.circle(Ldst2, (x1, y1), radius=0, color=(0, 0, 255), thickness=-1)

        ii1 = (Xcv_imgpts_inside_coords[:, 0] > bx[0]) & (Xcv_imgpts_inside_coords[:, 0] <bx[2])
        ii2 = (Xcv_imgpts_inside_coords[:, 1] > bx[1]) & (Xcv_imgpts_inside_coords[:, 1] < bx[3])
        Xbx = Xcv_front_inside[ii1 & ii2,:]
        d = np.min(Xbx[:,2])
        # cv2.putText(Ldst2, str(labels_f[i]) + " : " + str(scores_f[i]) + " || {0:.2f},{0:.2f},{0:.2f}".format(
        #     np.min(d), np.mean(d), np.max(d)), (boxes_f[i][0], boxes_f[i][1] - 10), 0, 1, (0, 255, 0), 2)
        cv2.putText(Ldst2, "{0:.2f}".format(d), (boxes_f[i][0], boxes_f[i][1] - 10), 0, 1, (0, 255, 0), 2)
        #
        # pcdi1i2 = velodynecalib.points2pcd(Xbx, index=None, colmat=Colcv[indz,:][ ii1 & ii2,:], colvec=None)
        # velodynecalib.plotpcds([pcdi1i2])


cv2.namedWindow('Depth', cv2.WINDOW_NORMAL)
while 1:
    cv2.imshow('Depth', Ldst2)
    lastkey = cv2.waitKey(10)
    if lastkey == 27 or lastkey == 113:
        break
    # if lastkey == 115:
    #     velodynecalib.plotpcds([velodynecalib.points2pcd(Xcv[indz, :], index=None, colmat=Colcv[indz, :], colvec=None)])

cv2.destroyAllWindows()