#import argparse
#import open3d as o3d
import math
import matplotlib.pyplot as plt
import numpy as np
import os
#import matplotlib.pyplot as plt
import pdb
#import skimage.io
import torch
import copy
#import torch.nn as nn
import torchvision.transforms as trchtrsnf
import torch.backends.cudnn as cudnn
#from torch.autograd import Variable
import time
#import pptk
#import torchvision.transforms.functional as F
#cudnn.benchmark = True
cudnn.benchmark = False

import torchvision
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
# from torchvision.models.detection import FasterRCNN_ResNet50_FPN_Weights
import torch
from torchvision.utils import draw_bounding_boxes
from torchvision.utils import draw_bounding_boxes
from torchvision.io import read_image
from torchvision.ops import nms
import cv2

from dataloader import kittimot_dataset

#root_dir = '/media/na0043/misc/DATA/KITTI/mots/dataset/training';
root_dir = '/run/user/1000/gvfs/sftp:host=10.42.0.1,user=na0043/media/na0043/misc/DATA/KITTI/mots/dataset/training'

seq = '0000';
kittimot = kittimot_dataset.KittiMOT(root_dir,seq)
imgL, imgR, velo, oxts, bbox, bbox3D, label, difficult=kittimot[0]

# L=[]
# for i in range(len(bbox3D)):
#     points = bbox3D[i][0].T
#     lines = []
#     for face in bbox3D[i][1]:
#         lines.append([face[0],face[1]])
#         lines.append([face[1], face[2]])
#         lines.append([face[2], face[3]])
#         lines.append([face[3], face[0]])
#     colors = [[1, 0, 0] for i in range(len(lines))]
#     line_set = o3d.geometry.LineSet(
#         points=o3d.utility.Vector3dVector(points),
#         lines=o3d.utility.Vector2iVector(lines),
#     )
#     line_set.colors = o3d.utility.Vector3dVector(colors)
#     L.append(line_set)
# 
# 
# 
# pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.Vector3dVector(velo[:,:3])
# pcd.colors = o3d.utility.Vector3dVector(velo[:,None,3]*np.ones((velo.shape[0],3)))
# 
# # radii = [0.5, 1]
# # pcd.estimate_normals()
# # rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
# #     pcd, o3d.utility.DoubleVector(radii))
# 
# o3d.visualization.draw_geometries([pcd]+L)
#
# vis = o3d.visualization.VisualizerWithKeyCallback()
# # cloud = o3d.io.read_point_cloud(out_fn)
# vis.create_window()
# # vis.register_key_callback('q', your_update_function)
# vis.add_geometry(pcd)
# vis.run()
# #
# def your_update_function():
#     #Your update routine
#     vis.update_geometry(cloud)
#     vis.update_renderer()
#     vis.poll_events()
#     vis.run()

# v = pptk.viewer(velo[:,:3])
# v.set(point_size=0.005)
methods=['fasterrcnn_resnet50_fpn','fasterrcnn_mobilenet_v3_large_fpn',
         'fasterrcnn_mobilenet_v3_large_320_fpn','ssd300_vgg16','ssdlite320_mobilenet_v3_large','yolov5s','yolov5m','yolov5l']
def getmodel(meth):
    if meth==methods[0]:
        model = torchvision.models.detection.fasterrcnn_resnet50_fpn(pretrained=True)
        model.cuda()
        model.eval()
    elif meth==methods[1]:
        model = torchvision.models.detection.fasterrcnn_mobilenet_v3_large_fpn(pretrained=True)
        model.cuda()
        model.eval()
    elif meth==methods[2]:
        model = torchvision.models.detection.fasterrcnn_mobilenet_v3_large_320_fpn(pretrained=True)
        model.cuda()
        model.eval()
    elif meth==methods[3]:
        model = torchvision.models.detection.ssd300_vgg16(pretrained=True)
        model.cuda()
        model.eval()
    elif meth==methods[4]:
        model = torchvision.models.detection.ssdlite320_mobilenet_v3_large(pretrained=True)
        model.cuda()
        model.eval()
    elif meth==methods[5]:
        model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        model.cuda()
        model.eval()
    elif meth==methods[6]:
        model = torch.hub.load('ultralytics/yolov5', 'yolov5m', pretrained=True)
        model.cuda()
        model.eval()
    elif meth==methods[7]:
        model = torch.hub.load('ultralytics/yolov5', 'yolov5l', pretrained=True)
        model.cuda()
        model.eval()

    return model



tft=trchtrsnf.Compose([
trchtrsnf.PILToTensor(),
trchtrsnf.ConvertImageDtype(torch.float),
# trchtrsnf.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
])

def get_scores(pred,bbox):
    try:
        scores = np.asarray(pred[0]['scores'].cpu().detach())
        boxes = np.asarray(pred[0]['boxes'].cpu().detach())
        labels = np.asarray(pred[0]['labels'].cpu().detach())
    except:
        scores = pred[0]['scores']
        boxes = pred[0]['boxes']
        labels = pred[0]['labels']

    idd2 = nms(torch.from_numpy(boxes), torch.from_numpy(scores), 0.1).cpu().detach().numpy()
    idd = scores[idd2] > 0.15

    scores_f = scores[idd2][idd]
    boxes_f = boxes[idd2][idd].astype(int)
    labels_f = labels[idd2][idd]

    ious = torchvision.ops.box_iou(torch.from_numpy(bbox[:, [1, 0, 3, 2]]), torch.from_numpy(boxes_f)).numpy()
    s=np.mean(np.max(ious, axis=1))
    return s,scores_f,boxes_f,labels_f
results={mm:{'time':[],'iouscore':[]} for mm in methods}
#from numba import cuda

for mm in methods:
    torch.cuda.empty_cache()
    # cuda.select_device(0)
    # cuda.close()

    model = getmodel(mm)
    for inx in range(len(kittimot)):
        print(inx,len(kittimot))
        imgL, imgR, velo, oxts, bbox, bbox3D, label, difficult=kittimot[inx]

        imgL_t = tft(imgL).cuda()
        imgR_t = tft(imgR).cuda()



        if 'yolo' in mm:
            st = time.time()
            predictions = model([imgL])
            et = time.time()
        else:
            st = time.time()
            predictions = model([imgL_t])
            et = time.time()
        print("%s detection = "%mm,et-st)
        torch.cuda.empty_cache()
        if 'yolo' in mm:
            preds = []
            for dd in predictions.pandas().xyxy:
                preds.append({'boxes': dd[['xmin', 'ymin', 'xmax', 'ymax']].values.astype(np.float32),
                                    'scores': dd['confidence'].values.astype(np.float32),
                                    'labels': dd['name'].values})
        else:
            preds= predictions
        s,scores_f,boxes_f,labels_f=get_scores(preds, bbox)
        results[mm]['time'].append(et-st)
        results[mm]['iouscore'].append(s)

    # st=time.time()
    # predictions_fstmblnet_high = fasterRcnnmodel_mblnet_highres([imgL_t])
    # et=time.time()
    # print("fasterRcnnmodel_mblnet_highres detection = ",et-st)
    # torch.cuda.empty_cache()
    # s,scores_f,boxes_f,labels_f=get_scores(predictions_fstmblnet_high, bbox)
    # results['fasterRcnnmodel_mblnet_highres']['time'].append(et - st)
    # results['fasterRcnnmodel_mblnet_highres']['iouscore'].append(s)
    #
    # st=time.time()
    # prediction_fstsmblnet_low = fasterRcnnmodel_mblnet_lowres([imgL_t])
    # et=time.time()
    # print("fasterRcnnmodel_mblnet_lowres detection = ",et-st)
    # s,scores_f,boxes_f,labels_f=get_scores(prediction_fstsmblnet_low, bbox)
    # results['fasterRcnnmodel_mblnet_lowres']['time'].append(et - st)
    # results['fasterRcnnmodel_mblnet_lowres']['iouscore'].append(s)
    #
    # # st=time.time()
    # # predictions_fst = fasterRcnnmodel([imgL_t])
    # # et=time.time()
    # # print("fasterRcnnmodel detection = ",et-st)
    # # torch.cuda.empty_cache()
    # # s,scores_f,boxes_f,labels_f=get_scores(predictions_fst, bbox)
    #
    #
    # st=time.time()
    # predictions_ssd = ssdvgg16([imgL_t])
    # et=time.time()
    # print("ssdvgg16 detection = ",et-st)
    # torch.cuda.empty_cache()
    # s,scores_f,boxes_f,labels_f=get_scores(predictions_ssd, bbox)
    # results['ssdvgg16']['time'].append(et - st)
    # results['ssdvgg16']['iouscore'].append(s)
    #
    # st=time.time()
    # predictions_ssdlite = ssdlite([imgL_t])
    # et=time.time()
    # print("ssdlite detection = ",et-st)
    # torch.cuda.empty_cache()
    # s,scores_f,boxes_f,labels_f=get_scores(predictions_ssdlite, bbox)
    # results['ssdlite']['time'].append(et - st)
    # results['ssdlite']['iouscore'].append(s)
    #
    # st=time.time()
    # predictions_yolo5s = yolov5s([imgL])
    # et=time.time()
    # print("yolov5s detection = ",et-st)
    # torch.cuda.empty_cache()
    # pred_yolo5s=[]
    # for dd in predictions_yolo5s.pandas().xyxy:
    #     pred_yolo5s.append({'boxes':dd[['xmin','ymin','xmax','ymax']].values.astype(np.float32),
    #                         'scores':dd['confidence'].values.astype(np.float32),
    #                         'labels':dd['name'].values})
    # s,scores_f,boxes_f,labels_f=get_scores(pred_yolo5s, bbox)
    # results['yolov5s']['time'].append(et - st)
    # results['yolov5s']['iouscore'].append(s)
    #
    # st=time.time()
    # predictions_yolo5m = yolov5m([imgL])
    # et=time.time()
    # print("yolov5m detection = ",et-st)
    # torch.cuda.empty_cache()
    # pred_yolo5m=[]
    # for dd in predictions_yolo5m.pandas().xyxy:
    #     pred_yolo5m.append({'boxes':dd[['xmin','ymin','xmax','ymax']].values.astype(np.float32),
    #                         'scores':dd['confidence'].values.astype(np.float32),
    #                         'labels':dd['name'].values})
    # s,scores_f,boxes_f,labels_f=get_scores(pred_yolo5m, bbox)
    # results['yolov5m']['time'].append(et - st)
    # results['yolov5m']['iouscore'].append(s)

    # st=time.time()
    # predictions_yolo5m = yolov5l([imgL])
    # et=time.time()
    # print("yolov5l detection = ",et-st)
    # torch.cuda.empty_cache()
    # pred_yolo5m=[]
    # for dd in predictions_yolo5m.pandas().xyxy:
    #     pred_yolo5m.append({'boxes':dd[['xmin','ymin','xmax','ymax']].values.astype(np.float32),
    #                         'scores':dd['confidence'].values.astype(np.float32),
    #                         'labels':dd['name'].values})
    # s,scores_f,boxes_f,labels_f=get_scores(pred_yolo5m, bbox)
    # results['yolov5l']['time'].append(et - st)
    # results['yolov5l']['iouscore'].append(s)

    # st=time.time()
    # predictions_yolo5m = yolov5x([imgL])
    # et=time.time()
    # print("yolov5x detection = ",et-st)
    # torch.cuda.empty_cache()
    # pred_yolo5m=[]
    # for dd in predictions_yolo5m.pandas().xyxy:
    #     pred_yolo5m.append({'boxes':dd[['xmin','ymin','xmax','ymax']].values.astype(np.float32),
    #                         'scores':dd['confidence'].values.astype(np.float32),
    #                         'labels':dd['name'].values})
    # s,scores_f,boxes_f,labels_f=get_scores(pred_yolo5m, bbox)
    # results['yolov5x']['time'].append(et - st)
    # results['yolov5x']['iouscore'].append(s)

    # scores = pred[0]['scores']
    
    # scores=np.asarray(pred[0]['scores'].cpu().detach())
    # boxes = np.asarray(pred[0]['boxes'].cpu().detach())
    # labels = np.asarray(pred[0]['labels'].cpu().detach())
    #
    # idd2=nms(pred[0]['boxes'],pred[0]['scores'],0.1).cpu().detach().numpy()
    # idd = scores[idd2]>0.15
    #
    # scores_f = scores[idd2][idd]
    # boxes_f = boxes[idd2][idd].astype(int)
    # labels_f = labels[idd2][idd]
    
    # cv2imgg = np.asarray(T.ToPILImage()(img))
    # cv2imgg=np.asarray(imgL)
    # drawn_boxes = draw_bounding_boxes(img, pred[0]['boxes'][idd2][idd], colors="red")
    # for i in range(len(scores_f)):
    #     cv2.rectangle(cv2imgg,(boxes_f[i][0],boxes_f[i][1]),(boxes_f[i][2],boxes_f[i][3]),(0,255,0),2)
    #     cv2.putText(cv2imgg,str(labels_f[i]),(boxes_f[i][0],boxes_f[i][1]-10),0,0.3,(0,255,0))


    #
    # cv2.imshow('boxes',cv2imgg )
    # key = cv2.waitKey(0)
    #
    # if key == 27 or key == 113:
    #     break
    # cv2.destroyAllWindows()
    
import pickle as pkl
with open("targetdetection_results.pkl",'wb') as F:
    pkl.dump(results,F)

with open("targetdetection_results.pkl",'rb') as F:
    results=pkl.load(F)
for mm in results.keys():
    fig, axs = plt.subplots(2)
    axs[0].hist(1/np.array(results[mm]['time']),bins=np.linspace(0,150))
    axs[1].hist(np.array(results[mm]['iouscore']), bins=np.linspace(0, 1))
    axs[0].set_title(mm)
    axs[0].set(xlabel='Hz', ylabel='#')
    axs[1].set(xlabel='iouscore', ylabel='#')
    plt.savefig('%s.png'%mm)

plt.show()
