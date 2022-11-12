import cv2
import open3d as o3d
import os
from scipy.spatial.transform import Rotation as RR
import copy
import scipy.optimize
import pickle as pkl
import numpy as np
import matplotlib.pyplot as plt
import time
from sklearn.cluster import AgglomerativeClustering
from sklearn.cluster import KMeans
from sklearn.cluster import AffinityPropagation
from camerahardware import calib_codes
from velohardware import velodynecalib
from scipy.spatial import KDTree


# with open(os.path.join('calibfiles', 'velo_step_calib.pkl'), 'rb') as F:
#     [Hest, Hest_opt] = pkl.load(F)
#
# step=0
# ang= (step-0)*0.9/40
# Hr = Hmat([-ang,0,0],[0,0,0])
# Hrv=Hest_opt
# Hvr=np.linalg.inv(Hrv)
# H=np.linalg.multi_dot([Hvr,Hr,Hrv])
# Hinv = np.linalg.inv(H)
# rHs=Hinv   .... source to local 0-step rotation frame
# Hrv : velodyne to rotation frame


folder='/media/na0043/misc/DATA/cam_velo_stepper/2022-11-07-16-38-06'
pcdfile = os.path.join(folder,'cummulative_pcd.pcd')
dsquare=0.175


pcd = o3d.io.read_point_cloud(pcdfile)
Xv,Colv = velodynecalib.pcd2points(pcd)

indf=(Xv[:,0]<0) & (Xv[:,2]<0)
pcdf=velodynecalib.points2pcd(Xv,index=indf,colmat=Colv)


pcd_clustered,labels_clustered = velodynecalib.dbscan_cluster(pcd)
velodynecalib.plotpcds([pcd_clustered])

objp_checker=velodynecalib.extractCheckerboard_from_clusters(pcd,pcd_clustered,labels_clustered,dsquare)

# now get image points----------------------------
cimg=cv2.imread(os.path.join(folder, 'cam_00.png'))
img = cv2.imread(os.path.join(folder, 'cam_00.png'),0)
h,  w = cimg.shape[:2]

dst = cv2.medianBlur(img,5)
dst = cv2.adaptiveThreshold(dst,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,91,10)

ret, cor = cv2.findChessboardCorners(dst, (4, 4), cv2.CALIB_CB_FAST_CHECK  + cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FILTER_QUADS)
print(ret)
if ret:
    st = time.time()
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    et = time.time()
    # print("subpixel refine = ", et - st)
    cor = cv2.cornerSubPix(dst, cor, (11, 11), (-1, -1), criteria)


for j in range(len(cor)):
    cv2.putText(cimg, "%d"%j, cor[j][0].astype(np.int)+10, fontFace = cv2.FONT_HERSHEY_COMPLEX, fontScale = 1.1, color = (250,225,100))
cv2.drawChessboardCorners(cimg, (4, 4), cor, ret)


velodynecalib.plotimage(cimg)




order_img_points=[3,7,11,15,
                  2,6,10,14,
                  1,5,9,13,
                  0,4,8,12]
img_pts=cor[order_img_points]


with open("calibrated_camera_parameters.pkl","rb") as F:
    calib_parameters = pkl.load(F)

Lmtx, Ldist =calib_parameters[1]['mono'][0][1:3]
ret,rvec,tvec,err=	cv2.solvePnPGeneric(objp_checker[:-4].astype(np.float32), img_pts, Lmtx, Ldist)

tvec=tvec[0].reshape(-1)
Rcv,_ = cv2.Rodrigues(rvec[0])
objp_cam = Rcv.dot(objp_checker.T).T+tvec
with open("calibfiles/velo_cam_calib.pkl",'wb') as F:
    pkl.dump([Rcv,tvec],F)