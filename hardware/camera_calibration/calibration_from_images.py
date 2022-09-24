#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Sep  9 12:28:32 2022

@author: nvidiaorin
"""

import numpy as np
import cv2 as cv
import os
import glob
import time
import open3d as o3d

folder = 'calibration_images'

def plotcv(imglist):
    for name,img in imglist:
        cv.namedWindow(name, cv.WINDOW_NORMAL) 
        cv.imshow(name, img)
        
    while(1):
        k = cv.waitKey(33)
        if k==27 or k==113:    # Esc key to stop
            break
#        if k==115:
#            print("saved")
#            objpoints.append(objp)
#            imgpointsL.append(corners2L)
#            imgpointsR.append(corners2R)
#            break
#        elif k==-1:  # normally -1 returned,so don't print it
#            continue
#        else:
#            print(k) # else print its value
    cv.destroyAllWindows()

 
#%%
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((10*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:10].T.reshape(-1,2)
objp = objp*0.24
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpointsL = [] # 2d points in image plane.
imgpointsR = [] # 2d points in image plane.
images = glob.glob(folder+'/*.png')
images=sorted(images)
for i in range(len(images)):
    fname=os.path.join(folder,'image_%d.png'%i)
    
    print(fname)
    
    img = cv.imread(fname)
    
    
    n=img.shape[1]
    Limg = img[:,:int(n/2),:]
    Rimg = img[:,int(n/2):,:]
    h,  w = Limg.shape[:2]
#    while(1):
#        cv.namedWindow('img0', cv.WINDOW_NORMAL) 
#        cv.imshow('img0',img)
#        k = cv.waitKey(33)
#        if k==27:    # Esc key to stop
#            break
#        elif k==-1:  # normally -1 returned,so don't print it
#            continue
#        else:
#            print(k) # else print its value
#    cv.destroyAllWindows()
    
    grayL = cv.cvtColor(Limg, cv.COLOR_BGR2GRAY)
    grayR = cv.cvtColor(Rimg, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    retL, cornersL = cv.findChessboardCornersSB(grayL, (7,10), None)
    retR, cornersR = cv.findChessboardCornersSB(grayR, (7,10), None)
    
    fm=cv.Laplacian(Limg, cv.CV_64F).var()
	cv.putText(Limg, "blurry: {:.2f}".format(fm), ( int(w/2),int(h/2) ),
	cv.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 255), 5)
    plotcv(["left_image",Limg])
    # If found, add object points, image points (after refining them)
    if retL == True and retR == True:
        
        
        corners2L = cv.cornerSubPix(grayL,cornersL, (11,11), (-1,-1), criteria)

        
        corners2R = cv.cornerSubPix(grayR,cornersR, (11,11), (-1,-1), criteria)
        
        
        
        # Draw and display the corners
#        cv.namedWindow('Window', cv.WINDOW_NORMAL) 
        cv.drawChessboardCorners(Limg, (7,10), corners2L, retL)
        cv.drawChessboardCorners(Rimg, (7,10), corners2R, retR)
        
        
        while(1):
            cv.namedWindow('img', cv.WINDOW_NORMAL) 
            cv.imshow('img', np.hstack([Limg,Rimg]))
            k = cv.waitKey(33)
            if k==27 or k==113:    # Esc key to stop
                break
            if k==115:
                print("saved")
                objpoints.append(objp)
                imgpointsL.append(corners2L)
                imgpointsR.append(corners2R)
                break
            elif k==-1:  # normally -1 returned,so don't print it
                continue
            else:
                print(k) # else print its value
        cv.destroyAllWindows()

np.savez(os.path.join(folder,'good_calib_points.npz'),objpoints=objpoints,imgpointsL=imgpointsL,imgpointsR=imgpointsR)    

#%%
data=np.load(os.path.join(folder,'good_calib_points.npz'))    
objpoints=data['objpoints']
for i in range(len(objpoints)):
    objpoints[i]=objpoints[i]*0.024

imgpointsL=data['imgpointsL']
imgpointsR=data['imgpointsR']

fname=os.path.join(folder,'image_%d.png'%129)
img = cv.imread(fname)
n=img.shape[1]
Limg = img[:,:int(n/2),:]
Rimg = img[:,int(n/2):,:]
grayL = cv.cvtColor(Limg, cv.COLOR_BGR2GRAY)
h,  w = Limg.shape[:2]

Lret, Lmtx, Ldist, Lrvecs, Ltvecs = cv.calibrateCamera(objpoints, imgpointsL, (w,h), None, None)
Lnewcameramtx, Lroi = cv.getOptimalNewCameraMatrix(Lmtx, Ldist, (w,h), 1, (w,h))

Rret, Rmtx, Rdist, Rrvecs, Rtvecs = cv.calibrateCamera(objpoints, imgpointsR, (w,h), None, None)
Rnewcameramtx, Rroi = cv.getOptimalNewCameraMatrix(Rmtx, Rdist, (w,h), 1, (w,h))

Lundstimg = cv.undistort(Limg, Lmtx, Ldist, None, Lnewcameramtx)
xc, yc, wc, hc = Lroi
croppeddst = Lundstimg[yc:yc+hc, xc:xc+wc]

L1 = Limg.copy()
L2 = Lundstimg.copy()
line_thickness = 2
for x in np.linspace(0+10,h-10,20):
    cv.line(L1, (0,int(x)), (w,int(x)), (0, 255, 0), thickness=line_thickness)
    cv.line(L2, (0,int(x)), (w,int(x)), (255, 0, 0), thickness=line_thickness)
    
gg=np.hstack([L1,L2])
plotcv([['gg',gg]])


# crop the image


#cv.destroyAllWindows()

# undistort
#mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)
#dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)
#x, y, w, h = roi
#dst = dst[y:y+h, x:x+w]

#reprojection error
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], Lrvecs[i], Ltvecs[i], Lmtx, Ldist)
    error = cv.norm(imgpointsL[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error
print( "total left error: {}".format(mean_error/len(objpoints)) )

mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], Rrvecs[i], Rtvecs[i], Rmtx, Rdist)
    error = cv.norm(imgpointsR[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error
print( "total right error: {}".format(mean_error/len(objpoints)) )

#%% stereo calibrate
h,  w = Limg.shape[:2]

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
retval, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = cv.stereoCalibrate(objpoints,imgpointsL,imgpointsR,Lmtx,Ldist,Rmtx,Rdist,(w,h))


R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv.stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, (w,h), R, T)



Lmapx,Lmapy=cv.initUndistortRectifyMap(	cameraMatrix1, distCoeffs1, R1, P1[0:3,0:3], (w,h), cv.CV_32FC2	) 
Rmapx,Rmapy=cv.initUndistortRectifyMap(	cameraMatrix2, distCoeffs2, R2, P2[0:3,0:3], (w,h), cv.CV_32FC2	) 


L1 = Limg.copy()
R1 = Rimg.copy()

#gg=np.hstack([L1,R1])
#plotcv([['gg',gg]])


Ldst = cv.remap(L1, Lmapx, Lmapy, cv.INTER_LINEAR)
Rdst = cv.remap(R1, Rmapx, Rmapy, cv.INTER_LINEAR)
line_thickness = 2
for x in np.linspace(0+10,h-10,50):
    cv.line(Ldst, (0,int(x)), (w,int(x)), (0, 255, 0), thickness=line_thickness)
    cv.line(Rdst, (0,int(x)), (w,int(x)), (255, 0, 0), thickness=line_thickness)
    
    cv.line(L1, (0,int(x)), (w,int(x)), (0, 255, 0), thickness=line_thickness)
    cv.line(R1, (0,int(x)), (w,int(x)), (255, 0, 0), thickness=line_thickness)
    
    
im1=np.hstack([L1,R1])
im2=np.hstack([Ldst,Rdst])
plotcv([['im1',im1],['im2',im2]])

minDisparity=16
stereo_sgm_cuda = cv.StereoSGBM_create(minDisparity=minDisparity,
                                            numDisparities=148,
                                            blockSize=3,
                                            disp12MaxDiff=5,
                                            P1=7,
                                            P2=20,
                                            mode=4,
                                            uniquenessRatio=5
                                            )
st=time.time()
disp = stereo_sgm_cuda.compute(Ldst,Rdst).astype(np.float32) /16
et = time.time()
print(et-st)

points = cv.reprojectImageTo3D(disp, Q)
colors = cv.cvtColor(Ldst, cv.COLOR_BGR2RGB)
mask = disp > disp.min()
out_points = points[mask]
out_colors = colors[mask]

ind = np.linalg.norm(out_points,axis=1)<=100

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(out_points[ind])
pcd.colors = o3d.utility.Vector3dVector(out_colors[ind]/255)

o3d.visualization.draw_geometries([pcd])

#%% just using image points
sift = cv.SIFT_create()
L1 = cv.imread("L2.png")
R1 = cv.imread("R2.png")
D1 = cv.imread("D2.png")

h,  w = L1.shape[:2]

#plotcv([['im',np.hstack([L1,R1])]])
# find the keypoints and descriptors with SIFT
kp1, des1 = sift.detectAndCompute(L1, None)
kp2, des2 = sift.detectAndCompute(R1, None)

imgSift = cv.drawKeypoints(
    L1, kp1, None, flags=cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
#plotcv([["SIFT Keypoints", imgSift]])

FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
search_params = dict(checks=50)   # or pass empty dictionary
flann = cv.FlannBasedMatcher(index_params, search_params)
matches = flann.knnMatch(des1, des2, k=2)

# Keep good matches: calculate distinctive image features
# Lowe, D.G. Distinctive Image Features from Scale-Invariant Keypoints. International Journal of Computer Vision 60, 91–110 (2004). https://doi.org/10.1023/B:VISI.0000029664.99615.94
# https://www.cs.ubc.ca/~lowe/papers/ijcv04.pdf
matchesMask = [[0, 0] for i in range(len(matches))]
good = []
pts1 = []
pts2 = []

for i, (m, n) in enumerate(matches):
    if m.distance < 0.7*n.distance:
        # Keep this keypoint pair
        matchesMask[i] = [1, 0]
        good.append(m)
        pts2.append(kp2[m.trainIdx].pt)
        pts1.append(kp1[m.queryIdx].pt)

pts1 = np.int32(pts1)
pts2 = np.int32(pts2)
fundamental_matrix, inliers = cv.findFundamentalMat(pts1, pts2, cv.FM_RANSAC)

# We select only inlier points
pts1 = pts1[inliers.ravel() == 1]
pts2 = pts2[inliers.ravel() == 1]



retval, H1, H2=cv.stereoRectifyUncalibrated(pts1,pts2,F,(w,h))

Ldst = cv.warpPerspective(L1, H1, (w, h))
Rdst = cv.warpPerspective(R1, H2, (w, h))

line_thickness = 2
for x in np.linspace(0+10,h-10,30):
    cv.line(Ldst, (0,int(x)), (w,int(x)), (0, 255, 0), thickness=line_thickness)
    cv.line(Rdst, (0,int(x)), (w,int(x)), (255, 0, 0), thickness=line_thickness)
    
    cv.line(L1, (0,int(x)), (w,int(x)), (0, 255, 0), thickness=line_thickness)
    cv.line(R1, (0,int(x)), (w,int(x)), (255, 0, 0), thickness=line_thickness)
 
im1=np.hstack([L1,R1])
im2=np.hstack([Ldst,Rdst])
plotcv([['im1',im1],['im2',im2]])

minDisparity=16
stereo_sgm_cuda = cv.StereoSGBM_create(minDisparity=minDisparity,
                                            numDisparities=148,
                                            blockSize=3,
                                            disp12MaxDiff=5,
                                            P1=170,
                                            P2=284,
                                            mode=4,
                                            uniquenessRatio=5
                                            )
st=time.time()
disp = stereo_sgm_cuda.compute(Ldst,Rdst).astype(np.float32) /16
et = time.time()
print(et-st)

points = cv.reprojectImageTo3D(disp, Q)
colors = cv.cvtColor(Ldst, cv.COLOR_BGR2RGB)
mask = disp > disp.min()
out_points = points[mask]
out_colors = colors[mask]

ind = np.linalg.norm(out_points,axis=1)<=100

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(out_points[ind])
pcd.colors = o3d.utility.Vector3dVector(out_colors[ind]/255)

o3d.visualization.draw_geometries([pcd])
