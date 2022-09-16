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
folder = 'calibration_images'

def plotcv(img):
    while(1):
        cv.namedWindow('img', cv.WINDOW_NORMAL) 
        cv.imshow('img', img)
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
    plotcv(Limg)
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
x, y, w, h = Lroi
croppeddst = Lundstimg[y:y+h, x:x+w]

plotcv(np.hstack([Limg,Lundstimg]))


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
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
retval, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = cv.stereoCalibrate(objpoints,imgpointsL,imgpointsR,Lmtx,Ldist,Rmtx,Rdist,(w,h))


R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv.stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, (w,h), R, T)

#%% just using image points
retval, H1, H2=cv.stereoRectifyUncalibrated(imgpointsL[0],imgpointsR[0],F,(w,h))