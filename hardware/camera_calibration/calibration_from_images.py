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

np.savez(os.path.join(folder,'good_calib_points'),objpoints=objpoints,imgpointsL=imgpointsL,imgpointsR=imgpointsR)    

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpointsL, grayL.shape[::-1], None, None)
img = cv.imread(fname)
h,  w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))


dst = cv.undistort(img, mtx, dist, None, newcameramtx)
# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]

# undistort
mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)
dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]

#reprojection error
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error
print( "total error: {}".format(mean_error/len(objpoints)) )