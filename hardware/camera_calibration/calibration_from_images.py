#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Sep  9 12:28:32 2022

@author: nvidiaorin
"""

import numpy as np
import cv2 as cv
import glob
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((10*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:10].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpointsL = [] # 2d points in image plane.
imgpointsR = [] # 2d points in image plane.
images = glob.glob('calibration_images/*.png')
images=sorted(images)
for fname in images:
    print(fname)
    
    img = cv.imread(fname)
    n=img.shape[1]
    Limg = img[:,:int(n/2),:]
    Rimg = img[:,int(n/2):,:]
    
    grayL = cv.cvtColor(Limg, cv.COLOR_BGR2GRAY)
    grayR = cv.cvtColor(Rimg, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    retL, cornersL = cv.findChessboardCorners(grayL, (7,10), None)
    retR, cornersR = cv.findChessboardCorners(grayR, (7,10), None)
    
    # If found, add object points, image points (after refining them)
    if retL == True and retR == True:
        objpoints.append(objp)
        corners2L = cv.cornerSubPix(grayL,cornersL, (11,11), (-1,-1), criteria)
        imgpointsL.append(corners2L)
        
        corners2R = cv.cornerSubPix(grayR,cornersR, (11,11), (-1,-1), criteria)
        imgpointsR.append(corners2R)
        
        
        # Draw and display the corners
        cv.drawChessboardCorners(Limg, (7,6), corners2L, retL)
        cv.drawChessboardCorners(Rimg, (7,6), corners2R, retR)
        cv.imshow('img', np.hstack([Limg,Rimg]))
        cv.waitKey(500)
cv.destroyAllWindows()