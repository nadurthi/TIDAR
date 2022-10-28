#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Sep  9 12:28:32 2022

@author: nvidiaorin
"""

import numpy as np
import cv2 as cv
import cv2
import os
import glob
import time
import open3d as o3d
import matplotlib.pyplot as plt
import collections as clc
import pickle as pkl
from mpl_toolkits.mplot3d import Axes3D

#folder = 'camera_saved_data/2022-10-04-18-36-04'


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

D=[[0,1,53.5],
   [1,2,56],
   [2,3,62],
   [3,4,60.5],
   [0,5,61],
   [5,6,53],
   [6,7,62],
   [7,8,58],
   [6,9,41],
   [5,9,67.5],
   [1,5,46.5],
   [2,9,51],
   [3,9,81],
   [8,9,126],
   [4,8,173.5],
   [0,9,120.5],
   [3,7,131]
   ]
   
   
def solveforObjPoints(X):
    Y=np.zeros((10,2))
    Y[1:,:] = X.reshape(9,2)
    d=0
    for i in range(len(D)):
       d+=(np.linalg.norm(Y[D[i][0]]-Y[D[i][1]]) -D[i][2])**2
    return d

#from scipy.optimize import dual_annealing,minimize
#lw = [0] * 18
#up = [400] * 18
#ret = dual_annealing(solveforObjPoints, bounds=list(zip(lw, up)))
#c = np.cos(-30*np.pi/180)
#s = np.sin(-30*np.pi/180)
#X=[[54,0],[110,0],[172,0],[232,0],[61*c,61*s],[104*c,104*s],[166*c,166*s],[224*c,224*s],[120.5*np.cos(-15*np.pi/180),120.5*np.sin(-15*np.pi/180)]]
#ret2=minimize(solveforObjPoints, np.array(X).reshape(-1), method='BFGS')
#                        
#Y=np.zeros((10,2))
#Y[1:,:] = ret2['x'].reshape(9,2)
#plt.plot(Y[:,0],Y[:,1],'ro')
#
#with open(os.path.join(folder,"good_custom_calibration_pts.pkl"),'rb') as F:
#    images_pts = pkl.load(F)
    
#%%
    

folder = 'camera_saved_data/2022-10-08-18-54-10'


def getchessboardpattern(folder,Ncam=3):
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((5*4,3), np.float32)
    objp[:,:2] = np.mgrid[0:4,0:5].T.reshape(-1,2)
    objp = objp*0.175
    # Arrays to store object points and image points from all the images.
    objpoints = {} # 3d point in real world space
    imgpoints = [{},{},{}] # 2d points in image plane.

    ff=os.listdir(folder)
    Nset = int(len(ff)/Ncam)
    cv.namedWindow('img', cv.WINDOW_NORMAL) 
    for si in range(Nset):
        for cj in range(Ncam): #Ncam
            print(si,cj)
            fname = "%05d_%02d.png"%(si,cj)
            Limg = cv2.imread(os.path.join(folder,fname))
            if Limg is None:
                continue

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
            
            # Find the chess board corners
            retL, cornersL = cv.findChessboardCornersSB(grayL, (4,5), cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_NORMALIZE_IMAGE)
            if retL==False:
                print("failed: ",si,cj)
            
#            fm=cv.Laplacian(Limg, cv.CV_64F).var()
#            cv.putText(Limg, "blurry: {:.2f}".format(fm), ( int(w/2),int(h/2) ),cv.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 255), 5)
#            plotcv(["left_image",Limg])
            # If found, add object points, image points (after refining them)
            if retL == True:
#                print("failed: ",si,cj)
                
                corners2L = cv.cornerSubPix(grayL,cornersL, (11,11), (-1,-1), criteria)
        
                
#                corners2R = cv.cornerSubPix(grayR,cornersR, (11,11), (-1,-1), criteria)
                
                
                
                # Draw and display the corners
        #        cv.namedWindow('Window', cv.WINDOW_NORMAL) 
                cv.drawChessboardCorners(Limg, (4,5), corners2L, retL)
#                cv.drawChessboardCorners(Rimg, (7,10), corners2R, retR)
                
                corners2L=np.vstack(corners2L)
                mn=np.min(corners2L,axis=0)
                mx=np.max(corners2L,axis=0)
                mn=mn.astype(int)
                mx=mx.astype(int)
#                print(mn,mx)
                while(1):
                    
                    cv.imshow('img', Limg[max((mn[1]-100),0):min((mx[1]+100),h),max((mn[0]-100),0):min((mx[0]+100),w)]) #[mn[0]:mx[0],mn[1]:mx[1]]
                    k = cv.waitKey(33)
                    if k==27 or k==113:    # Esc key to stop
                        break
                    if k==115:
                        print("saved")
                        objpoints[si]=objp
                        imgpoints[cj][si]=corners2L
                        break
                    elif k==-1:  # normally -1 returned,so don't print it
                        continue
                    else:
                        print(k) # else print its value
    cv.destroyAllWindows()
    with open(os.path.join(folder,'good_calib_points.pkl'),'wb') as F:
        pkl.dump([objpoints,imgpoints],F)

    return objpoints,imgpoints


def removept(X,pt):
    flg=None
    for i in range(len(X)):
        if np.array_equal(X[i],pt):
            flg=1
            break
    if flg is None:
        return X
    return np.delete(X,i,axis=0)

def closestpt(X,pt):
    d=np.linalg.norm(np.array(X)-np.array(pt),axis=1)
    ind = np.argmin(d)
    return X[ind]


def getcustompattern(folder,Ncam):
#    obj_pts=[[0,0],]
    ff=os.listdir(folder)
    Nset = int(len(ff)/Ncam)
    cv.namedWindow("Image pt confirm", cv.WINDOW_NORMAL) 
    images_pts=[]    
    for si in range(Nset):
        for cj in range(Ncam): #Ncam
            print(si,cj)
            fname = "%05d_%02d.png"%(si,cj)
            img = cv2.imread(os.path.join(folder,fname))
            try:
                C=detectcustompattern(img)
            except:
                print("failed for ",si,cj)
                continue
            for i in range(len(C)):
                cv2.putText(img, '%d'%i, (C[i][0],C[i][1]), cv2.FONT_HERSHEY_SIMPLEX, 
                               1, (0,0,255), 2, cv2.LINE_AA)
            
            cv.imshow("Image pt confirm", img)
            while(1):
                k = cv.waitKey(33)
                if k==27 or k==113:    # Esc key to stops
                    break
                if k==115:
                    images_pts.append([si,cj,C])
                    print("saved")
                    break
    cv.destroyAllWindows()
    return images_pts
#i=7
#j=1
#fname = "%05d_%02d.png"%(i,j)
#img = cv2.imread(os.path.join(folder,fname))
#plotcv([["test",img]])
def detectcustompattern(img):
    
#    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
#    hsv[:,:,0]=255
#    hsv[:,:,1]=255
#    lab[:,:,1]=lab[:,:,0]
#    lab[:,:,2]=lab[:,:,0]

    
#    alpha=0.88
#    beta=1
#    new_image = np.clip(alpha*img+beta,0,255).round()
#    plotcv([["img",img],["new_image",new_image],["hsv",hsv],["lab",lab]])
    
    #h, s, v = cv2.split(hsv)
    #value=200
    #lim = 255 - value
    #v[v > lim] = 255
    #v[v <= lim] += value
    #
    #final_hsv = cv2.merge((h, s, v))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#    blurred = cv2.GaussianBlur(gray, (1, 1), 0)
    
    thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)[1]
    
#    thresh = cv2.erode(thresh, None, iterations=2)
    thresh = cv2.dilate(thresh, None, iterations=4)
    
    
#    plotcv([["test",img],["test2",thresh]])
    
    contours,hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    centers=[]
    if len(contours)>=10:
        for c in contours:
           # calculate moments for each contour
           M = cv2.moments(c)
        
           # calculate x,y coordinate of center
           cX = int(M["m10"] / M["m00"])
           cY = int(M["m01"] / M["m00"])
        #   cv2.circle(img, (cX, cY), 5, (255, 0, 0), -1)
           centers.append([cX,cY])
    
    centers = np.array(centers)
    
    # now estimate lines
#    lines=[]
#    for i in range(len(centers)):
#        for j in range(len(centers)):
#            if i==j:
#                continue
#            x1,y1 = centers[i]
#            x2,y2 = centers[j]
#            m = (y2-y1)/(x2-x1)
#            th=np.arctan2(y2-y1,x2-x1)
#            c = y1-m*x1
#            lines.append([m,c,th])
#            
#    lines = np.array(lines)
#    hist,bins=np.histogram(lines[:,0],bins=np.sort(np.tan(np.arange(-np.pi,np.pi,5*np.pi/180))),density=True) 
#    ind=np.argsort(hist)
#    hist = hist[ind]
#    bins = bins[ind]
#    slope_2best = bins[-2:]
#    
    thresh=10
    cnt=0
    sols=[]
    DD=[]
    while cnt<200:
        ind = np.random.choice(len(centers),2)
        if ind[0]==ind[1]:
            continue
        idx=list(ind)
        for j in range(len(centers)):
            if j in ind:
                continue
            idx2 = idx+[j]
            idx2 = np.sort(idx2)
            A=np.vstack([centers[idx2,0],np.ones(len(idx2))]).T
            B=centers[idx2,1]
            x,residuals,rnk,s=np.linalg.lstsq(A,B)
            rr = np.abs(A.dot(x)-B)
            if np.all(rr<thresh):
                idx.append(j)
        if len(idx)>=3:
            A=np.vstack([centers[idx,0],np.ones(len(idx))]).T
            B=centers[idx,1]
            x,residuals,rnk,s=np.linalg.lstsq(A,B)
            sols.append(x)
            DD.append(tuple(np.sort(idx)))
        cnt+=1
    
    A=clc.Counter(DD)
    vv=np.sort(list(A.values()))[::-1]
    II=[]
    IIsol=[]
    for i in range(2):
        for idxs,v in A.items():
            if v == vv[i]:
                II.append(idxs)
                a=np.vstack([centers[idxs,0],np.ones(len(idxs))]).T
                b=centers[idxs,1]
                x,residuals,rnk,s=np.linalg.lstsq(a,b)
                IIsol.append(x)
                break
    
    IIsol=sorted(IIsol,key=lambda x:x[0])
    m1,c1 = IIsol[0]
    m2,c2 = IIsol[1] 
    x0 =(c2-c1)/ (m1-m2)
    y0 = m1*x0+c1
    c0 = closestpt(centers,(x0,y0))
    C=[]
    C.append(c0)
    
    centers2 = centers.copy()
    centers2=removept(centers2,c0)
    cc=c0
    while 1:
        if len(centers2)==0:
            break
        cc=closestpt(centers2,cc)
        centers2=removept(centers2,cc)
        e1 = np.abs(cc[1]-(m1*cc[0]+c1))
        e2 = np.abs(cc[1]-(m2*cc[0]+c2))
        if e1<e2 and e1<20:
            C.append(cc)
            
    centers2 = centers.copy()
    centers2=removept(centers2,c0)
    cc=c0
    while 1:
        if len(centers2)==0:
            break
        cc=closestpt(centers2,cc)
        centers2=removept(centers2,cc)
        e1 = np.abs(cc[1]-(m1*cc[0]+c1))
        e2 = np.abs(cc[1]-(m2*cc[0]+c2))
        if e2<e1 and e2<20:
            C.append(cc)
            
    
    
    centers2 = centers.copy()
    C=np.array(C,dtype=int)
    for i in range(len(C)):
        centers2=removept(centers2,C[i])
            
    C=np.array(C,dtype=int)
    C=np.vstack([C,centers2])
    
    
    return C

#C=detectcustompattern(img)
#plotcv([["test3",img]])
objpoints,imgpoints=getchessboardpattern(folder,3)
#with open(os.path.join(folder,"good_custom_calibration_pts.pkl"),'wb') as F:
#    pkl.dump(images_pts,F)
#    

#%% 
calib_parameters={}
imgpointsL_list=[]
imgpointsM_list=[]
imgpointsR_list=[]
objpts_list=[]

all_folders=[#'camera_saved_data/2022-10-08-16-45-44',
             #'camera_saved_data/2022-10-08-17-16-49',
             #'camera_saved_data/2022-10-08-17-33-24',
             #'camera_saved_data/2022-10-08-17-39-12bad',
             #'camera_saved_data/2022-10-08-17-54-44bad',
             'camera_saved_data/2022-10-08-18-12-02'
             ]
for ff in all_folders:
    with open(os.path.join(ff,'good_calib_points.pkl'),'rb') as F:
        objp,imgp = pkl.load(F)

    for i in range(len(objp)):
        if i in imgp[0] and i in imgp[1] and i in imgp[2]:
            imgpointsL_list.append(imgp[0][i].astype(np.float32))
            imgpointsM_list.append(imgp[1][i].astype(np.float32))
            imgpointsR_list.append(imgp[2][i].astype(np.float32))
            objpts_list.append(objp[i])
imgpoints_set_list=[imgpointsL_list,imgpointsM_list,imgpointsR_list]

fname = "%05d_%02d.png"%(0,0)
Limg = cv2.imread(os.path.join(all_folders[0],fname))
fname = "%05d_%02d.png"%(0,1)
Mimg = cv2.imread(os.path.join(all_folders[0],fname))
fname = "%05d_%02d.png"%(0,2)
Rimg = cv2.imread(os.path.join(all_folders[0],fname))

Images=[Limg,Mimg,Rimg]

grayL = cv.cvtColor(Limg, cv.COLOR_BGR2GRAY)
h,  w = Limg.shape[:2]

Lret, Lmtx, Ldist, Lrvecs, Ltvecs = cv.calibrateCamera(objpts_list, imgpointsL_list, (w,h), None, None)
Lnewcameramtx, Lroi = cv.getOptimalNewCameraMatrix(Lmtx, Ldist, (w,h), 1, (w,h))

Mret, Mmtx, Mdist, Mrvecs, Mtvecs = cv.calibrateCamera(objpts_list, imgpointsM_list, (w,h), None, None)
Mnewcameramtx, Mroi = cv.getOptimalNewCameraMatrix(Mmtx, Mdist, (w,h), 1, (w,h))

Rret, Rmtx, Rdist, Rrvecs, Rtvecs = cv.calibrateCamera(objpts_list, imgpointsR_list, (w,h), None, None)
Rnewcameramtx, Rroi = cv.getOptimalNewCameraMatrix(Rmtx, Rdist, (w,h), 1, (w,h))

calib_parameters['mono']={  0:[Lret, Lmtx, Ldist, Lrvecs, Ltvecs,Lnewcameramtx,Lroi],
                            1:[Mret, Mmtx, Mdist, Mrvecs, Mtvecs,Mnewcameramtx, Mroi],
                            2:[Rret, Rmtx, Rdist, Rrvecs, Rtvecs,Rnewcameramtx, Rroi],
                            }
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
colors=['r','g','b']
for c in range(3):
    rvecs = calib_parameters['mono'][c][3]
    tvecs = calib_parameters['mono'][c][4]
    for i in range(len(rvecs)):
        Rmat,_=cv.Rodrigues(rvecs[i])
        H=np.identity(4)
        H[0:3,0:3]=Rmat
        H[0:3,3]=tvecs[i].reshape(-1)
        campos=np.linalg.inv(H)[0:3,3]
        ax.plot(campos[0],campos[1],campos[2],colors[c]+'o')

    
ax.plot(objpts_list[0][:,0],objpts_list[0][:,1],objpts_list[0][:,2],'ko')
plt.show()
#%%
#Lundstimg = cv.undistort(Limg, Lmtx, Ldist, None, Lnewcameramtx)
#xc, yc, wc, hc = Lroi
#croppeddst = Lundstimg[yc:yc+hc, xc:xc+wc]
#
#L1 = Limg.copy()
#L2 = Lundstimg.copy()
#line_thickness = 2
#for x in np.linspace(0+10,h-10,20):
#    cv.line(L1, (0,int(x)), (w,int(x)), (0, 255, 0), thickness=line_thickness)
#    cv.line(L2, (0,int(x)), (w,int(x)), (255, 0, 0), thickness=line_thickness)
#    
#gg=np.hstack([L1,L2])
#plotcv([['gg',gg]])
#
#
## crop the image
#
#
##cv.destroyAllWindows()
#
## undistort
##mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)
##dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)
##x, y, w, h = roi
##dst = dst[y:y+h, x:x+w]
#
##reprojection error
#mean_error = 0
#for i in range(len(objpoints)):
#    imgpoints2, _ = cv.projectPoints(objpoints_list[i], Lrvecs[i], Ltvecs[i], Lmtx, Ldist)
#    error = cv.norm(imgpointsL_list[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
#    mean_error += error
#print( "total left error: {}".format(mean_error/len(objpoints)) )
#
#mean_error = 0
#for i in range(len(objpoints)):
#    imgpoints2, _ = cv.projectPoints(objpoints[i], Rrvecs[i], Rtvecs[i], Rmtx, Rdist)
#    error = cv.norm(imgpointsR[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
#    mean_error += error
#print( "total right error: {}".format(mean_error/len(objpoints)) )

#%% stereo calibrate
h,  w = Limg.shape[:2]
calib_parameters['stereo']={}
calib_parameters['stereo_rect']={}
for c1,c2 in [(0,2),(0,1)]:
    imgpt1 =  imgpoints_set_list[c1]
    imgpt2 =  imgpoints_set_list[c2]
    
    mtx1 = calib_parameters['mono'][c1][1]
    dist1 = calib_parameters['mono'][c1][2]
    
    mtx2 = calib_parameters['mono'][c2][1]
    dist2 = calib_parameters['mono'][c2][2]
    
    
    
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 3000, 0.000001)
    #,flags=cv.CALIB_FIX_INTRINSIC+cv.CALIB_USE_EXTRINSIC_GUESS 
    retval, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = cv.stereoCalibrate(objpts_list,imgpt1,imgpt2,mtx1,dist1,mtx2,dist2,(w,h),criteria=criteria)
    calib_parameters['stereo'][(c1,c2)] = [cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F]

    
    R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv.stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, (w,h), R, T)
    mapx1,mapy1=cv.initUndistortRectifyMap(	cameraMatrix1, distCoeffs1, R1, P1[0:3,0:3], (w,h), cv.CV_32FC2	) 
    mapx2,mapy2=cv.initUndistortRectifyMap(	cameraMatrix2, distCoeffs2, R2, P2[0:3,0:3], (w,h), cv.CV_32FC2	) 


    calib_parameters['stereo_rect'][(c1,c2)] = [R1, R2, P1, P2, Q, validPixROI1, validPixROI2,(mapx1,mapy1),(mapx2,mapy2)]
    
def plotRectifiedStereo(c1,c2,Images,calib_parameters):
    
    
    L1 = Images[c1].copy()
    R1 = Images[c2].copy()
    (Lmapx, Lmapy),(Rmapx, Rmapy)=calib_parameters['stereo_rect'][(c1,c2)][7:9]
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


ff='camera_saved_data/2022-10-08-16-45-44'
fname = "%05d_%02d.png"%(0,0)
Limg = cv2.imread(os.path.join(ff,fname))
fname = "%05d_%02d.png"%(0,1)
Mimg = cv2.imread(os.path.join(ff,fname))
fname = "%05d_%02d.png"%(0,2)
Rimg = cv2.imread(os.path.join(ff,fname))
Images=[Limg,Mimg,Rimg]
plotRectifiedStereo(0,2,Images,calib_parameters)  

with open("calibrated_camera_parameters.pkl","wb") as F:
    pkl.dump(calib_parameters,F)
#%%

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
with open("../calibrated_camera_parameters.pkl","rb") as F:
    calib_parameters=pkl.load(F)
    
ff='camera_saved_data/2022-10-08-16-45-44'
fname = "%05d_%02d.png"%(0,0)
Limg = cv2.imread(os.path.join(ff,fname))
fname = "%05d_%02d.png"%(0,1)
Mimg = cv2.imread(os.path.join(ff,fname))
fname = "%05d_%02d.png"%(0,2)
Rimg = cv2.imread(os.path.join(ff,fname))

Limg=cv2.undistort(	Limg, calib_parameters['mono'][0][1],calib_parameters['mono'][0][2], newCameraMatrix=calib_parameters['mono'][0][5]	)
Rimg=cv2.undistort(	Rimg, calib_parameters['mono'][2][1],calib_parameters['mono'][2][2], newCameraMatrix=calib_parameters['mono'][2][5]	)


sift = cv.SIFT_create()
h,  w = Limg.shape[:2]

#plotcv([['im',np.hstack([L1,R1])]])
# find the keypoints and descriptors with SIFT
kp1, des1 = sift.detectAndCompute(Limg, None)
kp2, des2 = sift.detectAndCompute(Rimg, None)

imgSift = cv.drawKeypoints(
    Limg, kp1, None, flags=cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
#plotcv([["SIFT Keypoints", imgSift]])

FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=2)
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



retval, H1, H2=cv.stereoRectifyUncalibrated(pts1,pts2,fundamental_matrix,(w,h))

#ff='camera_saved_data/2022-10-08-16-45-44'
#fname = "%05d_%02d.png"%(35,0)
#Limg = cv2.imread(os.path.join(ff,fname))
#fname = "%05d_%02d.png"%(35,1)
#Mimg = cv2.imread(os.path.join(ff,fname))
#fname = "%05d_%02d.png"%(35,2)
#Rimg = cv2.imread(os.path.join(ff,fname))

#Limg=cv2.undistort(	Limg, calib_parameters['mono'][0][1],calib_parameters['mono'][0][2], newCameraMatrix=calib_parameters['mono'][0][5]	)
#Rimg=cv2.undistort(	Rimg, calib_parameters['mono'][2][1],calib_parameters['mono'][2][2], newCameraMatrix=calib_parameters['mono'][2][5]	)


Ldst = cv.warpPerspective(Limg, H1, (w, h))
Rdst = cv.warpPerspective(Rimg, H2, (w, h))

line_thickness = 2
for x in np.linspace(0+10,h-10,30):
    cv.line(Ldst, (0,int(x)), (w,int(x)), (0, 255, 0), thickness=line_thickness)
    cv.line(Rdst, (0,int(x)), (w,int(x)), (255, 0, 0), thickness=line_thickness)
    
    cv.line(Limg, (0,int(x)), (w,int(x)), (0, 255, 0), thickness=line_thickness)
    cv.line(Rimg, (0,int(x)), (w,int(x)), (255, 0, 0), thickness=line_thickness)
 
im1=np.hstack([Limg,Rimg])
im2=np.hstack([Ldst,Rdst])
plotcv([['im1',im1],['im2',im2]])
