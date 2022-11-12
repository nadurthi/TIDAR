import sys
import cv2
import numpy as np
import scipy.fft
import json
import time
import matplotlib.pyplot as plt
import TIS
import threading
import queue
import os
from datetime import datetime
from blobdetector_setup import blobDetector
import object_points
import pickle as pkl
import _pickle as cPickle
import marshal
import pathlib
import argparse
import sys
import calib_codes

pathfile=pathlib.Path(__file__).parent.resolve()

savefolder = pathfile/pathlib.Path('camera_saved_data')

saveset = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
os.makedirs(os.path.join(savefolder,saveset))
Nmax=10

saveonclick=True
saveaspng=True
saveasavi=False

def imagesaver(q,exitflg,size):
    i=0
    if saveaspng:
        X=np.zeros((10,size[0],3*size[1],size[2]),dtype=np.uint8)
    if saveasavi:
        ff=datetime.now().strftime("%Y-%m-%d-%H-%M-%S.avi")
        out = cv2.VideoWriter(os.path.join(savefolder,saveset,ff),cv2.VideoWriter_fourcc('M','J','P','G'), 15, size)
    
    cnt=0
    imgpoints = [{}, {}, {}]  # 2d points in image plane.

    while True:
        try:
            img,CC = q.get(timeout=0.05)
        except queue.Empty:
            img = None
        if img is not None:
            print(cnt,"  saving this images")
            cnt+=1
            if saveasavi:
                out.write(img[0])
                
            elif saveaspng:
                for j in range(len(img)):
                    ff='%05d_%02d.png'%(i,j)
                    imgpoints[j][i] = CC[j]
                    cv2.imwrite(os.path.join(savefolder,saveset,ff),img[j])
            else:
#                X.append(img)
                X[i]=np.hstack(img)
        #        cv2.imwrite(ff,img)
                if i == Nmax-1:
                    ff=datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
#                    with open(os.path.join(savefolder,saveset,ff),'wb') as F:
#                        pkl.dump(X,F)
                    np.savez(os.path.join(savefolder,saveset,ff), X=X)
                    i=-1
            try:
                objp = object_points.objp_checker
                ho,wo=img[0].shape[:2]
                calib_parameters = calib_codes.calibrate_Cameras(img[0].copy(), ho, wo, imgpoints, objp,scales=[3])
                print("L-R : ",calib_parameters[3]['stereo'][(0,2)][5])
                print("L-M : ", calib_parameters[3]['stereo'][(0, 1)][5])
            except:
                print("failed calib")


            i+=1
            q.task_done()
            
        if q.empty() and exitflg.is_set():
            break

    with open(os.path.join(savefolder, saveset, 'image_points.pkl'), 'wb') as F:
        pkl.dump(imgpoints,F)

    if saveasavi:
        print("releasing out")
        out.release()
    print("saving remainder")
    if not saveaspng and len(X)>0:
        ff=datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
#        with open(os.path.join(savefolder,saveset,ff),'wb') as F:
#            pkl.dump(X,F)
        np.savez_compressed(os.path.join(savefolder,saveset,ff), X=X)
            
if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Saver and calibrator')
    parser.add_argument("--saveset" )
    parser.add_argument("--calib", action="store_true")
    parser.add_argument("--acq", action="store_true")
    args = parser.parse_args()
    print(args.saveset,args.calib,args.acq)



    if args.acq:

        exitflg = threading.Event()
        exitflg.clear()
        q = queue.Queue()


        lastkey = 0
    #    cv2.namedWindow('Window', cv2.WINDOW_NORMAL)  # Create an OpenCV output window

        mTis = TIS.MultiTis("calibfiles/cameras.json")
        mTis.start_cameras()

        while 1:
            images = mTis.snapImages()
            if images is not None:
                break

        size=images[0].shape
        print(size)
        print(images[0].dtype)

        x = threading.Thread(target=imagesaver, args=(q,exitflg,size))
        x.start()

        cv2windows=[]
        for i in range(len(mTis)):
            cv2windows.append(cv2.namedWindow('Camera_%d'%i, cv2.WINDOW_NORMAL))

        prev_img_points=None
        t0 = time.time()
        while 1:
    #        time.sleep(1)
            images = mTis.snapImages()
    #        for i in range(3):
    #            images[i]=cv2.cvtColor(images[i], cv2.COLOR_BGRA2BGR)
            h,  w = images[0].shape[:2]

            if images is None:
                continue
            # gray_imgs= [cv2.cvtColor(Limg, cv2.COLOR_BGR2GRAY) for Limg in images]
            gray_cropped=[]
            RET=[]
            CC={0:[],1:[],2:[],3:[]}
            for jj in range(len(mTis)):
                Limg = images[jj].copy()
                grayimg = cv2.cvtColor(images[jj], cv2.COLOR_BGR2GRAY)  #images[jj].copy()
                # ret, cor = cv2.findChessboardCornersSB(gray_imgs[jj], (4,5), cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_NORMALIZE_IMAGE)
                # ret, cor = cv2.findCirclesGrid(images[jj], (4,11), cv2.CALIB_CB_ASYMMETRIC_GRID ,blobDetector,None   )
                ret, cor = calib_codes.getchesspattern(grayimg)


                if ret:
                    cv2.drawChessboardCorners(Limg, (4,5), cor, ret)
                    # cv2.drawChessboardCorners(Limg, (4,11), cor, ret)
                    CC[jj]=cor
                    corners2L=np.vstack(cor)
                    mn=np.min(corners2L,axis=0)
                    mx=np.max(corners2L,axis=0)
                    mn=mn.astype(int)
                    mx=mx.astype(int)
                    RET.append(True)
                    gray_cropped.append( Limg[max((mn[1]-100),0):min((mx[1]+100),h),max((mn[0]-100),0):min((mx[0]+100),w)])
                else:
                    gray_cropped.append(Limg)
                    RET.append(False)
    #        if saveonclick is False:
    #            q.put(images)

    #        print(images[0].shape,images[1].shape)


            for i in range(len(mTis)):
                cv2.imshow('Camera_%d'%i, gray_cropped[i])

    #        cv2.imshow('Window', np.hstack(images))  # Display the result
            tf=time.time()

            lastkey = cv2.waitKey(10)
            if lastkey == 27 or lastkey == 113:
                break
            if saveonclick and lastkey == 115:
                q.put((images,CC))
                print("saved: ",datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))
            # if np.all(RET) and (prev_img_points is None or np.max(np.linalg.norm(prev_img_points-CC[0],axis=1))>100) and (tf-t0)>2:
            #     prev_img_points = CC[0]
            #     t0=time.time()
            #     q.put((images, CC))
            #     print("saved: ", datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))

        mTis.stop_cameras()

        cv2.destroyAllWindows()

        exitflg.set()
        x.join()
        # Stop the pipeline and clean up

        print('Program ends')


    if args.calib:
        if args.saveset is not None:
            saveset = args.saveset
        ##----------------------------------------ESTIMATE CALIBRATION PARAMETERS-----------------------------------------
        imgpoints = [{}, {}, {}]  # 2d points in image plane.
        if os.path.isfile(os.path.join(savefolder, saveset, 'image_points.pkl')) is False:
            g=[a for a in os.listdir(os.path.join(savefolder, saveset)) if '.png' in a]
            for si in range(int(len(g)/3)):
                for cj in range(3):
                    ff = '%05d_%02d.png' % (si, cj)
                    Limg_o = cv2.imread(os.path.join(savefolder, saveset, ff))
                    # ret, cor = cv2.findCirclesGrid(Limg_o, (4, 11), cv2.CALIB_CB_ASYMMETRIC_GRID, blobDetector, None)
                    grayimg = cv2.cvtColor(Limg_o, cv2.COLOR_BGR2GRAY)
                    ret,cor=calib_codes.getchesspattern(grayimg)
                    if ret:
                        imgpoints[cj][si]=cor
        else:
            with open(os.path.join(savefolder, saveset, 'image_points.pkl'), 'rb') as F:
                imgpoints = pkl.load(F)

        ff = '%05d_%02d.png' % (0, 0)
        Limg_o = cv2.imread(os.path.join(savefolder, saveset, ff))
        ho, wo = Limg_o.shape[:2]



        objp = object_points.objp_checker
        calib_parameters=calib_codes.calibrate_Cameras(Limg_o, ho, wo, imgpoints, objp)

        # plt.show()

        with open(os.path.join(savefolder, saveset, 'calibrated_camera_parameters.pkl'), 'wb') as F:
            pkl.dump(calib_parameters,F)


        # with open(os.path.join(savefolder, saveset, 'calibrated_camera_parameters.json'), 'wb') as F:
        #     json.dump(calib_parameters,F, indent=4)