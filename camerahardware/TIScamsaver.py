import sys
import cv2
import numpy as np
import scipy.fft
import json
import time
import TIS
import threading
import queue
import os
from datetime import datetime
import pickle as pkl
import _pickle as cPickle
import marshal
import pathlib
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

    while True:
        try:
            img = q.get(timeout=0.05)
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

            i+=1
            q.task_done()
            
        if q.empty() and exitflg.is_set():
            break
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
    exitflg = threading.Event()
    exitflg.clear()
    q = queue.Queue()
    
    
    lastkey = 0
#    cv2.namedWindow('Window', cv2.WINDOW_NORMAL)  # Create an OpenCV output window
    
    mTis = TIS.MultiTis(str(pathfile/pathlib.Path("cameras.json")))
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
        
  

    while 1:
#        time.sleep(1)
        images = mTis.snapImages()
#        for i in range(3):
#            images[i]=cv2.cvtColor(images[i], cv2.COLOR_BGRA2BGR)   
        h,  w = images[0].shape[:2]
        
        if images is None:
            continue
        gray_imgs= [cv2.cvtColor(Limg, cv2.COLOR_BGR2GRAY) for Limg in images]
        gray_cropped=[]
        cornersL=[]
        
        for jj in range(len(mTis)):
            ret, cor = cv2.findChessboardCornersSB(gray_imgs[jj], (4,5), cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_NORMALIZE_IMAGE)
            if ret:
                cv2.drawChessboardCorners(gray_imgs[jj], (4,5), cor, ret)
                corners2L=np.vstack(cor)
                mn=np.min(corners2L,axis=0)
                mx=np.max(corners2L,axis=0)
                mn=mn.astype(int)
                mx=mx.astype(int)
                Limg=gray_imgs[jj]
                gray_cropped.append( Limg[max((mn[1]-100),0):min((mx[1]+100),h),max((mn[0]-100),0):min((mx[0]+100),w)])  
            else:
                gray_cropped.append(gray_imgs[jj])
#        if saveonclick is False:
#            q.put(images)
        
#        print(images[0].shape,images[1].shape)
        
        
        for i in range(len(mTis)):
            cv2.imshow('Camera_%d'%i, gray_cropped[i])

#        cv2.imshow('Window', np.hstack(images))  # Display the result
    
        lastkey = cv2.waitKey(10)
        if lastkey == 27 or lastkey == 113:
            break
        if saveonclick and lastkey == 115:
            q.put(images)
            print("saved: ",datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))
    
    mTis.stop_cameras()
    
    cv2.destroyAllWindows()
    
    exitflg.set()
    x.join()
    # Stop the pipeline and clean up
    
    print('Program ends')
