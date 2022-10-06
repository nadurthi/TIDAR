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

import pathlib
pathfile=pathlib.Path(__file__).parent.resolve()

savefolder = pathfile/pathlib.Path('camera_saved_data')

saveset = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
os.makedirs(os.path.join(savefolder,saveset))
Nmax=1000

saveonclick=True
saveaspng=True


def imagesaver(q,exitflg):
    i=0
    X=[]
    while True:
        try:
            img = q.get(timeout=0.05)
        except queue.Empty:
            img = None
        if img is not None:
            print(i,"  saving this images")
            if saveaspng:
                for j in range(len(img)):
                    ff='%05d_%02d.png'%(i,j)
                    cv2.imwrite(os.path.join(savefolder,saveset,ff),img[j])
            else:
                X.append(img)
        #        cv2.imwrite(ff,img)
                if i == Nmax:
                    ff=datetime.now().strftime("%Y-%m-%d-%H-%M-%S")+'.pkl'
                    with open(os.path.join(savefolder,saveset,ff),'wb') as F:
                        pkl.dump(X,F)
                    i=0
                    X=[]
            i+=1
            q.task_done()
            
        if q.empty() and exitflg.is_set():
            break
    if not saveaspng and len(X)>0:
        ff=datetime.now().strftime("%Y-%m-%d-%H-%M-%S")+'.pkl'
        with open(os.path.join(savefolder,saveset,ff),'wb') as F:
            pkl.dump(X,F)
            
if __name__=="__main__":
    exitflg = threading.Event()
    exitflg.clear()
    q = queue.Queue()
    x = threading.Thread(target=imagesaver, args=(q,exitflg))
    x.start()
    
    lastkey = 0
#    cv2.namedWindow('Window', cv2.WINDOW_NORMAL)  # Create an OpenCV output window
    
    mTis = TIS.MultiTis(str(pathfile/pathlib.Path("cameras.json")))
    mTis.start_cameras()
    
    cv2windows=[]
    for i in range(len(mTis)):
        cv2windows.append(cv2.namedWindow('Camera_%d'%i, cv2.WINDOW_NORMAL))
        
  
    
    
    while 1:
        images = mTis.snapImages()
            
        if images is None:
            continue
        
        if saveonclick is False:
            q.put(images)
        
#        print(images[0].shape,images[1].shape)
        h,  w = images[0].shape[:2]
        
        for i in range(len(mTis)):
            cv2.imshow('Camera_%d'%i, images[i])

#        cv2.imshow('Window', np.hstack(images))  # Display the result
    
        lastkey = cv2.waitKey(10)
        if lastkey == 27 or lastkey == 113:
            break
        if saveonclick and lastkey == 115:
            q.put(images)
            print("saved: ",datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))
    
    exitflg.set()
    x.join()
    # Stop the pipeline and clean up
    mTis.stop_cameras()
    
    cv2.destroyAllWindows()
    print('Program ends')
