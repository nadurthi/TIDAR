import sys
import cv2
import numpy as np
import scipy.fft
import json
import time
from . import TIS
import threading
import queue
import os
from datetime import datetime
import pickle as pkl

import pathlib
pathfile=pathlib.Path(__file__).parent.resolve()

savefolder = pathfile/pathlib.Path('camera_calibration_data')

saveset = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
os.makedirs(os.path.join(savefolder,saveset,"cam0"))
os.makedirs(os.path.join(savefolder,saveset,"cam1"))
os.makedirs(os.path.join(savefolder,saveset,"cam2"))

def getImages():
    lastkey = 0
    cv2.namedWindow('Window0', cv2.WINDOW_NORMAL)  # Create an OpenCV output window
    cv2.namedWindow('Window1', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Window2', cv2.WINDOW_NORMAL)
    
    mTis = TIS.MultiTis(str(pathfile/pathlib.Path("cameras.json")))
    mTis.start_cameras()
    
    
    i=0
    while 1:
        images = mTis.snapImages()
            
        if images is None:
            continue
        

        h,  w = images[0].shape[:2]
        
    
        cv2.imshow('Window0', images[0])  # Display the result
        cv2.imshow('Window1', images[1])  # Display the result
        cv2.imshow('Window2', images[2])  # Display the result
        
        lastkey = cv2.waitKey(10)
        if lastkey == 27 or lastkey == 113:
            break
        
        if lastkey == 115:
           for c,img in enumerate(images):
                ff="%06d.png"%i
                cv2.imwrite(os.path.join(savefolder,saveset,"cam%d"%c,ff),img)
           i+=1
           
            
    
    mTis.stop_cameras()
    
def getImagePoints(folder):
    
    
if __name__=="__main__":
    
    getImages()
    
    # getting image points
    getImagePoints(os.path.join(savefolder,saveset))
    
    cv2.destroyAllWindows()
    
