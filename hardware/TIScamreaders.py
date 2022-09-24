import sys
import cv2
import numpy as np
import scipy.fft
import json
import time
import TIS


def detect_blur_fft(image, size=60, threshold=10):
    grayL = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    (h, w) = grayL.shape
    (cx, cy) = (int(w / 2.0), int(h / 2.0))
    fft = np.fft.fft2(image)
    fftShift = np.fft.fftshift(fft)

    
    fftShift[cy - size : cy + size, cx - size : cx + size] = 0
    fftShift = np.fft.ifftshift(fftShift)
    recon = np.fft.ifft2(fftShift)

    magnitude = 20 * np.log(np.abs(recon))
    mean = np.mean(magnitude)

    return (mean, mean <= threshold)

def getblurrymeasures(image):
    fm=cv2.Laplacian(image, cv2.CV_64F).var()
    fftblurr,flg = detect_blur_fft(image, size=60, threshold=10)

    

    cv2.putText(image, "lap blurry: {:.2f}".format(fm), ( int(w/2),int(h/2) ),
	cv2.FONT_HERSHEY_SIMPLEX, 5, (0, 0, 255), 7)
        
    cv2.putText(image, "fft blurry: {:.2f}".format(fftblurr), ( int(w/2),int(h/2)+500 ),
	cv2.FONT_HERSHEY_SIMPLEX, 5, (0, 255, 0), 7)

    return fm,fftblurr
# This sample shows, how to get an image and convert it to OpenCV
# needed packages:
# pyhton-opencv
# pyhton-gst-1.0
# tiscamera
             
lastkey = 0
cv2.namedWindow('Window', cv2.WINDOW_NORMAL)  # Create an OpenCV output window

mTis = TIS.MultiTis("cameras.json")
mTis.start_cameras()


showBlurryMeasure=0

while 1:
    images = mTis.snapImages()
        
    if images is None:
        continue
    
    print(images[0].shape,images[1].shape)
    h,  w = images[0].shape[:2]
    
    if showBlurryMeasure:        
        for i in range(len(mTis)): 
            fm,fft = getblurrymeasures(images[i])
    
    cv2.imshow('Window', np.hstack(images))  # Display the result

    lastkey = cv2.waitKey(10)
    if lastkey == 27 or lastkey == 113:
        break
    
# Stop the pipeline and clean up
mTis.stop_cameras()

cv2.destroyAllWindows()
print('Program ends')
