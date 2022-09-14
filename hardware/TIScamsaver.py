import sys
import cv2
import numpy as np
import os
import json
import time
import TIS

# This sample shows, how to get an image and convert it to OpenCV
# needed packages:
# pyhton-opencv
# pyhton-gst-1.0
# tiscamera

folder = "calibration_images"
os.makedirs(folder,exist_ok=True)

with open("cameras.json") as jsonFile:
    cameraconfigs = json.load(jsonFile)
    jsonFile.close()


TisCams = []
for cameraconfig in cameraconfigs['cameras']:
    camera = TIS.TIS()
    
    camera.openDevice(cameraconfig['serial'],
                      cameraconfig['width'],
                      cameraconfig['height'],
                      cameraconfig['framerate'],
                      TIS.SinkFormats.fromString(cameraconfig['pixelformat']),
                      False)

    TisCams.append(camera)
    
for i in range(len(TisCams)):
    properties = cameraconfigs['cameras'][i]['properties']
    try:
        TisCams[i].Set_Property(properties['property'],properties['value'])
    except Exception as error:
        print(error)
    
    TisCams[i].Set_Property("TriggerMode","Off")
    
#TisCams.openDevice("49124129", 3072, 2048, "30/1", TIS.SinkFormats.BGRA,True)
# the camera with serial number 10710286 uses a 640x480 video format at 30 fps and the image is converted to
# RGBx, which is similar to RGB32.

# The next line is for selecting a device, video format and frame rate.
#if not TisCams.selectDevice():
#    quit(0)

# Just in case trigger mode is enabled, disable it.
#try:
#    TisCams.Set_Property("TriggerMode","Off")
#except Exception as error:
#    print(error)

for i in range(len(TisCams)):
    TisCams[i].Start_pipeline()  # Start the pipeline so the camera streams

print('Press Esc to stop')
lastkey = 0

cv2.namedWindow('Window', cv2.WINDOW_NORMAL)  # Create an OpenCV output window

kernel = np.ones((5, 5), np.uint8)  # Create a Kernel for OpenCV erode function

savecalib=True
cnt=0
while lastkey != 27:
    tries=0
    while(1):
        flg = True
        for i in range(len(TisCams)):
            flg = flg and (TisCams[i].sample is not None and TisCams[i].newsample)
#            flg=flg and TisCams[i].Snap_image(1)
        if flg is True:
            for i in range(len(TisCams)):
                TisCams[i].brute_force_convert_numpy()
            break
        time.sleep(0.01)
        tries+=1
        if tries > 10:
            for i in range(len(TisCams)):
                TisCams[i].Stop_pipeline()
            cv2.destroyAllWindows()
            raise Exception("Failed to read cams after tries = ", tries)
        
    if flg:
#    if TisCams[0].Snap_image(0.1) is True and TisCams[1].Snap_image(0.1) is True:  # Snap an image with one second timeout
        images=[]
        for i in range(len(TisCams)):
            images.append(TisCams[i].Get_image())  # Get the image. It is a numpy array
#        image1 = TisCams[1].Get_image()
        #image = cv2.erode(image, kernel, iterations=5)  # Example OpenCV image processing
        print(images[0].shape,images[1].shape)
        comb_img = np.hstack(images)
        cv2.imshow('Window', comb_img)  # Display the result
        
        if savecalib:
            cv2.imwrite(os.path.join(folder,'image_%d.png'%cnt), comb_img)
            cnt+=1
    lastkey = cv2.waitKey(10)

# Stop the pipeline and clean up
for i in range(len(TisCams)):
    TisCams[i].Stop_pipeline()
cv2.destroyAllWindows()
print('Program ends')
