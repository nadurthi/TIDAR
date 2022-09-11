import sys
import cv2
import numpy as np
sys.path.append("../python-common")
import json

import TIS

# This sample shows, how to get an image and convert it to OpenCV
# needed packages:
# pyhton-opencv
# pyhton-gst-1.0
# tiscamera
with open("cameras.json") as jsonFile:
    cameraconfigs = json.load(jsonFile)
    jsonFile.close()


Tis = []
for cameraconfig in cameraconfigs['cameras']:
    camera = TIS.TIS()
    
    camera.openDevice(cameraconfig['serial'],
                      cameraconfig['width'],
                      cameraconfig['height'],
                      cameraconfig['framerate'],
                      TIS.SinkFormats.fromString(cameraconfig['pixelformat']),
                      False)

    Tis.append(camera)
    
for i in range(len(Tis)):
    properties = cameraconfigs['cameras'][i]['properties']
    try:
        Tis[i].Set_Property(properties['property'],properties['value'])
    except Exception as error:
        print(error)
    
    Tis[i].Set_Property("TriggerMode","Off")
    
#Tis.openDevice("49124129", 3072, 2048, "30/1", TIS.SinkFormats.BGRA,True)
# the camera with serial number 10710286 uses a 640x480 video format at 30 fps and the image is converted to
# RGBx, which is similar to RGB32.

# The next line is for selecting a device, video format and frame rate.
#if not Tis.selectDevice():
#    quit(0)

# Just in case trigger mode is enabled, disable it.
#try:
#    Tis.Set_Property("TriggerMode","Off")
#except Exception as error:
#    print(error)

for i in range(len(Tis)):
    Tis[i].Start_pipeline()  # Start the pipeline so the camera streams

print('Press Esc to stop')
lastkey = 0

cv2.namedWindow('Window', cv2.WINDOW_NORMAL)  # Create an OpenCV output window

kernel = np.ones((5, 5), np.uint8)  # Create a Kernel for OpenCV erode function

while lastkey != 27:
    if Tis[0].Snap_image(0.1) is True and Tis[1].Snap_image(0.1) is True:  # Snap an image with one second timeout
        image0 = Tis[0].Get_image()  # Get the image. It is a numpy array
        image1 = Tis[1].Get_image()
        #image = cv2.erode(image, kernel, iterations=5)  # Example OpenCV image processing
        cv2.imshow('Window', np.hstack([image0,image1]))  # Display the result

    lastkey = cv2.waitKey(10)

# Stop the pipeline and clean up
for i in range(len(Tis)):
    Tis[i].Stop_pipeline()
cv2.destroyAllWindows()
print('Program ends')
