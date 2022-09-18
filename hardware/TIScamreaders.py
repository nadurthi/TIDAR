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

# This sample shows, how to get an image and convert it to OpenCV
# needed packages:
# pyhton-opencv
# pyhton-gst-1.0
# tiscamera


class MultiTis:
    def __init__(self,configfile):
        with open(configfile) as jsonFile:
            self.cameraconfigs = json.load(jsonFile)
            jsonFile.close()
        self.TisCams = []
        
    def stop_cameras(self):
        for i in range(len(self.TisCams)):
            self.TisCams[i].Stop_pipeline()
    
    def start_cameras(self):

        for cameraconfig in self.cameraconfigs['cameras']:
            camera = TIS.TIS()
            
            camera.openDevice(cameraconfig['serial'],
                              cameraconfig['width'],
                              cameraconfig['height'],
                              cameraconfig['framerate'],
                              TIS.SinkFormats.fromString(cameraconfig['pixelformat']),
                              False)
        
            self.TisCams.append(camera)
    
        for i in range(len(self.TisCams)):
            properties = self.cameraconfigs['cameras'][i]['properties']
            try:
                self.TisCams[i].Set_Property(properties['property'],properties['value'])
            except Exception as error:
                print(error)
            
            self.TisCams[i].Set_Property("TriggerMode","Off")
        
        for i in range(len(self.TisCams)):
            self.TisCams[i].Start_pipeline() 
    
    def __len__(self):
        return len(self.TisCams)
    
    def snapImages(self):
        tries=0
        images=None
        while(1):
            flg = True
            for i in range(len(self.TisCams)):
                flg = flg and (self.TisCams[i].sample is not None and self.TisCams[i].newsample)
    #            flg=flg and TisCams[i].Snap_image(1)
            if flg is True:
                for i in range(len(self.TisCams)):
                    self.TisCams[i].brute_force_convert_numpy()
                
                images=[]
                for i in range(len(self.TisCams)):
                    images.append(self.TisCams[i].Get_image())  # Get the image. It is a numpy array
            
                break
            time.sleep(0.01)
            tries+=1
            if tries > 100:
                for i in range(len(self.TisCams)):
                    self.TisCams[i].Stop_pipeline()
                cv2.destroyAllWindows()
                raise Exception("Failed to read cams after tries = ", tries)
            

                
            
        return images
                    
lastkey = 0
cv2.namedWindow('Window', cv2.WINDOW_NORMAL)  # Create an OpenCV output window

mTis = MultiTis("cameras.json")
mTis.start_cameras()


showBlurryMeasure=0
imid=0
blurmeasures=[]
for i in range(len(mTis)):
    blurmeasures.append([0,0])

while 1:
    images = mTis.snapImages()
        
    if images is None:
        continue
    
    print(images[0].shape,images[1].shape)
    h,  w = images[0].shape[:2]
    
    if showBlurryMeasure:        
        fm=cv2.Laplacian(images[imid], cv2.CV_64F).var()
#        fftblurr,flg = detect_blur_fft(images[imid], size=60, threshold=10)
        blurmeasures[imid][0]=fm
#        blurmeasures[imid][1]=fftblurr
        print(blurmeasures)
        imid+=1
        if imid==3:
            imid=0
        for i in range(len(mTis)):    
            cv2.putText(images[i], "lap blurry: {:.2f}".format(blurmeasures[i][0]), ( int(w/2),int(h/2) ),
        	cv2.FONT_HERSHEY_SIMPLEX, 5, (0, 0, 255), 7)
                
            cv2.putText(images[i], "fft blurry: {:.2f}".format(blurmeasures[i][1]), ( int(w/2),int(h/2)+500 ),
        	cv2.FONT_HERSHEY_SIMPLEX, 5, (0, 255, 0), 7)
    
    cv2.imshow('Window', np.hstack(images))  # Display the result

    lastkey = cv2.waitKey(10)
    if lastkey == 27 or lastkey == 113:
        break
    
# Stop the pipeline and clean up
mTis.stop_cameras()

cv2.destroyAllWindows()
print('Program ends')
