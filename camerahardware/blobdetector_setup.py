import cv2
import os
import numpy as np

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

########################################Blob Detector##############################################

# Setup SimpleBlobDetector parameters.
blobParams = cv2.SimpleBlobDetector_Params()

# Change thresholds
blobParams.minThreshold = 100
blobParams.maxThreshold = 255

# Filter by Area.
blobParams.filterByArea = True
blobParams.minArea = 50     # minArea may be adjusted to suit for your experiment
blobParams.maxArea = 10500   # maxArea may be adjusted to suit for your experiment

# Filter by Circularity
blobParams.filterByCircularity = True
blobParams.minCircularity = 0.7

# Filter by Convexity
blobParams.filterByConvexity = True
blobParams.minConvexity = 0.87

# Filter by Inertia
blobParams.filterByInertia = True
blobParams.minInertiaRatio = 0.01

# Create a detector with the parameters
blobDetector = cv2.SimpleBlobDetector_create(blobParams)


# # Draw detected blobs as red circles. This helps cv2.findCirclesGrid() .
# im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
# im_with_keypoints_gray = cv2.cvtColor(im_with_keypoints, cv2.COLOR_BGR2GRAY)
# ret, corners = cv2.findCirclesGrid(im_with_keypoints, (4,11), None, flags = cv2.CALIB_CB_ASYMMETRIC_GRID)   # Find the circle grid
#

if __name__=="__main__":
    cv2.namedWindow('Camera', cv2.WINDOW_NORMAL)
    folder = 'camerahardware/camera_saved_data/2022-10-12-17-41-40'
    si=0
    cj=0
    fname = "%05d_%02d.png"%(si,cj)
    Limg = cv2.imread(os.path.join(folder,fname))

    ret, cor = cv2.findCirclesGrid(Limg, (4,11), cv2.CALIB_CB_ASYMMETRIC_GRID ,blobDetector,None   )
    if ret:
        cv2.drawChessboardCorners(Limg, (4,11), cor, ret)

    # gray = cv2.cvtColor(Limg, cv2.COLOR_BGR2GRAY)
    # keypoints = blobDetector.detect(gray) # Detect blobs.
    #
    # Limg_with_keypoints=Limg.copy()
    # Limg_with_keypoints = cv2.drawKeypoints(Limg, keypoints, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    while 1:
        cv2.imshow('Camera', Limg)
        lastkey = cv2.waitKey(10)
        if lastkey == 27 or lastkey == 113:
            break
    cv2.destroyAllWindows()