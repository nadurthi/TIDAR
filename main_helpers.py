import numpy as np
import json
import pickle as pkl
import os
import open3d as o3d
import cv2

def add_rect_lines(imgs):
    line_thickness = 2
    for img in imgs:
        h, w = img.shape[:2]
        for x in np.linspace(0+10,h-10,30):
            cv2.line(img, (0,int(x)), (w,int(x)), (0, 255, 0), thickness=line_thickness)



def normalizeImg(low, high, img):
    imgClip = np.clip(img, low, high)
    maxVal = np.max(imgClip)
    minVal = np.min(imgClip)
    return np.uint8((255.)/(maxVal-minVal)*(imgClip-maxVal)+255.)


def plotcv(imglist):
    for i in range(len(imglist)):
        if len(imglist[i]) == 2:
            name, img = imglist[i]
            func = None
        else:
            name, img, func, data = imglist[i]

        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.imshow(name, img)
        if func is not None:
            params = {}
            params['name'] = name
            params['img'] = img
            params['depths'] = data['depths']
            cv2.setMouseCallback(name, func, params)

    while (1):
        k = cv2.waitKey(33)
        if k == 27 or k == 113:  # Esc key to stop
            break
    cv2.destroyAllWindows()



def get_points(Limg, disp, Q):
    points = cv2.reprojectImageTo3D(disp, Q)
    points[points > 100] = np.inf

    colors = cv2.cvtColor(Limg, cv2.COLOR_BGR2RGB)
    mask = disp > disp.min()
    out_points = points[mask]
    out_colors = colors[mask]

    ind = np.linalg.norm(out_points, axis=1) <= 100

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(out_points[ind])
    pcd.colors = o3d.utility.Vector3dVector(out_colors[ind] / 255)
    return pcd


def click_event_depth(event, x, y, flags, params):
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:
        # displaying the coordinates
        # on the Shell
        print(x, ' ', y, "depth = ", params['depths'][y, x])

        # displaying the coordinates
        # on the image window
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(params['img'], str(params['depths'][y, x]), (x, y), font,
                    1, (0, 0, 255), 2)
        cv2.imshow(params['name'], params['img'])

    # checking for right mouse clicks
    if event == cv2.EVENT_RBUTTONDOWN:
        # displaying the coordinates
        # on the Shell
        print(x, ' ', y)

        # displaying the coordinates
        # on the image window
        font = cv2.FONT_HERSHEY_SIMPLEX
        b = params['img'][y, x, 0]
        g = params['img'][y, x, 1]
        r = params['img'][y, x, 2]
        cv2.putText(params['img'], str(b) + ',' +
                    str(g) + ',' + str(r),
                    (x, y), font, 1,
                    (0, 255, 255), 2)
        cv2.imshow(params['name'], params['img'])


def getUncalibRect(Limg, Rimg):
    sift = cv2.SIFT_create()
    h, w = Limg.shape[:2]

    # plotcv([['im',np.hstack([L1,R1])]])
    # find the keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(Limg, None)
    kp2, des2 = sift.detectAndCompute(Rimg, None)

    #    imgSift = cv2.drawKeypoints(
    #        Limg, kp1, None, flags=cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # plotcv([["SIFT Keypoints", imgSift]])

    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=2)
    search_params = dict(checks=50)  # or pass empty dictionary
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    matches = flann.knnMatch(des1, des2, k=2)

    # Keep good matches: calculate distinctive image features
    # Lowe, D.G. Distinctive Image Features from Scale-Invariant Keypoints. International Journal of Computer Vision 60, 91–110 (2004). https://doi.org/10.1023/B:VISI.0000029664.99615.94
    # https://www.cs.ubc.ca/~lowe/papers/ijcv04.pdf
    matchesMask = [[0, 0] for i in range(len(matches))]
    good = []
    pts1 = []
    pts2 = []

    for i, (m, n) in enumerate(matches):
        if m.distance < 0.7 * n.distance:
            # Keep this keypoint pair
            matchesMask[i] = [1, 0]
            good.append(m)
            pts2.append(kp2[m.trainIdx].pt)
            pts1.append(kp1[m.queryIdx].pt)

    pts1 = np.int32(pts1)
    pts2 = np.int32(pts2)
    fundamental_matrix, inliers = cv2.findFundamentalMat(pts1, pts2, cv2.FM_RANSAC)

    # We select only inlier points
    pts1 = pts1[inliers.ravel() == 1]
    pts2 = pts2[inliers.ravel() == 1]

    retval, H1, H2 = cv2.stereoRectifyUncalibrated(pts1, pts2, fundamental_matrix, (w, h))

    # ff='camera_saved_data/2022-10-08-16-45-44'
    # fname = "%05d_%02d.png"%(35,0)
    # Limg = cv2.imread(os.path.join(ff,fname))
    # fname = "%05d_%02d.png"%(35,1)
    # Mimg = cv2.imread(os.path.join(ff,fname))
    # fname = "%05d_%02d.png"%(35,2)
    # Rimg = cv2.imread(os.path.join(ff,fname))

    # Limg=cv2.undistort(	Limg, calib_parameters['mono'][0][1],calib_parameters['mono'][0][2], newCameraMatrix=calib_parameters['mono'][0][5]	)
    # Rimg=cv2.undistort(	Rimg, calib_parameters['mono'][2][1],calib_parameters['mono'][2][2], newCameraMatrix=calib_parameters['mono'][2][5]	)

    Ldst = cv2.warpPerspective(Limg, H1, (w, h))
    Rdst = cv2.warpPerspective(Rimg, H2, (w, h))

    return Ldst, Rdst

def get_colored_depth_fromdepth(Ldst,depth,dmax=100):
    points = cv2.reprojectImageTo3D(disp.astype(np.int16), Q)
    depths = np.linalg.norm(points, axis=2)
    depths[depths > dmax] = np.inf
    scaled_depths = (depths * 10).round().astype(np.uint8)
    rangeToUse = [0, 255]  # from 20-30° celsius
    normed_range = normalizeImg(rangeToUse[0], rangeToUse[1], scaled_depths)
    depthcolor = cv2.applyColorMap(normed_range, 5)

def get_colored_depth(Ldst,disp,Q,dmax=100,returnpcd=False):
    points = cv2.reprojectImageTo3D(disp.astype(np.int16), Q)
    depths = np.linalg.norm(points, axis=2)
    depths[depths > dmax] = np.inf
    scaled_depths = (depths * 10).round().astype(np.uint8)
    rangeToUse = [0, 255]  # from 20-30° celsius
    normed_range = normalizeImg(rangeToUse[0], rangeToUse[1], scaled_depths)
    depthcolor = cv2.applyColorMap(normed_range, 5)

    pcd=None
    if returnpcd:
        colors = cv2.cvtColor(Ldst, cv2.COLOR_BGR2RGB)
        mask = disp > disp.min()
        out_points = points[mask]
        out_colors = colors[mask]

        ind = np.linalg.norm(out_points, axis=1) <= dmax

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(out_points[ind])
        pcd.colors = o3d.utility.Vector3dVector(out_colors[ind] / 255)

    return depths,depthcolor,pcd

