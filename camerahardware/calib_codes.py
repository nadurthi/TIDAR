import cv2
import numpy as np
import pickle as pkl
import time
from collections import namedtuple
import os
import copy
# class CalibrationSet:
#     def __init__(self,filename=None,calib_parameters=None):
#         self.filename=filename
#         if filename is not None:
#             with open(filename, "rb") as F:
#                 self.calib_parameters = pkl.load(F)
#         else:
#             self.calib_parameters=calib_parameters
#
#     def get(self,scale,):
CalibrationMono = namedtuple("CalibrationMono", "scale size ret mtx dst rvecs tvecs newmtx roi")
CalibrationStereo = namedtuple("CalibrationStereo", "scale size R T E F")
CalibrationStereoRect = namedtuple("CalibrationStereoRect", "scale size R1 R2 P1 P2 Q validPixROI1 validPixROI2 map1 map2")

def calib_dict_2_tuple(calib_parameters):
    calib_param=copy.deepcopy(calib_parameters)
    for scale in calib_parameters.keys():
        sze = calib_parameters[scale]['size']
        for camid in calib_parameters[scale]['mono'].keys():

            L=[scale,sze]+calib_parameters[scale]['mono'][camid]
            calib_param[scale]['mono'][camid]=CalibrationMono(*L)
        for (c1,c2) in calib_parameters[scale]['stereo'].keys():
            L=[scale,sze]+calib_parameters[scale]['stereo'][(c1, c2)][4:]
            calib_param[scale]['stereo'][(c1, c2)]=CalibrationStereo(*L)
            L = [scale, sze] + calib_parameters[scale]['stereo_rect'][(c1, c2)]
            calib_param[scale]['stereo_rect'][(c1, c2)] = CalibrationStereoRect(*L)
    return calib_param
        # calib_parameters[scale]['stereo'][(c1, c2)] = [cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E,
        #                                                F]
        # calib_parameters[scale]['stereo_rect'][(c1, c2)] = [R1, R2, P1, P2, Q, validPixROI1, validPixROI2,
        #                                                    (mapx1, mapy1),
        #                                                    (mapx2, mapy2)]
def getchesspattern(Limg_o,fast=False):

    st=time.time()
    if fast:
        retL, cor = cv2.findChessboardCorners(Limg_o, (4, 5), cv2.CALIB_CB_FAST_CHECK  + cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FILTER_QUADS)
    else:
        Limg_o11= cv2.resize(Limg_o, None, fx=1 / 3, fy=1 / 3, interpolation=cv2.INTER_AREA)
        retL, cor = cv2.findChessboardCorners(Limg_o11, (4, 5),cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FILTER_QUADS)
        if retL:
            cor=3*cor
    et = time.time()
    # print("chess board detectio time = ",et-st)
    if retL:
        st = time.time()
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        et=time.time()
        # print("subpixel refine = ", et - st)
        cor = cv2.cornerSubPix(Limg_o, cor, (11, 11), (-1, -1), criteria)

    return retL,cor
def calibrate_Cameras(Limg_o,ho,wo,imgpoints,object_points,scales=None,calib_parameters_old=None):


    idxs = set(imgpoints[0].keys()) & set(imgpoints[1].keys()) & set(imgpoints[2].keys())
    objpointsL_list=[]
    imgpointsL_list_o=[]
    imgpointsM_list_o=[]
    imgpointsR_list_o=[]
    for i in idxs:
        if i in imgpoints[0] and i in imgpoints[1] and i in imgpoints[2]:
            imgpointsL_list_o.append(imgpoints[0][i].astype(np.float32))
            imgpointsM_list_o.append(imgpoints[1][i].astype(np.float32))
            imgpointsR_list_o.append(imgpoints[2][i].astype(np.float32))
            objpointsL_list.append(object_points)

    if scales is None:
        scales=range(1,4)
    calib_parameters={}
    for scale in scales:
        print("scale = ",scale)
        h=int(ho/scale)
        w=int(wo/scale)
        calib_parameters[scale]={'size':(h,w),'scale':scale}
        Limg =cv2.resize(Limg_o.copy(),None,fx=1/scale,fy=1/scale,interpolation=cv2.INTER_AREA)
        imgpointsL_list=[a/scale for a in imgpointsL_list_o]
        imgpointsM_list = [a / scale for a in imgpointsM_list_o]
        imgpointsR_list = [a / scale for a in imgpointsR_list_o]

        imgpoints_set_list = [imgpointsL_list, imgpointsM_list, imgpointsR_list]

        Lret, Lmtx, Ldist, Lrvecs, Ltvecs = cv2.calibrateCamera(objpointsL_list, imgpointsL_list, (w, h), None, None)
        Lnewcameramtx, Lroi = cv2.getOptimalNewCameraMatrix(Lmtx, Ldist, (w, h), 1, (w, h))

        Mret, Mmtx, Mdist, Mrvecs, Mtvecs = cv2.calibrateCamera(objpointsL_list, imgpointsM_list, (w, h), None, None)
        Mnewcameramtx, Mroi = cv2.getOptimalNewCameraMatrix(Mmtx, Mdist, (w, h), 1, (w, h))

        Rret, Rmtx, Rdist, Rrvecs, Rtvecs = cv2.calibrateCamera(objpointsL_list, imgpointsR_list, (w, h), None, None)
        Rnewcameramtx, Rroi = cv2.getOptimalNewCameraMatrix(Rmtx, Rdist, (w, h), 1, (w, h))

        calib_parameters[scale]['mono'] = {0: [Lret, Lmtx, Ldist, Lrvecs, Ltvecs, Lnewcameramtx, Lroi],
                                    1: [Mret, Mmtx, Mdist, Mrvecs, Mtvecs, Mnewcameramtx, Mroi],
                                    2: [Rret, Rmtx, Rdist, Rrvecs, Rtvecs, Rnewcameramtx, Rroi],
                                    }
        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # colors = ['r', 'g', 'b']
        # for c in range(3):
        #     rvecs = calib_parameters[scale]['mono'][c][3]
        #     tvecs = calib_parameters[scale]['mono'][c][4]
        #     for i in range(len(rvecs)):
        #         Rmat, _ = cv2.Rodrigues(rvecs[i])
        #         H = np.identity(4)
        #         H[0:3, 0:3] = Rmat
        #         H[0:3, 3] = tvecs[i].reshape(-1)
        #         campos = np.linalg.inv(H)[0:3, 3]
        #         ax.plot(campos[0], campos[1], campos[2], colors[c] + 'o')
        #
        # ax.plot(objpointsL_list[0][:, 0], objpointsL_list[0][:, 1], objpointsL_list[0][:, 2], 'ko')


        # %% stereo calibrate
        h, w = Limg.shape[:2]
        calib_parameters[scale]['stereo'] = {}
        calib_parameters[scale]['stereo_rect'] = {}
        for c1, c2 in [(0, 2), (0, 1),(1,0),(1,2)]:

            imgpt1 = imgpoints_set_list[c1]
            imgpt2 = imgpoints_set_list[c2]

            mtx1 = calib_parameters[scale]['mono'][c1][1]
            dist1 = calib_parameters[scale]['mono'][c1][2]

            mtx2 = calib_parameters[scale]['mono'][c2][1]
            dist2 = calib_parameters[scale]['mono'][c2][2]

            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 3000, 0.000001)
            # ,flags=cv2.CALIB_FIX_INTRINSIC+cv2.CALIB_USE_EXTRINSIC_GUESS
            retval, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = cv2.stereoCalibrate(objpointsL_list,
                                                                                                            imgpt1, imgpt2,
                                                                                                            mtx1, dist1,
                                                                                                            mtx2, dist2,
                                                                                                            (w, h),
                                                                                                            criteria=criteria)
            calib_parameters[scale]['stereo'][(c1, c2)] = [cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F]

            R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2,
                                                                             distCoeffs2, (w, h), R, T)
            mapx1, mapy1 = cv2.initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1[0:3, 0:3], (w, h), cv2.CV_32FC2)
            mapx2, mapy2 = cv2.initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2[0:3, 0:3], (w, h), cv2.CV_32FC2)

            calib_parameters[scale]['stereo_rect'][(c1, c2)] = [R1, R2, P1, P2, Q, validPixROI1, validPixROI2, (mapx1, mapy1),
                                                         (mapx2, mapy2)]

    return calib_parameters


def calibrate_Cameras_2cams(Limg_o,ho,wo,imgpoints,object_points,scales=None,calib_parameters_old=None):


    idxs = set(imgpoints[0].keys()) &  set(imgpoints[1].keys())
    objpointsL_list=[]
    imgpointsL_list_o=[]
    imgpointsR_list_o=[]
    for i in idxs:
        if i in imgpoints[0] and i in imgpoints[1]:
            imgpointsL_list_o.append(imgpoints[0][i].astype(np.float32))
            imgpointsR_list_o.append(imgpoints[1][i].astype(np.float32))
            objpointsL_list.append(object_points)

    if scales is None:
        scales=range(1,4)
    calib_parameters={}
    for scale in scales:
        print("scale = ",scale)
        h=int(ho/scale)
        w=int(wo/scale)
        calib_parameters[scale]={'size':(h,w),'scale':scale}
        Limg =cv2.resize(Limg_o.copy(),None,fx=1/scale,fy=1/scale,interpolation=cv2.INTER_AREA)
        imgpointsL_list=[a/scale for a in imgpointsL_list_o]
        imgpointsR_list = [a / scale for a in imgpointsR_list_o]

        imgpoints_set_list = [imgpointsL_list, imgpointsR_list]

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 300, 0.000000001)
        Lmtx = None
        Ldist = None
        Rmtx = None
        Rdist = None

        if calib_parameters_old is not None:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER + cv2.CALIB_USE_INTRINSIC_GUESS, 300, 0.000000001)
            Lmtx = calib_parameters_old[scale]['mono'][0][1].copy()
            Ldist = calib_parameters_old[scale]['mono'][0][2].copy()
            if 2 in calib_parameters_old[scale]['mono'].keys():
                Rmtx = calib_parameters_old[scale]['mono'][2][1].copy()
                Rdist = calib_parameters_old[scale]['mono'][2][2].copy()
            else:
                Rmtx = calib_parameters_old[scale]['mono'][1][1].copy()
                Rdist = calib_parameters_old[scale]['mono'][1][2].copy()

        Lret, Lmtx, Ldist, Lrvecs, Ltvecs = cv2.calibrateCamera(objpointsL_list, imgpointsL_list, (w, h), Lmtx, Ldist,criteria=criteria)
        Lnewcameramtx, Lroi = cv2.getOptimalNewCameraMatrix(Lmtx, Ldist, (w, h), 1, (w, h))

        Rret, Rmtx, Rdist, Rrvecs, Rtvecs = cv2.calibrateCamera(objpointsL_list, imgpointsR_list, (w, h), Rmtx, Rdist)
        Rnewcameramtx, Rroi = cv2.getOptimalNewCameraMatrix(Rmtx, Rdist, (w, h), 1, (w, h))

        calib_parameters[scale]['mono'] = {0: [Lret, Lmtx, Ldist, Lrvecs, Ltvecs, Lnewcameramtx, Lroi],
                                    1: [Rret, Rmtx, Rdist, Rrvecs, Rtvecs, Rnewcameramtx, Rroi],
                                    }


        # %% stereo calibrate
        h, w = Limg.shape[:2]
        calib_parameters[scale]['stereo'] = {}
        calib_parameters[scale]['stereo_rect'] = {}
        for c1, c2 in [ (0, 1)]:

            imgpt1 = imgpoints_set_list[c1]
            imgpt2 = imgpoints_set_list[c2]

            # mtx1 = calib_parameters[scale]['mono'][c1][1]
            # dist1 = calib_parameters[scale]['mono'][c1][2]
            #
            # mtx2 = calib_parameters[scale]['mono'][c2][1]
            # dist2 = calib_parameters[scale]['mono'][c2][2]

            if calib_parameters_old is None:
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 300, 0.000001)
                mtx1 = None
                dist1 = None
                mtx2 = None
                dist2 = None
                R=None
                T=None

            else:
                criteria = (
                cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER + cv2.CALIB_FIX_INTRINSIC +cv2.CALIB_USE_EXTRINSIC_GUESS , 300, 0.000000001)

                if (0,2) in calib_parameters_old[scale]['stereo'].keys():
                    mtx1 = calib_parameters_old[scale]['stereo'][(0, 2)][0].copy()
                    dist1 = calib_parameters_old[scale]['stereo'][(0,2)][1].copy()
                    mtx2 = calib_parameters_old[scale]['stereo'][(0, 2)][2].copy()
                    dist2 = calib_parameters_old[scale]['stereo'][(0, 2)][3].copy()
                    R= calib_parameters_old[scale]['stereo'][(0, 2)][4].copy()
                    T= calib_parameters_old[scale]['stereo'][(0, 2)][5].copy()
                else:
                    mtx1 = calib_parameters_old[scale]['stereo'][(0, 1)][0].copy()
                    dist1 = calib_parameters_old[scale]['stereo'][(0, 1)][1].copy()
                    mtx2 = calib_parameters_old[scale]['stereo'][(0, 1)][2].copy()
                    dist2 = calib_parameters_old[scale]['stereo'][(0, 1)][3].copy()
                    R = calib_parameters_old[scale]['stereo'][(0, 1)][4].copy()
                    T = calib_parameters_old[scale]['stereo'][(0, 1)][5].copy()


            # ,flags=cv2.CALIB_FIX_INTRINSIC+cv2.CALIB_USE_EXTRINSIC_GUESS
            retval, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = cv2.stereoCalibrate(objpointsL_list,
                                                                                                            imgpt1, imgpt2,
                                                                                                            mtx1, dist1,
                                                                                                            mtx2, dist2,
                                                                                                            (w, h),R,T,
                                                                                                            criteria=criteria)
            calib_parameters[scale]['stereo'][(c1, c2)] = [cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F]

            R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2,
                                                                             distCoeffs2, (w, h), R, T)
            mapx1, mapy1 = cv2.initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1[0:3, 0:3], (w, h), cv2.CV_32FC2)
            mapx2, mapy2 = cv2.initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2[0:3, 0:3], (w, h), cv2.CV_32FC2)

            calib_parameters[scale]['stereo_rect'][(c1, c2)] = [R1, R2, P1, P2, Q, validPixROI1, validPixROI2, (mapx1, mapy1),
                                                         (mapx2, mapy2)]

    return calib_parameters
