########################################################################
#
# Copyright (c) 2022, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

"""
    This sample demonstrates how to capture a live 3D point cloud   
    with the ZED SDK and display the result in an OpenGL window.    
"""

import sys
#import ogl_viewer.viewer as gl
import pyzed.sl as sl
import numpy as np
import time
import os
import pickle as pkl
import cv2 

folder = 'sampleset1'

if __name__ == "__main__":
    print("Running Depth Sensing sample ... Press 'Esc' to quit")
    
    os.makedirs(os.path.join(folder,"left"),exist_ok=True)
    os.makedirs(os.path.join(folder,"right"),exist_ok=True)
    os.makedirs(os.path.join(folder,"depth"),exist_ok=True)
    os.makedirs(os.path.join(folder,"pointcloud"),exist_ok=True)
    
    init = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD1080,
                                 depth_mode=sl.DEPTH_MODE.ULTRA,
                                 coordinate_units=sl.UNIT.MILLIMETER,
                                 coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP)

    init.depth_minimum_distance = 100  # Set the depth mode to ULTRA
    init.depth_maximum_distance = 5000
    init.camera_fps = 10 

    zed = sl.Camera()
    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()
    calibration_params = zed.get_camera_information().camera_configuration.calibration_parameters
    fx = calibration_params.left_cam.fx
    fy = calibration_params.left_cam.fy
    cx = calibration_params.left_cam.cx
    cy = calibration_params.left_cam.cy
    b = calibration_params.get_camera_baseline()
    R=[ [1.000000, 0.000000, 0.000000, 62.936974],
        [0.000000, 1.000000, 0.000000, 0.000000],
        [0.000000, 0.000000, 1.000000, 0.000000],
        [0.000000, 0.000000, 0.000000, 1.000000]]
    
    dist = calibration_params.left_cam.disto[0]
    
#    rt_param = sl.RuntimeParameters()
#    rt_param.sensing_mode = sl.SENSING_MODE.FILL
    runtime = sl.RuntimeParameters()

    width = zed.get_camera_information().camera_resolution.width
    height = zed.get_camera_information().camera_resolution.height

#    camera_model = zed.get_camera_information().camera_model
    # Create OpenGL viewer
#    viewer = gl.GLViewer()
#    viewer.init(len(sys.argv), sys.argv, camera_model, res)
    
    L=[]
#    point_cloud = sl.Mat(res.width, res.height, sl.MAT_TYPE.F32_C4, sl.MEM.CPU)
    
    cnt=0
#    while viewer.is_available():
    while 1:
        Limage = sl.Mat();
#        Rimage = sl.Mat();
#        depth_map = sl.Mat(width, height, sl.MAT_TYPE.F32_C1)
        point_cloud = sl.Mat(width, height, sl.MAT_TYPE.F32_C4, sl.MEM.CPU)   
        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
#            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA,sl.MEM.CPU)
            zed.retrieve_image(Limage, sl.VIEW.LEFT) # Retrieve left image
#            zed.retrieve_image(Rimage, sl.VIEW.RIGHT) # Retrieve left image
#            zed.retrieve_measure(depth_map, sl.MEASURE.DEPTH) # Retrieve depth

#            L.append([Limage,depth_map,point_cloud])
#            X= np.zeros((res.width*res.height,4))
#            k=1
#            for i in range(res.width):
#                for j in range(res.height):
#                    err,point3D = point_cloud.get_value(200,300)
#                    X[k,0]=point3D[0]
#                    X[k,1]=point3D[1]
#                    X[k,2]=point3D[2]
#                    X[k,3]=point3D[3]
#                    k+=1
            
#            print(x,y,z,color)
#            viewer.updateData(point_cloud)
            
            point_cloud.write(os.path.join(folder,"pointcloud","image_%d.pcd"%cnt))
            Limage.write(os.path.join(folder,"left","image_%d.png"%cnt))
            # saving data
#            if Limage.write(os.path.join(folder,"left","image_%d.png"%cnt))!= sl.ERROR_CODE.SUCCESS:
#                break
#            if Limage.write(os.path.join(folder,"right","image_%d.png"%cnt))!= sl.ERROR_CODE.SUCCESS:
#                break
#            if depth_map.write(os.path.join(folder,"depth","depth_%d.png"%cnt))!= sl.ERROR_CODE.SUCCESS:
#                break
            
            cnt+=1
            time.sleep(0.1)
            print(cnt)
            if cnt==200:
                break

#    viewer.exit()
    zed.close()

#for cnt,m in enumerate(L):
#    Limage,Rimage,depth_map = m
#    Limage.write(os.path.join(folder,"left","image_%d.png"%cnt))
#    Rimage.write(os.path.join(folder,"right","image_%d.png"%cnt))
#    depth_map.write(os.path.join(folder,"depth","depth_%d.png"%cnt))
#    
#with open(os.path.join(folder,'calib.pkl'),'wb') as F:
#    pkl.dump([fx,fy,cx,cy,res],F)