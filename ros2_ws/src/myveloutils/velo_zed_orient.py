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
import open3d as o3d

import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
import open3d as o3d
import os
import threading
import time
import queue
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan,PointCloud2
import numpy as np
import sys

folder = 'simulations/sampleset1'
os.makedirs(os.path.join(folder,"velodyne"),exist_ok=True)

class VeloSubscriber(Node):

    def __init__(self,q):
        super().__init__('velosave_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            'velodyne_points',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.q=q
        self.cnt=0
    
        self.Xcurr=None
    def read_Xcurr(self):
        if self.Xcurr is not None:
            X=self.Xcurr.copy()
            self.Xcurr=None
            return X
        return None
            
    def listener_callback(self, msg):
        
        X = rnp.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=True)
#        thrd = threading.Thread(target=savePCD, args=(X,self.cnt))
#        thrd.daemon=True
#        thrd.start()
#        self.q.put(X)
        np.save(os.path.join(folder,"velodyne","velodyne_%d"%self.cnt), X)
        print("pushded cnt= ",self.cnt)

        
        self.cnt+=1

        

def cv2pcd_to_o3dpcd(X):
    PP=np.ravel(X[:, :, 3]).view('uint8').reshape((X.shape[0],X.shape[1], 4))
    pcd = o3d.geometry.PointCloud()
    XX=X[:,:,:3].reshape(-1,3)/1000
    ind=np.isfinite(XX[:,0])
    pcd.points = o3d.utility.Vector3dVector(XX[ind])
    cc=PP[:,:,:3].reshape(-1,3)[ind]
    pcd.colors = o3d.utility.Vector3dVector(cc/255.0) #
#    o3d.visualization.draw_geometries([pcd])
    return pcd


if __name__ == "__main__":
    print("Running Depth Sensing sample ... Press 'Esc' to quit")

    init = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD1080,
                                 depth_mode=sl.DEPTH_MODE.ULTRA,
                                 coordinate_units=sl.UNIT.MILLIMETER,
                                 coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP)

    init.depth_minimum_distance = 100  # Set the depth mode to ULTRA
    init.depth_maximum_distance = 5000
    init.camera_fps = 20 
    

    zed = sl.Camera()
    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()

    tracking_parameters = sl.PositionalTrackingParameters()
    err = zed.enable_positional_tracking(tracking_parameters)

    
    runtime = sl.RuntimeParameters()
    camera_pose = sl.Pose()

    camera_info = zed.get_camera_information()
    
    py_translation = sl.Translation()
    pose_data = sl.Transform()
    
#    calibration_params = zed.get_camera_information().camera_configuration.calibration_parameters
#    fx = calibration_params.left_cam.fx
#    fy = calibration_params.left_cam.fy
#    cx = calibration_params.left_cam.cx
#    cy = calibration_params.left_cam.cy
#    b = calibration_params.get_camera_baseline()
#    R=[ [1.000000, 0.000000, 0.000000, 62.936974],
#        [0.000000, 1.000000, 0.000000, 0.000000],
#        [0.000000, 0.000000, 1.000000, 0.000000],
#        [0.000000, 0.000000, 0.000000, 1.000000]]
#    
#    dist = calibration_params.left_cam.disto[0]
#    
#    rt_param = sl.RuntimeParameters()
#    rt_param.sensing_mode = sl.SENSING_MODE.FILL
#    runtime = sl.RuntimeParameters()

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
        Rimage = sl.Mat();
#        depth_map = sl.Mat(width, height, sl.MAT_TYPE.F32_C1)
        point_cloud = sl.Mat(width, height, sl.MAT_TYPE.F32_C4, sl.MEM.CPU)   
        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA,sl.MEM.CPU)
            zed.retrieve_image(Limage, sl.VIEW.LEFT) # Retrieve left image
            zed.retrieve_image(Rimage, sl.VIEW.RIGHT) # Retrieve left image
            tracking_state = zed.get_position(camera_pose) #, sl.REFERENCE_FRAME.WORLD
            if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                rotation = camera_pose.get_rotation_vector()
                translation = camera_pose.get_translation(py_translation)
                text_rotation = str((round(rotation[0], 2), round(rotation[1], 2), round(rotation[2], 2)))
                text_translation = str((round(translation.get()[0], 2), round(translation.get()[1], 2), round(translation.get()[2], 2)))
                
                print(text_rotation)
                print(text_translation)
                pose_data = camera_pose.pose_data(sl.Transform())
                print(pose_data)
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
            
            
            # saving data
#            if Limage.write(os.path.join(folder,"left","image_%d.png"%cnt))!= sl.ERROR_CODE.SUCCESS:
#                break
#            if Limage.write(os.path.join(folder,"right","image_%d.png"%cnt))!= sl.ERROR_CODE.SUCCESS:
#                break
#            if depth_map.write(os.path.join(folder,"depth","depth_%d.png"%cnt))!= sl.ERROR_CODE.SUCCESS:
#                break
            
#            image_ocv = Limage.get_data()
            # Display the left image from the numpy array
#            cv2.imshow("Image", image_ocv)
#            key = cv2.waitKey(0)
            
#            if key == 27 or key == 113:
#                break
            
            if 0: #key == 115:
#                point_cloud.write(os.path.join(folder,"pointcloud","image_%d.pcd"%cnt))
                Limage.write(os.path.join(folder,"left","image_%d.png"%cnt))
                Rimage.write(os.path.join(folder,"right","image_%d.png"%cnt))
                
#                PP=np.ravel(point_cloud.get_data()[:, :, 3]).view('uint8').reshape((height,width, 4))
                cvpcd=point_cloud.get_data()
                np.save(os.path.join(folder,"pointcloud","image_%d"%cnt),cvpcd)
                
#                X= np.zeros((width*height,3))
#                Xc= np.zeros((width*height,3))
#                k=0
#                for i in range(width):
#                    for j in range(height):
#                        err,point3D = point_cloud.get_value(i,j)
#                        X[k,0]=point3D[0]
#                        X[k,1]=point3D[1]
#                        X[k,2]=point3D[2]
#                        
#                        Xc[k,0]=image_ocv[i,j,0]
#                        Xc[k,1]=image_ocv[i,j,1]
#                        Xc[k,2]=image_ocv[j,i,2]
#                        
#                        
#                        k+=1
#                
#                break
#                                        
#                pcd = o3d.geometry.PointCloud()
#                X[:,:3]=X[:,:3]/1000
#                pcd.points = o3d.utility.Vector3dVector(X)
#                pcd.colors = o3d.utility.Vector3dVector(Xc)
#                o3d.io.write_point_cloud(os.path.join(folder,"pointcloud","image_%d.pcd"%cnt), pcd)
                
                cnt+=1
                
                print(cnt)
                
            time.sleep(0.1)
            if cnt==300:
                break

#    viewer.exit()
    zed.close()
    cv2.destroyAllWindows()
#for cnt,m in enumerate(L):
#    Limage,Rimage,depth_map = m
#    Limage.write(os.path.join(folder,"left","image_%d.png"%cnt))
#    Rimage.write(os.path.join(folder,"right","image_%d.png"%cnt))
#    depth_map.write(os.path.join(folder,"depth","depth_%d.png"%cnt))
#    
#with open(os.path.join(folder,'calib.pkl'),'wb') as F:
#    pkl.dump([fx,fy,cx,cy,res],F)