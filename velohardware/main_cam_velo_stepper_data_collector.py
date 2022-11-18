# -*- coding: utf-8 -*-
"""
Created on Tue Nov  1 18:16:10 2022

@author: nadur
"""
import os
import sys
p1=os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(p1)
sys.path.append(os.path.join(p1,'camerahardware'))
from smbus import SMBus
import time
import serial
import time
import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
import open3d as o3d
import os
import threading
import time
from datetime import datetime
import queue
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, PointCloud2
import numpy as np
import sys
from camerahardware import TIS,calib_codes

import cv2
import pickle as pkl
import velodynecalib


mTis = None
try:
    mTis = TIS.MultiTis("calibfiles/cameras.json", onlycams=[0,2])
    mTis.start_cameras()
except:
    mTis = None
    print("No mTis")


def get_xyzi_points(cloud_array, remove_nans=True, dtype=np.float):
    '''Pulls out x, y, and z columns from the cloud recordarray, and returns
	a 3xN matrix.
    '''
    # remove crap points
    if remove_nans:
        mask = np.isfinite(cloud_array['x']) & np.isfinite(cloud_array['y']) & np.isfinite(cloud_array['z'])
        cloud_array = cloud_array[mask]

    # pull out x, y, and z values
    points = np.zeros(cloud_array.shape + (4,), dtype=dtype)
    points[..., 0] = cloud_array['x']
    points[..., 1] = cloud_array['y']
    points[..., 2] = cloud_array['z']
    points[..., 3] = cloud_array['intensity']

    return points


def pointcloud2_to_xyzi_array(cloud_msg, remove_nans=True):
    return get_xyzi_points(rnp.point_cloud2.pointcloud2_to_array(cloud_msg), remove_nans=remove_nans)


class VeloSubscriber(Node):

    def __init__(self):
        super().__init__('velosave_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            'velodyne_points',
            self.listener_callback,
            1)
        self.q = []
        self.cnt = 0
        self.N = 0

    def listener_callback(self, msg):
        X = pointcloud2_to_xyzi_array(msg, remove_nans=True)
        if self.N > 0:
            self.q.append(X)

        #        thrd = threading.Thread(target=savePCD, args=(X,self.cnt))
        #        thrd.daemon=True
        #        thrd.start()
        #        self.q.put(X)
        # X.astype('int16').tofile(filename)
        # np.save(os.path.join(folder, "velodyne", "velodyne_%d" % self.cnt), X)
        # print("pushded cnt= ", self.cnt)
        #
        # self.cnt += 1


class ArduinoCtrl:
    def __init__(self, port='/dev/ttyACM0'):
        self.serialcom = serial.Serial(port=port, baudrate=9600, timeout=None)
        time.sleep(5)
        self.serialcom.reset_input_buffer()
        self.serialcom.reset_output_buffer()

    def write_read(self, x):

        self.serialcom.write(bytes(x, 'utf-8'))

        data = None
        while (data is None):
            data = self.serialcom.readline()
            if len(data) == 0:
                data = None

        data = data.decode()

        return data

    def close(self):
        self.serialcom.close()

class ArduinoI2cStepper:
    def __init__(self,bus=7,i2caddress = 0x33 ):
        self.i2cbus = SMBus(bus)
        self.i2caddress = i2caddress

    def StringToBytes(self,val):
        retVal = []
        for c in val:
            retVal.append(ord(c))
        return retVal
    def BytesToString(selfs,rec):
        ss=[chr(rec[i]) for i in range(len(rec)) if rec[i]<150]
        return "".join(ss)

    def ping(self):
        self.i2cbus.write_block_data(self.i2caddress, 0x01, self.StringToBytes('<p>'))
        time.sleep(0.01)
        while(1):
            rec = self.i2cbus.read_i2c_block_data(self.i2caddress, 0x01, 10)
            ss=self.BytesToString(rec)
            if '<p>' in ss:
                break
            time.sleep(0.05)
        return ss

    def flip(self):
        self.i2cbus.write_block_data(self.i2caddress, 0x01, self.StringToBytes('<f>'))
        time.sleep(0.01)
        while (1):
            rec = self.i2cbus.read_i2c_block_data(self.i2caddress, 0x01, 10)
            ss = self.BytesToString(rec)
            if '<cw>' in ss or '<ccw>' in ss:
                break
            time.sleep(0.05)
        return ss


    def set(self):
        self.i2cbus.write_block_data(self.i2caddress, 0x01, self.StringToBytes('<s>'))
        time.sleep(0.01)
        while (1):
            rec = self.i2cbus.read_i2c_block_data(self.i2caddress, 0x01, 10)
            ss = self.BytesToString(rec)
            if '<s>' in ss:
                break
            time.sleep(0.05)
        return ss

    def getcurrSteploc(self):
        self.i2cbus.write_block_data(self.i2caddress, 0x01, self.StringToBytes('<g>'))
        while (1):
            rec = self.i2cbus.read_i2c_block_data(self.i2caddress, 0x01, 10)
            ss = self.BytesToString(rec)
            if 'None' not in ss:
                break
            time.sleep(0.05)
        return ss

    def encoderval(self):
        self.i2cbus.write_block_data(self.i2caddress, 0x01, self.StringToBytes('<e>'))
        while (1):
            rec = self.i2cbus.read_i2c_block_data(self.i2caddress, 0x01, 10)
            ss = self.BytesToString(rec)
            if 'None' not in ss:
                break
            time.sleep(0.05)
        return ss

    def step(self,stp):
        self.i2cbus.write_block_data(self.i2caddress, 0x01, self.StringToBytes('<%d>'%stp))
        while (1):
            rec = self.i2cbus.read_i2c_block_data(self.i2caddress, 0x01, 10)
            ss = self.BytesToString(rec)
            if '<ok>' in ss:
                break
            time.sleep(0.1)
        return ss

    def close(self):
        self.i2cbus.close()


def recordImagesCalib(folder,tag='cam_calib'):
    cv2windows = []
    for i in range(len(mTis)):
        cv2windows.append(cv2.namedWindow('Camera_%d' % i, cv2.WINDOW_NORMAL))

    while (1):
        images = mTis.snapImages()
        if images is None:
            continue
        h, w = images[0].shape[:2]
        # for i, img in enumerate(images):
        #     cv2.imwrite(os.path.join(folder, "cam_%02d.png" % i), img)
        # print("done saving image")
        gray_cropped = []
        RET = []
        CC = {0: [], 1: [], 2: [], 3: []}
        for jj in range(len(mTis)):
            Limg = images[jj].copy()
            grayimg = cv2.cvtColor(images[jj], cv2.COLOR_BGR2GRAY)  # images[jj].copy()
            ret, cor = calib_codes.getchesspattern(grayimg)

            if ret:
                cv2.drawChessboardCorners(Limg, (4, 5), cor, ret)
                # cv2.drawChessboardCorners(Limg, (4,11), cor, ret)
                CC[jj] = cor
                corners2L = np.vstack(cor)
                mn = np.min(corners2L, axis=0)
                mx = np.max(corners2L, axis=0)
                mn = mn.astype(int)
                mx = mx.astype(int)
                RET.append(True)
                gray_cropped.append(
                    Limg[max((mn[1] - 100), 0):min((mx[1] + 100), h), max((mn[0] - 100), 0):min((mx[0] + 100), w)])
            else:
                gray_cropped.append(Limg)
                RET.append(False)


        for i in range(len(mTis)):
            cv2.imshow('Camera_%d' % i, gray_cropped[i])

        lastkey = cv2.waitKey(10)
        if lastkey == 27 or lastkey == 113:
            break
        if lastkey == 115:
            for jj in range(len(mTis)):
                cv2.imwrite(os.path.join(folder, "cam_%s_%02d.png" %(tag,jj)), images[jj])
            print("saved: ", datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))

    cv2.destroyAllWindows()


if __name__ == "__main__":
    saveset = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    folder = os.path.join("simulations", "cam_velo_stepper", saveset)
    os.makedirs(folder)
    q = queue.Queue()
    doneflg = threading.Event()
    doneflg.clear()

    dang = 20
    step = 0
    try:
        # rclpy.spin(velo_subscriber)
        ardstepper = ArduinoI2cStepper()
        rclpy.init(args=None)

        if mTis is not None:
            recordImagesCalib(folder,tag='cam_calib')
            print("Done with cam-to-cam calib ... now do velo-to-cam calib")
            recordImagesCalib(folder, tag='velo_cam_calib')


        velo_subscriber = VeloSubscriber()

        print('before')
        while 1:
            print(step)
            time.sleep(0.01)
            rclpy.spin_once(velo_subscriber, timeout_sec=0.01)
            print("spinned first")
            value = ardstepper.step(step)
            print(value)
            velo_subscriber.q = []
            velo_subscriber.N = 4
            while (1):
                rclpy.spin_once(velo_subscriber, timeout_sec=0.1)
                if len(velo_subscriber.q) > 2:
                    break
            velo_subscriber.N = 0
            # for i in range(min(4,len(velo_subscriber.q))):

            velo_subscriber.q[-1].astype(np.float32).tofile(os.path.join(folder, 'velo_%05d_%02d.bin' % (step, 0)))

            # print('writn gto file')
            step = step + dang
            if step > 9000 - dang:
                step = 0
                print("done with one rev - homing and shutdown node")
                value = ardstepper.step(0)
                break

    except KeyboardInterrupt:
        print('server stopped cleanly')
    except BaseException:
        print('exception in server:', file=sys.stderr)
        raise
    finally:
        # Destroy the node explicitly
        # (optional - Done automatically when node is garbage collected)
        #        thrd.join()
        doneflg.set()
        velo_subscriber.destroy_node()
        rclpy.shutdown()

        if mTis is not None:
            mTis.stop_cameras()

    value = ardstepper.step(0)
    ardstepper.close()

    with open(os.path.join('calibfiles', 'velo_step_calib.pkl'), 'rb') as F:
        [Hest, Hest_opt] = pkl.load(F)

    # o3d.visualization.draw_geometries([source_pcd,target_pcd])
    X = []
    p1 = 0
    pcd = None
    mn = 1
    mx = 150

    for step in range(0, 9000, 5*dang):
        i = 0
        print(step)
        fname = os.path.join(os.path.join(folder, 'velo_%05d_%02d.bin' % (step, i)))
        if os.path.isfile(fname) is False:
            continue
        x = np.fromfile(fname, dtype=np.float32).reshape(4, -1, order='F').T
        R = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        x[:, :3] = R.dot(x[:, :3].T).T
        # X.append(x)

        ang = (step - 0) * 0.9 / 40
        Hr = velodynecalib.Hmat([-ang, 0, 0], [0, 0, 0])
        Hrv = Hest_opt
        Hvr = np.linalg.inv(Hrv)
        H = np.linalg.multi_dot([Hvr, Hr, Hrv])
        # H=np.linalg.multi_dot([Hrv,Hr,Hvr])
        Hinv = np.linalg.inv(H)

        source_pcd = o3d.geometry.PointCloud()
        source_pcd.points = o3d.utility.Vector3dVector(x[:, :3])
        c = x[:, 3]
        mn = min(mn, np.min(c))
        mx = min(mx, np.max(c))
        c = (c - mn) / mx
        cols = np.ones((len(c), 3)) * c.reshape(-1, 1)
        source_pcd.colors = o3d.utility.Vector3dVector(cols)
        source_pcd = source_pcd.transform(H)

        if pcd is None:
            pcd = source_pcd
        else:
            pcd = pcd + source_pcd
        if step%500==0:
            pcd = pcd.voxel_down_sample(voxel_size=0.01)
    pcd = pcd.voxel_down_sample(voxel_size=0.01)
    o3d.visualization.draw_geometries([pcd])
    # pcd.estimate_normals(
    #     search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.15, max_nn=30))
    # pcd.orient_normals_consistent_tangent_plane(100)

    o3d.io.write_point_cloud(os.path.join(folder, 'cummulative_pcd.pcd'), pcd)
    print("saved to folder: ",folder)