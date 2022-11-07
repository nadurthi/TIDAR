# -*- coding: utf-8 -*-
"""
Created on Tue Nov  1 18:16:10 2022

@author: nadur
"""
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
from sensor_msgs.msg import LaserScan,PointCloud2
import numpy as np
import sys

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
    points[...,0] = cloud_array['x']
    points[...,1] = cloud_array['y']
    points[...,2] = cloud_array['z']
    points[...,3] = cloud_array['intensity']
    
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
        self.N=0

    def listener_callback(self, msg):
        X = pointcloud2_to_xyzi_array(msg, remove_nans=True)
        if self.N>0:
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
    def __init__(self,port='/dev/ttyACM0'):
        self.serialcom = serial.Serial(port=port, baudrate=9600, timeout=None)
        time.sleep(5)
        self.serialcom.reset_input_buffer()
        self.serialcom.reset_output_buffer()
        
    def write_read(self,x):
    
        self.serialcom.write(bytes(x, 'utf-8'))
    
        data=None
        while(data is None):
            data = self.serialcom.readline()
            if len(data)==0:
                data=None
    
        data=data.decode()
    
        return data
    def close(self):
        self.serialcom.close()

if __name__=="__main__":
    saveset = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    folder = os.path.join("simulations","cam_velo_stepper",saveset)
    os.makedirs(folder)
    q = queue.Queue()
    doneflg = threading.Event()
    doneflg.clear()


    dang=25
    step=0
    try:
        # rclpy.spin(velo_subscriber)
        ardstepper = ArduinoCtrl()
        

        rclpy.init(args=None)
        velo_subscriber = VeloSubscriber()

        print('before')
        while 1:
            print(step)
            time.sleep(0.01)
            rclpy.spin_once(velo_subscriber,timeout_sec=0.01)
            print("spinned first")
            value = ardstepper.write_read('<%d>'%step)
            print(ardstepper.write_read('<g>'))
            velo_subscriber.q=[]
            velo_subscriber.N=4
            while(1):
                rclpy.spin_once(velo_subscriber, timeout_sec=0.1)
                if len(velo_subscriber.q)>2:
                    break
            velo_subscriber.N=0
            # for i in range(min(4,len(velo_subscriber.q))):
            st=time.time()
            velo_subscriber.q[-1].astype(np.float32).tofile(os.path.join(folder,'velo_%05d_%02d.bin'%(step,0)))
            et=time.time()
            print("time to write = ",et-st)
            # print('writn gto file')
            step=step+dang
            if step>16000-dang:
                step=0
                print("done with one rev - homing and shutdown node")
                value = ardstepper.write_read('<0>')
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
        value = ardstepper.write_read('<0>')
        ardstepper.close()



    # value = write_read('s\n',retflag='ok')
# print(value) # printing the value


# value = write_read('h\n', retflag='ok')
# print(value)  # printing the value
# 
# value = write_read('g\n', retflag=None)
# print(value)  # printing the value
# 
# 
