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


class VeloSubscriber(Node):

    def __init__(self):
        super().__init__('velosave_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            'velodyne_points',
            self.listener_callback,
            10)
        self.q = []
        self.cnt = 0


    def listener_callback(self, msg):
        X = rnp.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=True)
        if len(self.q)>15:
            self.q=[]

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




class ArduinoStepper:
    def __init__(self):
        self.arduino=serial.Serial(port='/dev/ttyACM1', baudrate=9600, timeout=None)

    def write_read(self,x, retflag=None):
        # self.arduino.reset_input_buffer()
        # self.arduino.reset_output_buffer()
        self.arduino.write(bytes(x, 'utf-8'))



        data = None
        while (data is None):
            data = self.arduino.readline()
            if len(data) == 0:
                data = None

        data = data.decode()

        if retflag is not None:
            if retflag in data:
                return True, data
            else:
                return False, data
        else:
            return True, data


if __name__=="__main__":
    saveset = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    folder = os.path.join("simulations","cam_velo_stepper",saveset)
    os.makedirs(folder)
    q = queue.Queue()
    doneflg = threading.Event()
    doneflg.clear()


    dang=100
    step=0
    try:
        # rclpy.spin(velo_subscriber)
        ardstepper = ArduinoStepper()
        time.sleep(1)

        rclpy.init(args=None)
        velo_subscriber = VeloSubscriber()

        print('before')
        while 1:
            print(step)
            rclpy.spin_once(velo_subscriber,timeout_sec=0.01)
            print("spinned first")
            value = ardstepper.write_read('+%d\n'%dang, retflag='ok')
            time.sleep(1)
            print("steped ok")
            velo_subscriber.q=[]
            for i in range(5):
                rclpy.spin_once(velo_subscriber, timeout_sec=0.01)
                print("itertating spin")
            print(len(velo_subscriber.q))
            for i in range(min(4,len(velo_subscriber.q))):
                velo_subscriber.q[i].astype(np.float32).tofile(os.path.join(folder,'velo_%05d_%02d.bin'%(step,i)))
                print('writn gto file')
            step=step+dang
            if step>16000-dang:
                print("done with one rev - homing and shutdown node")
                value = ardstepper.write_read('h\n', retflag='ok')
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
        ardstepper.arduino.close()



    # value = write_read('s\n',retflag='ok')
# print(value) # printing the value


# value = write_read('h\n', retflag='ok')
# print(value)  # printing the value
# 
# value = write_read('g\n', retflag=None)
# print(value)  # printing the value
# 
# 
