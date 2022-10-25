# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import pandas as pd
import numpy as np
import open3d as o3d
import os
import cv2 
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

path=r"C:\Users\nadur\Downloads\L515-optitrackData"
pose_opt = pd.read_csv(os.path.join(path,"Poses.csv"),header=None,
                       names=['sec','nsec','x','y','z','qx','qy','qz','qw'])


# H_f_c?
# H_c_0*H_0_1*H_1_c
Hpose_w_f=[]

for ind in range(414):
    r = R.from_quat(pose_opt.loc[ind,['qx','qy','qz','qw']])
    RR=r.as_matrix()
    H=np.identity(4)
    H[0:3,0:3]=RR
    H[0:3,3]=pose_opt.loc[ind,['x','y','z']]
    Hpose_w_f.append(H)
    
    
Hpose_rel=[np.identity(4)]
for i in range(len(Hpose_w_f)-1):
    H_w_0=Hpose_w_f[i]
    H_w_1=Hpose_w_f[i+1]
    H_0_1 = np.linalg.inv(H_w_0).dot(H_w_1)
    Hpose_rel.append(H_0_1)    
