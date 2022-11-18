import os
import sys
import sys
p1=os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(p1)
sys.path.append(os.path.join(p1,'camerahardware'))
import time


import open3d as o3d
import os
import threading
import time
from datetime import datetime
import queue

import numpy as np
import sys
from camerahardware import TIS,calib_codes

import cv2
import pickle as pkl
import velodynecalib

with open(os.path.join('calibfiles', 'velo_step_calib.pkl'), 'rb') as F:
    [Hest, Hest_opt] = pkl.load(F)

# o3d.visualization.draw_geometries([source_pcd,target_pcd])
X = []
p1 = 0
pcd = None
mn = 1
mx = 150
folder='simulations/cam_velo_stepper/2022-11-15-14-48-19'
for step in range(0, 9000, 10):
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
    if step % 500 == 0:
        pcd = pcd.voxel_down_sample(voxel_size=0.01)
pcd = pcd.voxel_down_sample(voxel_size=0.01)
o3d.visualization.draw_geometries([pcd])
# pcd.estimate_normals(
#     search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.15, max_nn=30))
# pcd.orient_normals_consistent_tangent_plane(100)

o3d.io.write_point_cloud(os.path.join(folder, 'cummulative_pcd.pcd'), pcd)
print("saved to folder: ", folder)