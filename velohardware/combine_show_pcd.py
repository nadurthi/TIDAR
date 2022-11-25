import copy
import os
import sys
import sys
# p1=os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
# sys.path.append(p1)
# sys.path.append(os.path.join(p1,'camerahardware'))
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
from velohardware import velodynecalib

with open(os.path.join('calibfiles', 'velo_step_calib.pkl'), 'rb') as F:
    [Hest, Hest_opt] = pkl.load(F)

# o3d.visualization.draw_geometries([source_pcd,target_pcd])
X = []
p1 = 0
pcd = None
PCD=[]
mn = 1
mx = 150
# folder='/media/na0043/misc/DATA/cam_velo_stepper/1m/2022-11-15_outside/2022-11-15-14-13-44'
folder='/media/na0043/misc/DATA/cam_velo_stepper/1m/2022-11-15_outside/2022-11-15-14-34-13_calib'
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

    if step==5000:
        pcd = pcd.voxel_down_sample(voxel_size=0.01)
        PCD.append(pcd)
        pcd=None

pcd = pcd.voxel_down_sample(voxel_size=0.01)
PCD.append(pcd)

o3d.visualization.draw_geometries([PCD[0]])
o3d.visualization.draw_geometries([PCD[1]])
o3d.visualization.draw_geometries(PCD)
PCD_copy = copy.deepcopy(PCD)

PCD_copy[0] = PCD_copy[0].voxel_down_sample(voxel_size=0.05)
PCD_copy[1] = PCD_copy[1].voxel_down_sample(voxel_size=0.05)
PCD_copy[0].estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
PCD_copy[0].orient_normals_consistent_tangent_plane(15)
PCD_copy[1].estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
PCD_copy[1].orient_normals_consistent_tangent_plane(15)
print("ok")

crti=o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=0.000001,
                                           relative_rmse=0.000001,
                                           max_iteration=50000)
reg_p2l = o3d.pipelines.registration.registration_icp(
    PCD_copy[1], PCD_copy[0], 1, np.identity(4),
    o3d.pipelines.registration.TransformationEstimationPointToPlane(),crti)
print("ok2")

icp_fine = o3d.pipelines.registration.registration_generalized_icp(
        PCD_copy[1], PCD_copy[0], 1,
        reg_p2l.transformation,
        o3d.pipelines.registration.
        TransformationEstimationForGeneralizedICP(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness=1e-6,
            relative_rmse=1e-6,
            max_iteration=30))
print("ok3")

o3d.visualization.draw_geometries([copy.deepcopy(PCD[1]).transform(icp_fine.transformation),PCD[0] ])
# pcd.estimate_normals(
#     search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.15, max_nn=30))
# pcd.orient_normals_consistent_tangent_plane(100)

pcd = copy.deepcopy(PCD[1]).transform(reg_p2l.transformation)+PCD[0]
o3d.io.write_point_cloud(os.path.join(folder, 'cummulative_pcd.pcd'), pcd)
print("saved to folder: ", folder)