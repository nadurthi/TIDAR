import cv2
import open3d as o3d
import os
from scipy.spatial.transform import Rotation as RR
import copy
import scipy.optimize
import pickle as pkl
import numpy as np


def Hmat(eulangles, t):
    R = RR.from_euler('zyx', eulangles, degrees=True)
    H = np.identity(4)
    H[0:3, 0:3] = R.as_matrix()
    H[0:3, 3] = t
    return H
with open(os.path.join('calibfiles', 'velo_step_calib.pkl'), 'rb') as F:
    [Hest, Hest_opt] = pkl.load(F)

step=0
ang= (step-0)*0.9/40
Hr = Hmat([-ang,0,0],[0,0,0])
Hrv=Hest_opt
Hvr=np.linalg.inv(Hrv)
H=np.linalg.multi_dot([Hvr,Hr,Hrv])
Hinv = np.linalg.inv(H)
# rHs=Hinv   .... source to local 0-step rotation frame
# Hrv : velodyne to rotation frame


folder='/media/na0043/misc/DATA/cam_velo_stepper/2022-11-07-16-38-06'
pcdfile = os.path.join(folder,'cummulative_pcd.pcd')
img = cv2.imread(os.path.join(folder, 'cam_00.png'))



pcd = o3d.io.read_point_cloud(pcdfile)
Xv=np.array(pcd.points)

indf=Xv[:,0]<0
pcdv = o3d.geometry.PointCloud()
pcdv.points=o3d.utility.Vector3dVector(Xv[indf,:])
pcdv.colors = o3d.utility.Vector3dVector(np.array(pcd.colors)[indf])
mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=0.6, origin=[0, 0, 0])
o3d.visualization.draw_geometries([pcdv,mesh_frame])
# red is x, green is y, blue is z

# #%%
# Hcv_est=np.linalg.multi_dot([Hmat([0,0,0],[0.5,0,0]),Hmat([0,0,90],[0,0,0]),Hmat([90,0,0],[0,0,0]),Hrv])
# Xc=Hcv_est[0:3,0:3].dot(Xv.T).T+Hcv_est[0:3,3]
#
# indf=Xc[:,2]>-1000000000000
# pcdc = o3d.geometry.PointCloud()
# pcdc.points=o3d.utility.Vector3dVector(Xc[indf,:])
# pcdc.colors = o3d.utility.Vector3dVector(np.array(pcd.colors)[indf])
#
# o3d.visualization.draw_geometries([pcdc,mesh_frame])

h=1500
w=1500
K=np.array([[609,0,w//2],[0,609,h//2],[0,0,1]])

indc=Xv[:,2]<0
Xcfrust=K.dot(Xv[indf,:].T).T
Xcfrust=Xcfrust/Xcfrust[:,2].reshape(-1,1)
img_vf = np.zeros((h,w))
ind_f = (Xcfrust[:,0]>0) & (Xcfrust[:,0]<w-1) & (Xcfrust[:,1]>0) & (Xcfrust[:,1]<h-1)
Xcinside = Xcfrust[ind_f,:].round().astype(int)
Colcinside = np.array(pcd.colors)[indf,:][ind_f,:]
for i in range(len(Xcinside)):
    img_vf[Xcinside[i,0],Xcinside[i,1]]=np.round(Colcinside[i,0]*100).astype(int)


cv2.namedWindow('gg', cv2.WINDOW_NORMAL)
cv2.imshow("gg",img_vf)
while 1:
    lastkey = cv2.waitKey(10)
    if lastkey == 27 or lastkey == 113:
        break
cv2.destroyAllWindows()


pcd2 = o3d.geometry.PointCloud()
pcd2.points=o3d.utility.Vector3dVector(Xcinside)
pcd2.colors = o3d.utility.Vector3dVector(Colcinside)
o3d.visualization.draw_geometries([pcd2])


