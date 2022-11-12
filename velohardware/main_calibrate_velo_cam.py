import cv2
import open3d as o3d
import os
from scipy.spatial.transform import Rotation as RR
import copy
import scipy.optimize
import pickle as pkl
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from sklearn.cluster import AgglomerativeClustering
from sklearn.cluster import KMeans
from sklearn.cluster import AffinityPropagation
from camerahardware import calib_codes,velodynecalib
from scipy.spatial import KDTree

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



pcd = o3d.io.read_point_cloud(pcdfile)
Xv,Xvcol = velodynecalib.pcd2points(pcd)

# Xv=np.array(pcd.points).copy()
# Xvcol = np.array(pcd.colors).copy()

indf=(Xv[:,0]<0) & (Xv[:,2]<0)
pcdv = o3d.geometry.PointCloud()
pcdv.points=o3d.utility.Vector3dVector(Xv[indf,:])
pcdv.colors = o3d.utility.Vector3dVector(np.array(pcd.colors)[indf])
mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=0.6, origin=[0, 0, 0])
o3d.visualization.draw_geometries([pcdv,mesh_frame])

pcd_out = copy.deepcopy(pcd.voxel_down_sample(voxel_size=0.025))
pcd_out.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
pcd_out.orient_normals_consistent_tangent_plane(100)
XX=np.hstack([np.array(pcd_out.points),np.array(pcd_out.normals)])
clustering = DBSCAN(eps=0.1, min_samples=25).fit(XX)
# clustering = AffinityPropagation().fit(XX)
# clustering = AgglomerativeClustering(n_clusters=10).fit(XX)
# clustering = KMeans(n_clusters=20, random_state=0).fit(XX)
labels=clustering.labels_


dsquare=0.175
max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
pcd_out.colors = o3d.utility.Vector3dVector(colors[:, :3])
o3d.visualization.draw_geometries([pcd_out,mesh_frame])
L=np.arange(len(labels),dtype=int)
for i in range(max_label):
    print(i,max_label)
    inlier_cloud = pcd_out.select_by_index(L[labels==i])


    bbx3d=inlier_cloud.get_axis_aligned_bounding_box()
    bbx3d_scaled=bbx3d.scale(1.2, bbx3d.get_center())
    insidebox_inds = bbx3d_scaled.get_point_indices_within_bounding_box(pcd.points)
    chess_cloud = pcd.select_by_index(insidebox_inds)

    plane_model, inliers2 = chess_cloud.segment_plane(distance_threshold=0.02,
                                                      ransac_n=3,
                                                      num_iterations=1000)


    chess_cloud_inlier = chess_cloud.select_by_index(inliers2)
    if chess_cloud_inlier.get_oriented_bounding_box().volume()>0.2:
        continue
    Xchess = np.array(chess_cloud_inlier.points).copy()
    orig = np.mean(Xchess,axis=0)
    kdt=KDTree(Xchess)

    _,x0ind=kdt.query(orig, k=1)
    x0=Xchess[x0ind]
    x1ind = kdt.query_ball_point(x0, 0.25)[10]
    x1 = Xchess[x1ind]

    [a, b, c, d] = plane_model
    nbar = [a, b, c]
    nbar=nbar/np.linalg.norm(nbar)
    zbar=[0,0,1]
    vbar=np.cross(zbar,nbar)
    theta=np.arccos(np.dot(nbar,zbar))
    rotmat1 = RR.from_rotvec(-theta * vbar).as_matrix()
    rotmat2 = RR.from_rotvec(0*np.pi/180 * np.array([0,0,1])).as_matrix()
    Xchess_trans=Xchess-x0
    Xchess_rot=np.matmul(rotmat2,rotmat1).dot(Xchess_trans.T).T



    cc=np.array(chess_cloud_inlier.colors).copy()
    # plt.hist(cc[:,0],bins=300)
    cx,bins=np.histogram(cc[:,0],bins=300)
    ind=np.argsort(cx)
    thresh_int=np.mean(bins[ind][-5])
    # thresh_int=0.1
    # clustering = KMeans(n_clusters=2, random_state=0).fit(cc[:,0].reshape(-1,1))
    # thresh_int = np.mean(clustering.cluster_centers_)
    cc[cc>thresh_int]=1
    cc[cc < thresh_int] = 0.1

    # pcd_Xchess_rot = o3d.geometry.PointCloud()
    # pcd_Xchess_rot.points = o3d.utility.Vector3dVector(Xchess_rot)
    # pcd_Xchess_rot.colors = o3d.utility.Vector3dVector(cc)
    # o3d.visualization.draw_geometries([pcd_Xchess_rot, mesh_frame])

    Xwhite = Xchess_rot[cc[:,0]>0.5,:]
    Xblack = Xchess_rot[cc[:,0] < 0.5,:]
    # plt.plot(Xwhite[:,0],Xwhite[:,1],'r.')
    # plt.plot(Xblack[:, 0], Xblack[:, 1], 'g.')
    try:
        blackclustering = AgglomerativeClustering(n_clusters=15,linkage='ward').fit(Xblack)
    except:
        continue
    centers=[]
    for ll in range(15):
        centers.append(np.mean(Xblack[blackclustering.labels_==ll, :],axis=0))
    centers=np.array(centers)

    # plt.scatter(Xblack[:, 0], Xblack[:, 1], c=blackclustering.labels_, cmap=plt.cm.nipy_spectral)
    # plt.plot(centers[:,0],centers[:,1],'y*')
    # pcd_copy = copy.deepcopy(chess_cloud_inlier)
    # pcd_copy.colors=  o3d.utility.Vector3dVector(cc)
    # o3d.visualization.draw_geometries([pcd_copy, mesh_frame])
    kdt2 = KDTree(centers)
    D, x0ind = kdt2.query(centers, k=2)
    D=np.max(D,axis=1)
    if np.max(D)-np.min(D)>0.05:
        pass
        continue
    D, x0ind = kdt2.query(centers, k=5)
    tt=0.175*(np.sqrt(2)+2)/2
    D=D[:,1:]
    i1 = D < tt
    i2 = D > tt
    D[i1]=1
    D[i2]=0
    D=np.sum(D,axis=1)
    leftcorners = centers[D==1]
    if len(leftcorners)!=2:
        continue
        pass
    diff=leftcorners[0]-leftcorners[1]
    theta2=np.arctan2(diff[1],diff[0])


    rotmat2 = RR.from_rotvec(-(theta2+np.pi/2)  * np.array([0, 0, 1])).as_matrix()
    leftcorners_aligned=rotmat2.dot(leftcorners.T).T
    if np.all(leftcorners_aligned[:,0]>0):
        rotmat2 = RR.from_rotvec(-(theta2 - np.pi / 2) * np.array([0, 0, 1])).as_matrix()
        leftcorners_aligned = rotmat2.dot(leftcorners.T).T
    # Xchess_trans = Xchess - x0
    # Xchess_rot = np.matmul(rotmat2, rotmat1).dot(Xchess_trans.T).T
    # pcdtest = o3d.geometry.PointCloud()
    # pcdtest.points = o3d.utility.Vector3dVector(Xchess_rot)
    # pcdtest.colors = o3d.utility.Vector3dVector(cc)
    # o3d.visualization.draw_geometries([pcdtest, mesh_frame])

    centers_aligned=rotmat2.dot(centers.T).T
    # plt.plot(centers_aligned[:,0],centers_aligned[:,1],'r*')
    if leftcorners_aligned[0,1]<leftcorners_aligned[1,1]:
        bottom_left_center = leftcorners_aligned[0]
    else:
        bottom_left_center = leftcorners_aligned[1]
    bottom_left = bottom_left_center+[dsquare/2,dsquare/2,0]
    objp_checker = np.zeros((5 * 4, 3), np.float32)
    objp_checker[:, :2] = np.mgrid[0:5, 0:4].T.reshape(-1, 2,order='F')
    objp_checker = objp_checker * dsquare
    objp_checker+=bottom_left

    # now reversing the points
    objp_checker = np.linalg.inv(np.matmul(rotmat2,rotmat1)).dot(objp_checker.T).T
    objp_checker=objp_checker+x0

    pcdobjp_checker= o3d.geometry.PointCloud()
    pcdobjp_checker.points = o3d.utility.Vector3dVector(objp_checker)
    colobj=np.zeros((objp_checker.shape[0],3))
    colobj[:,0]=1
    colobj[0] = [0,1,0]
    colobj[1] = [0,0,1]
    colobj[4] = [1,1, 0]
    pcdobjp_checker.colors = o3d.utility.Vector3dVector(colobj)
    o3d.visualization.draw_geometries([pcdobjp_checker,pcd, mesh_frame])

    break

# now get image points----------------------------
cimg=cv2.imread(os.path.join(folder, 'cam_00.png'))
img = cv2.imread(os.path.join(folder, 'cam_00.png'),0)
h,  w = cimg.shape[:2]

dst = cv2.medianBlur(img,5)
dst = cv2.adaptiveThreshold(dst,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,91,10)


cv2.namedWindow('gg', cv2.WINDOW_NORMAL)
while 1:
    cv2.imshow("gg", cimg)
    lastkey = cv2.waitKey(10)
    if lastkey == 27 or lastkey == 113:
        break
        time.sleep(0.2)
cv2.destroyAllWindows()

ret, cor = cv2.findChessboardCorners(dst, (4, 4), cv2.CALIB_CB_FAST_CHECK  + cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FILTER_QUADS)
print(ret)
if ret:
    st = time.time()
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    et = time.time()
    # print("subpixel refine = ", et - st)
    cor = cv2.cornerSubPix(dst, cor, (11, 11), (-1, -1), criteria)

cv2.drawChessboardCorners(cimg, (4,4), cor, ret)

# order_img_points=[12,8,4,0,13,9,5,1,14,10,6,2,15,11,7,3]
order_img_points=[3,7,11,15,
                  2,6,10,14,
                  1,5,9,13,
                  0,4,8,12]
img_pts=cor[order_img_points]

# Lret, Lmtx, Ldist, Lrvecs, Ltvecs = cv2.calibrateCamera([objp_checker[:-4].astype(np.float32)], [img_pts], (w, h), None, None)
with open("calibrated_camera_parameters.pkl","rb") as F:
    calib_parameters = pkl.load(F)

Lmtx, Ldist =calib_parameters[1]['mono'][0][1:3]
ret,rvec,tvec,err=	cv2.solvePnPGeneric(objp_checker[:-4].astype(np.float32), img_pts, Lmtx, Ldist)

tvec=tvec[0].reshape(-1)
Rcv,_ = cv2.Rodrigues(rvec[0])
objp_cam = Rcv.dot(objp_checker.T).T+tvec
with open("calibfiles/velo_cam_calib.pkl",'wb') as F:
    pkl.dump([Rcv,tvec],F)