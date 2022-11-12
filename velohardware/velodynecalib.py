
import cv2
import open3d as o3d
import numpy as np
import copy
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation as RR
from sklearn.cluster import AgglomerativeClustering

mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=0.6, origin=[0, 0, 0])

def Hmat(eulangles, t):
    R = RR.from_euler('zyx', eulangles, degrees=True)
    H = np.identity(4)
    H[0:3, 0:3] = R.as_matrix()
    H[0:3, 3] = t
    return H

def plotimage(cimg):
    cv2.namedWindow('gg', cv2.WINDOW_NORMAL)
    while 1:
        cv2.imshow("gg", cimg)
        lastkey = cv2.waitKey(10)
        if lastkey == 27 or lastkey == 113:
            break
            time.sleep(0.2)
    cv2.destroyAllWindows()



def points2pcd(X,index=None,colmat=None,colvec=None):
    pcd = o3d.geometry.PointCloud()
    if index is None:
        pcd.points = o3d.utility.Vector3dVector(X)
    else:
        pcd.points = o3d.utility.Vector3dVector(X[index])

    if colmat is not None:
        if index is None:
            pcd.colors = o3d.utility.Vector3dVector(colmat)
        else:
            pcd.colors = o3d.utility.Vector3dVector(colmat[index])


    elif colvec is not None:
        colmat=np.ones((len(colvec),3))
        colmat=colmat*colvec.reshape(-1,1)
        if index is None:
            pcd.colors = o3d.utility.Vector3dVector(colmat)
        else:
            pcd.colors = o3d.utility.Vector3dVector(colmat[index])
    return pcd

def pcd2points(pcd,inliners=None,outliers=None):
    Xv = np.array(pcd.points).copy()
    Cv = np.array(pcd.colors).copy()
    return Xv,Cv


def dbscan_cluster(pcd):
    pcd_out=copy.deepcopy(pcd)
    pcd_out = copy.deepcopy(pcd_out.voxel_down_sample(voxel_size=0.025))
    pcd_out.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
    pcd_out.orient_normals_consistent_tangent_plane(100)
    XX = np.hstack([np.array(pcd_out.points), np.array(pcd_out.normals)])
    clustering = DBSCAN(eps=0.1, min_samples=25).fit(XX)
    # clustering = AffinityPropagation().fit(XX)
    # clustering = AgglomerativeClustering(n_clusters=10).fit(XX)
    # clustering = KMeans(n_clusters=20, random_state=0).fit(XX)
    labels = clustering.labels_

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pcd_out.colors = o3d.utility.Vector3dVector(colors[:, :3])

    return pcd_out,labels

def extractCheckerboard_from_clusters(pcd,pcd_clustered,labels,dsquare):
    pcd = copy.deepcopy(pcd)
    pcd_clustered=copy.deepcopy(pcd_clustered)

    max_label = labels.max()
    L = np.arange(len(labels), dtype=int)
    for i in range(max_label):
        print(i, max_label)
        inlier_cloud = pcd_clustered.select_by_index(L[labels == i])

        bbx3d = inlier_cloud.get_axis_aligned_bounding_box()
        bbx3d_scaled = bbx3d.scale(1.2, bbx3d.get_center())
        insidebox_inds = bbx3d_scaled.get_point_indices_within_bounding_box(pcd.points)
        chess_cloud = pcd.select_by_index(insidebox_inds)

        plane_model, inliers2 = chess_cloud.segment_plane(distance_threshold=0.02,
                                                          ransac_n=3,
                                                          num_iterations=1000)

        chess_cloud_inlier = chess_cloud.select_by_index(inliers2)
        if chess_cloud_inlier.get_oriented_bounding_box().volume() > 0.2:
            continue
        Xchess,Colchess = pcd2points(chess_cloud_inlier, inliners=None, outliers=None)

        # Xchess = np.array(chess_cloud_inlier.points).copy()
        orig = np.mean(Xchess, axis=0)
        kdt = KDTree(Xchess)

        _, x0ind = kdt.query(orig, k=1)
        x0 = Xchess[x0ind]
        x1ind = kdt.query_ball_point(x0, 0.25)[10]
        x1 = Xchess[x1ind]

        [a, b, c, d] = plane_model
        nbar = [a, b, c]
        nbar = nbar / np.linalg.norm(nbar)
        zbar = [0, 0, 1]
        vbar = np.cross(zbar, nbar)
        theta = np.arccos(np.dot(nbar, zbar))
        rotmat1 = RR.from_rotvec(-theta * vbar).as_matrix()
        rotmat2 = RR.from_rotvec(0 * np.pi / 180 * np.array([0, 0, 1])).as_matrix()
        Xchess_trans = Xchess - x0
        Xchess_rot = np.matmul(rotmat2, rotmat1).dot(Xchess_trans.T).T


        cx, bins = np.histogram(Colchess[:, 0], bins=300)
        ind = np.argsort(cx)
        thresh_int = np.mean(bins[ind][-5])

        # cc=Colchess.copy()
        # cc[cc > thresh_int] = 1
        # cc[cc < thresh_int] = 0.1

        Xwhite = Xchess_rot[Colchess[:, 0] > thresh_int, :]
        Xblack = Xchess_rot[Colchess[:, 0] < thresh_int, :]

        try:
            blackclustering = AgglomerativeClustering(n_clusters=15, linkage='ward').fit(Xblack)
        except:
            continue
        centers = []
        for ll in range(15):
            centers.append(np.mean(Xblack[blackclustering.labels_ == ll, :], axis=0))
        centers = np.array(centers)

        kdt2 = KDTree(centers)
        D, x0ind = kdt2.query(centers, k=2)
        D = np.max(D, axis=1)
        if np.max(D) - np.min(D) > 0.05:
            pass
            continue
        D, x0ind = kdt2.query(centers, k=5)
        tt = 0.175 * (np.sqrt(2) + 2) / 2
        D = D[:, 1:]
        i1 = D < tt
        i2 = D > tt
        D[i1] = 1
        D[i2] = 0
        D = np.sum(D, axis=1)
        leftcorners = centers[D == 1]
        if len(leftcorners) != 2:
            continue
            pass
        diff = leftcorners[0] - leftcorners[1]
        theta2 = np.arctan2(diff[1], diff[0])

        rotmat2 = RR.from_rotvec(-(theta2 + np.pi / 2) * np.array([0, 0, 1])).as_matrix()
        leftcorners_aligned = rotmat2.dot(leftcorners.T).T
        if np.all(leftcorners_aligned[:, 0] > 0):
            rotmat2 = RR.from_rotvec(-(theta2 - np.pi / 2) * np.array([0, 0, 1])).as_matrix()
            leftcorners_aligned = rotmat2.dot(leftcorners.T).T


        centers_aligned = rotmat2.dot(centers.T).T
        # plt.plot(centers_aligned[:,0],centers_aligned[:,1],'r*')
        if leftcorners_aligned[0, 1] < leftcorners_aligned[1, 1]:
            bottom_left_center = leftcorners_aligned[0]
        else:
            bottom_left_center = leftcorners_aligned[1]
        bottom_left = bottom_left_center + [dsquare / 2, dsquare / 2, 0]
        objp_checker = np.zeros((5 * 4, 3), np.float32)
        objp_checker[:, :2] = np.mgrid[0:5, 0:4].T.reshape(-1, 2, order='F')
        objp_checker = objp_checker * dsquare
        objp_checker += bottom_left

        # now reversing the points
        objp_checker = np.linalg.inv(np.matmul(rotmat2, rotmat1)).dot(objp_checker.T).T
        objp_checker = objp_checker + x0

        colobj = np.zeros((objp_checker.shape[0], 3))
        colobj[:, 0] = 1
        colobj[0] = [0, 1, 0]
        colobj[1] = [0, 0, 1]
        colobj[4] = [1, 1, 0]
        pcdobjp_checker=points2pcd(objp_checker, index=None, colmat=colobj, colvec=None)
        plotpcds([pcdobjp_checker,pcd])


        break

    return objp_checker
def plotpcds(pcds):
    o3d.visualization.draw_geometries(pcds+[ mesh_frame])
###############  Build Plance model ####################33
def fitplane():
    pcd_out = copy.deepcopy(pcd.voxel_down_sample(voxel_size=0.05))
    planes_pcds=[]
    while 1:
        print(len(planes_pcds))
        if len(pcd_out.points)<1000:
            break
        plane_model, inliers = pcd_out.segment_plane(distance_threshold=0.075,
                                                 ransac_n=3,
                                                 num_iterations=1000)

        [a, b, c, d] = plane_model
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

        inlier_cloud = pcd_out.select_by_index(inliers)
        planes_pcds.append(copy.deepcopy(inlier_cloud))
        inlier_cloud.paint_uniform_color([1.0, 0, 0])
        outlier_cloud = pcd_out.select_by_index(inliers, invert=True)

        o3d.visualization.draw_geometries([inlier_cloud,outlier_cloud,mesh_frame])
        pcd_out = outlier_cloud
    # red is x, green is y, blue is z










def project3Dpoints_on_imageplane():
    h=480
    w=640
    foc=600
    K=np.array([[foc,0,w//2],[0,foc,h//2],[0,0,1]])
    indf=(Xv[:,0]<0) & (Xv[:,2]<0)
    Xvfil = Xv[indf,:]
    Xvcolfil = Xvcol[indf,:]
    Hcamrot=Hmat([0,135,0],[0,0,0])
    Xc = Hcamrot[0:3,0:3].dot(Xvfil.T).T

    Xcfrust=K.dot(Xc.T).T
    Xcfrust=Xcfrust/Xcfrust[:,2].reshape(-1,1)
    img_vf = np.ones((h,w),dtype=int)*np.nan
    ind_f = (Xcfrust[:,0]>0) & (Xcfrust[:,0]<w-1) & (Xcfrust[:,1]>0) & (Xcfrust[:,1]<h-1)
    Xcinside = Xcfrust[ind_f,:].round().astype(int)
    Colcinside = Xvcolfil[ind_f,:]
    mx=Colcinside[:,0].max()
    mn=Colcinside[:,0].min()
    for i in range(len(Xcinside)):
        a=(Colcinside[i,0]-mn)/mx
        # a=0.1
        # if Colcinside[i,0]>0.010:
        #     a=1

        img_vf[Xcinside[i,1],Xcinside[i,0]]=np.round(255*a).astype(int)

    mask=np.zeros_like(img_vf,dtype=np.uint8)
    mask[np.isnan(img_vf)]=1
    img_vf[np.isnan(img_vf)]=0
    img_vf=img_vf.astype(np.uint8)
    dst = cv2.inpaint(img_vf,mask,3,cv2.INPAINT_TELEA)

    # dstblur = cv2.blur(dst,(5,5))
    # # dstblur = dst
    # ret,dstblur = cv2.threshold(dstblur,8,255,cv2.THRESH_BINARY)
    dstblur = cv2.medianBlur(dst,1)
    # ret,dstblur = cv2.threshold(img,127,255,cv2.THRESH_BINARY)
    dstblur = cv2.adaptiveThreshold(dstblur,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,21,5)
    # dstblur = cv2.adaptiveThreshold(dst,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)


    cv2.namedWindow('gg', cv2.WINDOW_NORMAL)

    while 1:
        cv2.imshow("gg", dstblur[165:265,231:327])
        lastkey = cv2.waitKey(10)
        if lastkey == 27 or lastkey == 113:
            break
            time.sleep(0.2)

    cv2.destroyAllWindows()

    ret, cor = cv2.findChessboardCorners(dstblur[165:265,231:327], (4, 5), cv2.CALIB_CB_FAST_CHECK  + cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FILTER_QUADS)
    print(ret)

    pcd2 = o3d.geometry.PointCloud()
    pcd2.points=o3d.utility.Vector3dVector(Xc)
    pcd2.colors = o3d.utility.Vector3dVector(Colcinside)
    o3d.visualization.draw_geometries([pcd2,mesh_frame])


