
import cv2
import open3d as o3d
import numpy as np


def points2pcd(X,colmat=None,colvec=None):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(X)
    if colmat is not None:
        pcd.colors = o3d.utility.Vector3dVector(colmat)
    elif colvec is not None:
        colmat=np.ones((len(colvec),3))
        colmat=colmat*colvec.reshape(-1,1)
        pcd.colors = o3d.utility.Vector3dVector(colmat)
    return pcd

def pcd2points(pcd):
    Xv = np.array(pcd.points).copy()
    Cv = np.array(pcd.colors).copy()
    return Xv,Cv

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


    `cv2.namedWindow('gg', cv2.WINDOW_NORMAL)

    while 1:
        cv2.imshow("gg", dstblur[165:265,231:327])
        lastkey = cv2.waitKey(10)
        if lastkey == 27 or lastkey == 113:
            break
            time.sleep(0.2)

    cv2.destroyAllWindows()`

    ret, cor = cv2.findChessboardCorners(dstblur[165:265,231:327], (4, 5), cv2.CALIB_CB_FAST_CHECK  + cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FILTER_QUADS)
    print(ret)

    pcd2 = o3d.geometry.PointCloud()
    pcd2.points=o3d.utility.Vector3dVector(Xc)
    pcd2.colors = o3d.utility.Vector3dVector(Colcinside)
    o3d.visualization.draw_geometries([pcd2,mesh_frame])


