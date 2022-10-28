#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Sep  6 10:40:16 2022

@author: nvidiaorin
"""
import open3d as o3d
import numpy as np
import copy
import time
import argparse
import os
import pickle as pkl
from joblib import Parallel, delayed
path = 'sampleset1'
import networkx as nx

os.makedirs(os.path.join(path,'registered_fragments'),exist_ok=True)

# os.makedirs(os.path.join(path,'rgbd'),exist_ok=True)
# os.makedirs(os.path.join(path,'ply'),exist_ok=True)

def load_ply_file(i,voxel_size=0.0):
    X=np.load(os.path.join(path,'velodyne','velodyne_%d.npy'%i))
#    pcd = o3d.io.read_point_cloud(os.path.join(path,'velodyne','velodyne_%d.pcd'%i))
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(X[:,:3])  
    
    radius_normal = voxel_size * 5
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)

    return pcd_down

def load_point_clouds(voxel_size=0.0):
    pcds = []
#    demo_icp_pcds = o3d.data.DemoICPPointClouds()
    for i in range(1,25):
        pcd_down = load_ply_file(i,voxel_size=voxel_size)

        pcds.append(pcd_down)
        
#    for path in demo_icp_pcds.paths:
#        pcd = o3d.io.read_point_cloud(path)
#        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
#        pcds.append(pcd_down)
    return pcds




def pairwise_registration(source, target):
    print("Apply point-to-plane ICP")

    icp_coarse = o3d.pipelines.registration.registration_icp(
         source, target, max_correspondence_distance_coarse, np.identity(4),
         o3d.pipelines.registration.TransformationEstimationPointToPlane())

#    icp_coarse = o3d.pipelines.registration.registration_colored_icp(
#        source, target, voxel_size,
#        np.identity(4),
#        o3d.pipelines.registration.
#        TransformationEstimationForColoredICP(),
#        o3d.pipelines.registration.ICPConvergenceCriteria(
#            relative_fitness=1e-6,
#            relative_rmse=1e-6,
#            max_iteration=30))

    icp_fine = o3d.pipelines.registration.registration_generalized_icp(
        source, target, max_correspondence_distance_fine,
        icp_coarse.transformation,
        o3d.pipelines.registration.
        TransformationEstimationForGeneralizedICP(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness=1e-6,
            relative_rmse=1e-6,
            max_iteration=30))


    # icp_fine = o3d.pipelines.registration.registration_icp(
    #     source, target, max_correspondence_distance_fine,
    #     icp_coarse.transformation,
    #     o3d.pipelines.registration.TransformationEstimationPointToPlane())

    transformation_icp = icp_fine.transformation

    information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source, target, max_correspondence_distance_fine,
        icp_fine.transformation)
    return transformation_icp, information_icp

def intermediate(s_pcd,target_id):
    t_pcd = load_ply_file(target_id, voxel_size=voxel_size)
    transformation_icp, information_icp = pairwise_registration(
        s_pcd, t_pcd)
    return target_id,transformation_icp, information_icp
def full_registration(n0_pcd,nf_pcd, max_correspondence_distance_coarse,
                      max_correspondence_distance_fine):
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    G = nx.Graph()
    # n_pcds = len(pcds)
    D={}
    for source_id in range(n0_pcd,nf_pcd):
        s_pcd=load_ply_file(source_id,voxel_size=voxel_size)
        out = Parallel(n_jobs=5, prefer="threads")(delayed(intermediate)(s_pcd,target_id) for target_id in range(max(n0_pcd,source_id-5),min(source_id+5,nf_pcd)))
        if source_id not in G.nodes:
            G.add_node(source_id)

        for ii in range(len(out)):
            # t_pcd = load_ply_file(target_id, voxel_size=voxel_size)
            # transformation_icp, information_icp = pairwise_registration(
            #     s_pcd, t_pcd)
            #
            target_id, transformation_icp, information_icp = out[ii]
            print(source_id,target_id)
            if target_id not in G.nodes:
                G.add_node(target_id)

            # print("Build o3d.pipelines.registration.PoseGraph")
            if target_id == source_id + 1:  # odometry case
                odometry = np.dot(transformation_icp, odometry)
                pose_graph.nodes.append(
                    o3d.pipelines.registration.PoseGraphNode(
                        np.linalg.inv(odometry)))
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id-n0_pcd,
                                                             target_id-n0_pcd,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=False))
                G.nodes[target_id]['odometry']=np.linalg.inv(odometry)

            else:  # loop closure case
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id-n0_pcd,
                                                             target_id-n0_pcd,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=True))
            G.add_edge(source_id, target_id, transformation_icp=transformation_icp, information_icp=information_icp)

    return pose_graph,G


if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--n0', type=int, required=True)
    parser.add_argument('--nf', type=int, required=True)
    args = parser.parse_args()
    
    voxel_size_fine=0.01
    
    voxel_size = 0.01
    max_correspondence_distance_coarse = 1
    max_correspondence_distance_fine = 0.6
    n0_pcd = args.n0
    nf_pcd = args.nf

    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        pose_graph,G = full_registration(n0_pcd,nf_pcd,
                                       max_correspondence_distance_coarse,
                                       max_correspondence_distance_fine)
    
    

    print("Optimizing PoseGraph ...")
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=max_correspondence_distance_fine,
        edge_prune_threshold=0.25,
        reference_node=0)
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        o3d.pipelines.registration.global_optimization(
            pose_graph,
            o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
            o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
            option)

    for point_id in range(n0_pcd, nf_pcd):
        G.nodes[point_id]["optimized_pose"] = pose_graph.nodes[point_id - n0_pcd].pose

    pose_graph_cp = copy.deepcopy(pose_graph)

    pose_graph2 = copy.deepcopy(pose_graph_cp)
    #%%combine
    # pcds = load_point_clouds(voxel_size=0.001)
    pcd_combined = o3d.geometry.PointCloud()
    NN=nf_pcd-n0_pcd
    if NN>50:
        vxpt=list(np.round(np.linspace(n0_pcd,nf_pcd,10)))
    else:
        vxpt=[nf_pcd-1]
    for point_id in range(n0_pcd,nf_pcd):
        s_pcd = load_ply_file(point_id, voxel_size=voxel_size_fine)
        s_pcd.transform(pose_graph2.nodes[point_id-n0_pcd].pose)
        pcd_combined += s_pcd
        if point_id in vxpt:
            pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=voxel_size_fine)
    o3d.visualization.draw_geometries([pcd_combined_down])

    pgraph_file = os.path.join(path,'registered_fragments',"PoseGraphShip_%d_%d.json"%(n0_pcd,nf_pcd))
    o3d.io.write_pose_graph(pgraph_file,pose_graph)

    pgraphnetworkx_file = os.path.join(path, 'registered_fragments', "PoseGraphShipPickle_%d_%d.pkl" % (n0_pcd, nf_pcd))
    with open(pgraphnetworkx_file,'wb') as F:
        pkl.dump(G,F)

    pcdcomb_file = os.path.join(path,'registered_fragments',"PCDShip_%d_%d.pcd"%(n0_pcd,nf_pcd))
    o3d.io.write_point_cloud(pcdcomb_file,pcd_combined_down)


    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=True) #works for me with False, on some systems needs to be true
    vis.add_geometry(pcd_combined_down)
    vis.update_geometry(pcd_combined_down)
    vis.poll_events()
    vis.update_renderer()
    pngcomb_file = os.path.join(path, 'registered_fragments', "PCDShipImage_%d_%d.png" % (n0_pcd, nf_pcd))
    vis.capture_screen_image(pngcomb_file)
    vis.destroy_window()
    
    
    r=0.089
    thy=
    pcd1=load_ply_file(400,voxel_size=0.01)
    n1=np.asarray(pcd1.points).shape
    pcd1.paint_uniform_color([1,0,0])
    
    pcd2=load_ply_file(501,voxel_size=0.01)
    n2=np.asarray(pcd1.points).shape
    pcd2.paint_uniform_color([0,1,0])
    
    o3d.visualization.draw_geometries([pcd1,pcd2])
    
    icp_fine = o3d.pipelines.registration.registration_generalized_icp(
        pcd2, pcd1, 0.1,
        np.identity(4),
        o3d.pipelines.registration.
        TransformationEstimationForGeneralizedICP(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness=1e-6,
            relative_rmse=1e-6,
            max_iteration=170))
    
    pcd3=pcd2.transform(icp_fine.transformation)
    
    o3d.visualization.draw_geometries([pcd1,pcd3])