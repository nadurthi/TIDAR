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

import networkx as nx

class PCDfpfh:
    def __init__(self,pcd,pcd_fpfh):
        self.pcd = pcd
        self.pcd_fpfh = pcd_fpfh

# os.makedirs(os.path.join(path,'rgbd'),exist_ok=True)
# os.makedirs(os.path.join(path,'ply'),exist_ok=True)



def load_data_file(i,params,mode='coarse'):
    pcd = o3d.io.read_point_cloud(os.path.join(params['fragmentfolder'], params['fname'] % i))

    if mode == 'fine':
        pcd_down = pcd.voxel_down_sample(voxel_size=params['voxel_size_fine'])
    else:
        pcd_down = pcd.voxel_down_sample(voxel_size=params['voxel_size'])

    print(":: Estimate normal with search radius %.3f." % params['radius_normal'])
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=params['radius_normal'], max_nn=30))


    # print(":: Compute FPFH feature with search radius %.3f." % params['radius_feature'])
    # pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
    #     pcd_down,
    #     o3d.geometry.KDTreeSearchParamHybrid(radius=params['radius_feature'], max_nn=100))


    return pcd_down
    # return PCDfpfh(pcd_down, pcd_fpfh)

def execute_fast_global_registration(source_down, target_down, params):

    print(":: Apply fast global registration with distance threshold %.3f" \
            % params['fast_global_distance_threshold'])
    result = o3d.pipelines.registration.registration_fast_based_on_feature_matching(
        source_down.pcd, target_down.pcd, source_down.pcd_fpfh, target_down.pcd_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=params['fast_global_distance_threshold'], iteration_number=30))
    return result

def pairwise_registration(source, target,params,T0=np.identity(4)):
    print("Apply point-to-plane ICP")

    # result=execute_fast_global_registration(source, target,params)
    icp_coarse = o3d.pipelines.registration.registration_icp(
        source, target, params['max_corr_dist_coarse'], T0,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())

    icp_fine = o3d.pipelines.registration.registration_generalized_icp(
        source, target, params['max_corr_dist_fine'],
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
        source, target, params['max_corr_dist_fine'],
        icp_fine.transformation)
    return transformation_icp, information_icp


def intermediate_seq(path,fname,s_pcd, source_id,params):
    target_id = source_id + 1
    t_pcd = load_data_file(target_id, params)
    transformation_icp, information_icp = pairwise_registration(s_pcd, t_pcd,params)
    return target_id, transformation_icp, information_icp


def seq_registration(n0_pcd, nf_pcd,params):
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    G = nx.Graph()

    for source_id in range(n0_pcd, nf_pcd):
        s_pcd = load_data_file(source_id, params)
        for target_id in [source_id + 1,source_id - 1]:
            if target_id<n0_pcd or target_id>=nf_pcd:
                continue
            t_pcd = load_data_file(target_id, params)
            transformation_icp, information_icp = pairwise_registration(s_pcd, t_pcd, params)


            if source_id not in G.nodes:
                G.add_node(source_id)
            if target_id not in G.nodes:
                G.add_node(target_id)

            print(source_id, target_id)
            # print("Build o3d.pipelines.registration.PoseGraph")
            if target_id == source_id + 1:  # odometry case
                odometry = np.dot(transformation_icp, odometry)
                pose_graph.nodes.append(
                    o3d.pipelines.registration.PoseGraphNode(
                        np.linalg.inv(odometry)))
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id - n0_pcd,
                                                             target_id - n0_pcd,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=False))
                G.nodes[target_id]['odometry'] = np.linalg.inv(odometry)

            else:  # loop closure case
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id - n0_pcd,
                                                             target_id - n0_pcd,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=True))
            G.add_edge(source_id, target_id, transformation_icp=transformation_icp, information_icp=information_icp)

    return pose_graph, G

def FULL_registration(n0_pcd, nf_pcd,pose_graph,G,params):


    for source_id in range(n0_pcd, nf_pcd):
        s_pcd = load_data_file(source_id, params)
        for target_id in range(n0_pcd, nf_pcd):
            if target_id in [source_id + 1,source_id - 1]:
                continue

            t_pcd = load_data_file(target_id, params)
            if 'optimized_pose' in G.nodes[target_id]:
                Tt = G.nodes[target_id]['optimized_pose']
            else:
                Tt = G.nodes[target_id]['odometry']

            if 'optimized_pose' in G.nodes[source_id]:
                Ts = G.nodes[source_id]['optimized_pose']
            else:
                Ts = G.nodes[source_id]['odometry']

            T0 = np.matmul(np.linalg.inv(Tt),Ts)

            transformation_icp, information_icp = pairwise_registration(s_pcd, t_pcd, params,T0=T0)



            print(source_id, target_id)
            # print("Build o3d.pipelines.registration.PoseGraph")

            pose_graph.edges.append(
                o3d.pipelines.registration.PoseGraphEdge(source_id - n0_pcd,
                                                         target_id - n0_pcd,
                                                         transformation_icp,
                                                         information_icp,
                                                         uncertain=True))
            G.add_edge(source_id, target_id, transformation_icp=transformation_icp, information_icp=information_icp)

    return pose_graph, G


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--n0', type=int, required=False)
    parser.add_argument('--nf', type=int, required=False)
    args = parser.parse_args()

    Nfrag = 1
    fragstep = 'step_%05d' % Nfrag
    nextfragstep = 'step_%05d' % (Nfrag + 1)

    voxel_size = 0.01
    params = {
        'fragmentfolder': '/media/na0043/misc/DATA/ship/color_image_bgra_files/registered_fragments/%s' % fragstep,
        'fname': 'fragmentPCD_%05d.pcd',
        'fragmentsNextfolder' : '/media/na0043/misc/DATA/ship/color_image_bgra_files/registered_fragments/%s' % nextfragstep,

        'fast_global_distance_threshold': voxel_size * 0.5,
        'max_corr_dist_fine': 0.1,
        'max_corr_dist_coarse': 0.7,
        'voxel_size': voxel_size,
        'voxel_size_fine': 0.001,
        'radius_normal': voxel_size * 3,
        'radius_feature': voxel_size * 5,
    }


    os.makedirs(params['fragmentsNextfolder'], exist_ok=True)

    n0_pcd = 0
    nf_pcd = len([f for f in os.listdir(params['fragmentfolder']) if '.pcd' in f])

    # ---- seq registration
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        pose_graph,G = seq_registration(n0_pcd,nf_pcd,params)

    print("Optimizing PoseGraph ...")
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=params['max_corr_dist_fine'],
        edge_prune_threshold=0.25,
        reference_node=0)
    # with o3d.utility.VerbosityContextManager(
    #         o3d.utility.VerbosityLevel.Debug) as cm:
    o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option)

    for point_id in range(n0_pcd, nf_pcd):
        G.nodes[point_id]["optimized_pose"] = pose_graph.nodes[point_id - n0_pcd].pose

    # ---- full registration
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        pose_graph,G = FULL_registration(n0_pcd, nf_pcd, pose_graph, G, params)

    print("Optimizing PoseGraph ...")
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=params['max_corr_dist_fine'],
        edge_prune_threshold=0.25,
        reference_node=0)
    # with o3d.utility.VerbosityContextManager(
    #         o3d.utility.VerbosityLevel.Debug) as cm:
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

    for point_id in range(n0_pcd,nf_pcd):
        s_pcd = load_data_file(point_id,params,mode='fine')
        s_pcd.transform(pose_graph2.nodes[point_id-n0_pcd].pose)
        pcd_combined += s_pcd
        if (point_id-n0_pcd)%10==0:
            pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=params['voxel_size_fine'])
    pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=params['voxel_size_fine'])

    o3d.visualization.draw_geometries([pcd_combined_down])

    pgraph_file = os.path.join(params['fragmentsNextfolder'],"FullGraph_%d_%d.json"%(n0_pcd,nf_pcd))
    o3d.io.write_pose_graph(pgraph_file,pose_graph)

    pgraphnetworkx_file = os.path.join(params['fragmentsNextfolder'], "fragmentGraphNx_%d_%d.pkl" % (n0_pcd, nf_pcd))
    with open(pgraphnetworkx_file,'wb') as F:
        pkl.dump(G,F)

    pcdcomb_file = os.path.join(params['fragmentsNextfolder'],"fragmentPCD_%d_%d.pcd"%(n0_pcd,nf_pcd))
    o3d.io.write_point_cloud(pcdcomb_file,pcd_combined_down)


    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=True) #works for me with False, on some systems needs to be true
    vis.add_geometry(pcd_combined_down)
    vis.update_geometry(pcd_combined_down)
    vis.poll_events()
    vis.update_renderer()
    pngcomb_file = os.path.join(params['fragmentsNextfolder'], "fragmentSnapShot_%d_%d.png" % (n0_pcd, nf_pcd))
    vis.capture_screen_image(pngcomb_file)
    vis.destroy_window()