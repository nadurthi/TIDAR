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

# os.makedirs(os.path.join(path,'rgbd'),exist_ok=True)
# os.makedirs(os.path.join(path,'ply'),exist_ok=True)

def load_data_file(i,params,mode='coarse'):
    mesh = o3d.io.read_triangle_mesh(os.path.join(params['path'], params['fname']%i))
    pcd = o3d.geometry.PointCloud()
    pcd.points = mesh.vertices
    pcd.colors = mesh.vertex_colors


    print(":: Estimate normal with search radius %.3f." % params['radius_normal'])
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=params['radius_normal'], max_nn=30))

    if mode=='fine':
        pcd_down = pcd.voxel_down_sample(voxel_size=params['voxel_size_fine'])
    else:
        pcd_down = pcd.voxel_down_sample(voxel_size=params['voxel_size'])

    return pcd_down





def pairwise_registration(source, target,params):
    print("Apply point-to-plane ICP")


    # try:
    # icp_coarse = o3d.pipelines.registration.registration_colored_icp(
    #     source, target, params['voxel_size'],
    #     np.identity(4),
    #     o3d.pipelines.registration.
    #     TransformationEstimationForColoredICP(),
    #     o3d.pipelines.registration.ICPConvergenceCriteria(
    #         relative_fitness=1e-6,
    #         relative_rmse=1e-6,
    #         max_iteration=30))
    # except:
    #
    #
    icp_coarse = o3d.pipelines.registration.registration_icp(
        source, target, params['max_corr_dist_coarse'], np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane())

    icp_fine = o3d.pipelines.registration.registration_generalized_icp(
        source, target, params['max_corr_dist_fine'],
        icp_coarse.transformation,
        o3d.pipelines.registration.
        TransformationEstimationForGeneralizedICP(),
        o3d.pipelines.registration.ICPConvergenceCriteria()) #            relative_fitness=1e-6,
            # relative_rmse=1e-6,
            # max_iteration=50


    # icp_fine = o3d.pipelines.registration.registration_icp(
    #     source, target, max_correspondence_distance_fine,
    #     icp_coarse.transformation,
    #     o3d.pipelines.registration.TransformationEstimationPointToPlane())

    transformation_icp = icp_fine.transformation

    information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source, target, params['max_corr_dist_fine'],
        icp_fine.transformation)
    return transformation_icp, information_icp

def intermediate(s_pcd,target_id,params):
    t_pcd = load_data_file(target_id,params)
    transformation_icp, information_icp = pairwise_registration(s_pcd, t_pcd,params)
    return target_id,transformation_icp, information_icp
def full_registration(n0_pcd,nf_pcd, params):
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    G = nx.Graph()
    # n_pcds = len(pcds)
    D={}
    for source_id in range(n0_pcd,nf_pcd):
        s_pcd=load_data_file(source_id,params)
        out = Parallel(n_jobs=5, prefer="threads")(delayed(intermediate)(s_pcd,target_id,params) for target_id in range(n0_pcd,nf_pcd))
        if source_id not in G.nodes:
            G.add_node(source_id)

        for ii in range(len(out)):
            # t_pcd = load_data_file(target_id, voxel_size=voxel_size)
            # transformation_icp, information_icp = pairwise_registration(
            #     s_pcd, t_pcd)
            #
            target_id, transformation_icp, information_icp = out[ii]
            print(source_id,target_id)
            if target_id not in G.nodes:
                G.add_node(target_id)

            # tOg: Odomentry from ground to target
            # sOg: Odomentry from ground to source
            # Ps== gOs: pose of source
            # Pt== gOt: pose of target .... as Pt[0,0,0,1] with directly give you the vehicle position
            # Ps== gOs: pose of source .... as Ps[0,0,0,1] with directly give you the vehicle position
            # tOg = tRs sOg ... where tRs is source to target computed fromsource to target

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
    parser.add_argument('--n0', type=int, required=False)
    parser.add_argument('--nf', type=int, required=False)
    args = parser.parse_args()

    voxel_size = 0.01
    Nfrag = 1
    fragstep = 'step_%05d' % Nfrag

    params = {
        'fragmentfolder': '/media/na0043/misc/DATA/ship/color_image_bgra_files/registered_fragments/%s' % fragstep,
        'fname': 'data_%d.ply',
        'path' : '/media/na0043/misc/DATA/ship/color_image_bgra_files/ply',
                 
        'max_corr_dist_fine': 0.1,
        'max_corr_dist_coarse': 0.7,
        'voxel_size': voxel_size,
        'voxel_size_fine': 0.000001,
        'radius_normal': voxel_size * 5,
        'radius_feature': voxel_size * 5,
    }

    os.makedirs(params['fragmentfolder'], exist_ok=True)

    if args.n0 is None or args.nf is None:
        n0_pcd_main=0
        nf_pcd_main=len(os.listdir(params['path']))
    else:
        n0_pcd_main = args.n0
        nf_pcd_main = args.nf
    cnt=0
    for jj in range(n0_pcd_main,nf_pcd_main,5):
        n0_pcd=jj
        nf_pcd=jj+5
        with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
            pose_graph,G = full_registration(n0_pcd,nf_pcd,params)



        print("Optimizing PoseGraph ...")
        option = o3d.pipelines.registration.GlobalOptimizationOption(
            max_correspondence_distance=params['max_corr_dist_fine'],
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

        for point_id in range(n0_pcd,nf_pcd):
            s_pcd = load_data_file(point_id,params,mode='fine')
            s_pcd.transform(pose_graph2.nodes[point_id-n0_pcd].pose)
            pcd_combined += s_pcd
            if (point_id-n0_pcd)%10==0:
                pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=params['voxel_size_fine'])
        pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=params['voxel_size_fine'])

        # o3d.visualization.draw_geometries([pcd_combined_down])

        pgraph_file = os.path.join(params['fragmentfolder'],"fragmentGraph_%05d.json"%(cnt,))
        o3d.io.write_pose_graph(pgraph_file,pose_graph)

        pgraphnetworkx_file = os.path.join(params['fragmentfolder'], "fragmentGraphNx_%05d.pkl" % (cnt,))
        with open(pgraphnetworkx_file,'wb') as F:
            pkl.dump(G,F)

        pcdcomb_file = os.path.join(params['fragmentfolder'],"fragmentPCD_%05d.pcd"%(cnt,))
        o3d.io.write_point_cloud(pcdcomb_file,pcd_combined_down)


        vis = o3d.visualization.Visualizer()
        vis.create_window(visible=True) #works for me with False, on some systems needs to be true
        vis.add_geometry(pcd_combined_down)
        vis.update_geometry(pcd_combined_down)
        vis.poll_events()
        vis.update_renderer()
        pngcomb_file = os.path.join(params['fragmentfolder'], "fragmentSnapShot_%05d.png" % (cnt, ))
        vis.capture_screen_image(pngcomb_file)
        vis.destroy_window()

        cnt+=1
