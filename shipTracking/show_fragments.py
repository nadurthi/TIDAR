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



def load_data_file(i,params):

    pcd = o3d.io.read_point_cloud(os.path.join(params['fragmentfolder'], params['fname']%i))
    pcd_down = pcd.voxel_down_sample(voxel_size=params['voxel_size'])

    return pcd_down


if __name__ == "__main__":


    Nfrag = 1
    fragstep = 'step_%05d' % Nfrag


    voxel_size = 0.0001
    params = {
        'fragmentfolder': '/media/na0043/misc/DATA/ship/color_image_bgra_files/registered_fragments/%s' % fragstep,
        'fname': 'fragmentPCD_%05d.pcd',
        'fast_global_distance_threshold': voxel_size * 0.5,
        'max_corr_dist_gcp': 0.1,
        'max_corr_dist_icp': 0.7,
        'voxel_size': voxel_size,
        'voxel_size_fine': 0.001,
        'radius_normal': voxel_size * 3,
        'radius_feature': voxel_size * 5,
    }
    os.makedirs(os.path.join(params['fragmentfolder'],'removed'),exist_ok=True)
    import shutil
    ff=os.listdir(params['fragmentfolder'])
    for i in range(len(ff)):
        s_pcd = load_data_file(i, params)
        o3d.visualization.draw_geometries([s_pcd])
        r = input('remove this pcd: (d)= delete')
        if r=='d':
            shutil.move(os.path.join(params['fragmentfolder'], params['fname']%i),
                        os.path.join(params['fragmentfolder'],'removed' ,params['fname'] % i))
