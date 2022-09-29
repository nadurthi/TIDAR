#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 24 20:09:26 2022

@author: nvidiaorin
"""
import configparser
import stereocodes.SGMpython.gpu_library  as gpu_library

config = configparser.ConfigParser()

config.read_file(open('multicam_config.ini'))
config_dict={}
for k in config.keys():
    config_dict[k]=dict(config[k])
    
gpu_library.Algo_libsgm()
