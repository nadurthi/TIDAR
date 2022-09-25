#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 24 20:09:26 2022

@author: nvidiaorin
"""
import configparser
config = configparser.ConfigParser()


with open('multicam_config.ini', 'w') as configfile:
    config.read(configfile)