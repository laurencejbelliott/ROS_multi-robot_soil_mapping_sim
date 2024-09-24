#!/usr/bin/env python3
__author__ = "Laurence Roberts-Elliott"

import os
import random
import rosbag
import json
import cv2
import numpy as np
import matplotlib.pyplot as plt
from rospkg import RosPack
from figs_from_bag import figs_from_bag

random.seed(0)

# Get path of bag files
rp = RosPack()
bag_folder_path = rp.get_path('sim_soil_mapping_mrs') + '/bags'

bag_paths = []
start_time = None

# Get the path to all .bag files in the folder
for path in os.listdir(bag_folder_path):
    if path.endswith(".bag"):
        bag_paths.append(bag_folder_path + "/" + path)

for bag_path in bag_paths:
    print("Creating figures from: " + bag_path)
    figs_from_bag(bag_path)