#!/usr/bin/env python3
__author__ = "Laurence Roberts-Elliott"

import os
import pandas as pd
import numpy as np
from bagpy import bagreader
from rospkg import RosPack
from metrics_from_bag import metrics_from_bag

if __name__ == '__main__':
    # Get the path to the bag files folder
    rp = RosPack()
    bag_folder_path = rp.get_path('stage_soil_mapping_mrs') + '/bags'
    # bag_folder_path = "."

    bag_paths = []
    # Get the path to all .bag files in the folder
    for path in os.listdir(bag_folder_path):
        if path.endswith(".bag"):
            bag_paths.append(bag_folder_path + "/" + path)

    for bag_path in bag_paths:
        metrics_from_bag(bag_path, bag_folder_path)
