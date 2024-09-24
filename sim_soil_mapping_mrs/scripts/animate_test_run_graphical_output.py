#!/usr/bin/env python3
# This script is used to animate the ROSbags.
__author__ = "Laurence Roberts-Elliott"
# Import modules
import os
import glob
import imageio
from rospkg import RosPack
from animate_graphical_output import convert_to_gif


if __name__ == '__main__':
    # Get path of figures directory
    rp = RosPack()
    package_path = rp.get_path('sim_soil_mapping_mrs')

    figures_path = package_path + '/figures/test_run'
    print(figures_path)

    # Animate kriging interpolation
    path_to_png_files = figures_path + '/interpolation_raw/'

    png_files = glob.glob(path_to_png_files + '*.png')

    convert_to_gif(path_to_png_files,
                figures_path+'/interpolation_animation.gif')
    
    # Animate kriging variance
    path_to_png_files = figures_path + '/variance_raw/'

    png_files = glob.glob(path_to_png_files + '*.png')

    convert_to_gif(path_to_png_files,
                    figures_path+'/variance_animation.gif')