#!/usr/bin/env python3
# This script is used to animate the ROSbags.
__author__ = "Laurence Roberts-Elliott"
# Import modules
import os
import glob
import imageio
from rospkg import RosPack

# Convert png images into single gif
def convert_to_gif(path_to_png_files, gif_name, fps=.5, text_filter=None):
    """
    Converts png images into a single gif.
    :param path_to_png_files: Path to png files.
    :param gif_name: Name of gif file.
    :param fps: Frames per second.
    :return: None
    """

    # Get all png files in directory
    png_files = glob.glob(path_to_png_files + '*.png')

    # Sort png files
    png_files = sorted(png_files, key=lambda x: int(x.split('/')[-1].split('_')[0][:-7]))
    # print(png_files)

    # Create gif
    images = []
    for file_name in png_files:
        if text_filter is None:
            images.append(imageio.imread(file_name))
        elif text_filter in file_name:
            # print(file_name)
            images.append(imageio.imread(file_name))
    imageio.mimsave(gif_name, images, fps=fps)
    print("GIF saved to: " + gif_name)


if __name__ == '__main__':
    # Get path of figures directory
    rp = RosPack()
    package_path = rp.get_path('stage_soil_mapping_mrs')
    
    for i in range(1, 11):

        figures_path = package_path + '/figures/distance_over_variance_with_insertion_use_queue_sorting_False_' + str(i)

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