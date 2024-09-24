#!/usr/bin/env python3

import random
import rosbag
import json
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from rospkg import RosPack

random.seed(0)

# Get path of bag files
rp = RosPack()
figures_path = rp.get_path('sim_soil_mapping_mrs') + '/figures'

def figs_from_bag(bag_path):
    bag = rosbag.Bag(bag_path)
    bag_name = bag.filename.split('/')[-1].split('.')[0]

    # Get timestamp of '/sim_time_initialized' message
    for topic, msg, t in bag.read_messages():
        if 'sim_initialized' in topic:
            print("Sim initialized at: " + str(t))
            start_time = t
            break

    robots = {} # dictionary to store robot trajectories

    for topic, msg, t in bag.read_messages():
        kriging_interpolation_dict = {}
        kriging_variance_dict = {}

        # Get robot trajectories
        if 'robot_pose' in topic and t > start_time:
            robot_name = topic.split('/')[1]
            x = msg.pose.position.x
            y = msg.pose.position.y
            if robot_name not in robots:
                robots[robot_name] = {'x': [], 'y': [], 'color': (random.random(), random.random(), random.random())}
            robots[robot_name]['x'].append(x)
            robots[robot_name]['y'].append(y)

        # Get per-robot sampling positions
        if 'mock_pen_reading' in topic and t > start_time:
            robot_name = topic.split('/')[1]
            # Get x and y from message (encoded as JSON)
            data_json = msg

            data_json = str(data_json.data).replace("'", '"')
            data_json = json.loads(data_json)

            x = int(data_json["x"])
            y = int(data_json["y"])

            # Check if samples_x and samples_y exist for the robot, then append from message
            if robot_name not in robots:
                robots[robot_name] = {'x': [], 'y': [], 'color': (random.random(), random.random(), random.random())}
            if 'samples_x' not in robots[robot_name]:
                robots[robot_name]['samples_x'] = []
            if 'samples_y' not in robots[robot_name]:
                robots[robot_name]['samples_y'] = []
            robots[robot_name]['samples_x'].append(x)
            robots[robot_name]['samples_y'].append(y)

        # Get raw kriging interpolation, converting from Map msg to numpy array
        if 'interpolated_map_raw' in topic and t > start_time:
            kriging_interpolation_dict['time'] = t
            # Get rows from Map msg
            numpy_array = [row.data for row in msg.row]
            numpy_array = np.array(numpy_array)
            # print(numpy_array)
            # print(type(numpy_array))
            # print(numpy_array.shape)

            
            # # Shrink the numpy array back to the original size
            # numpy_array = cv2.resize(numpy_array, (int(numpy_array.shape[1]/20), int(numpy_array.shape[0]/20)), interpolation=cv2.INTER_NEAREST)

            kriging_interpolation_dict['interpolation_raw'] = numpy_array

            # Generate figure
            fig, ax = plt.subplots()
            ax.imshow(numpy_array, cmap='gray', origin='lower')

            # Calculate time since start in seconds
            time_since_start = np.round((t - start_time).to_sec())

            ax.set_title('Kriging Interpolation at Time: ' + str(time_since_start) + 's')
            fig.colorbar(ax.imshow(numpy_array, cmap='gray', origin='lower'), ax=ax)
            ax.set_xlabel('X')
            ax.set_ylabel('Y')

            # Plot robot trajectories
            for robot_name, robot_data in robots.items():
                ax.plot(robot_data['x'], robot_data['y'], color=robot_data['color'])

            # Plot sample positions
            for robot_name, robot_data in robots.items():
                ax.scatter(robot_data['samples_x'], robot_data['samples_y'], color=robot_data['color'], label=robot_name)

            ax.legend()
            # plt.show()
            if not os.path.exists(figures_path + '/' + bag_name):
                os.makedirs(figures_path + '/' + bag_name)
            if not os.path.exists(figures_path + '/' + bag_name + '/interpolation_raw'):
                os.makedirs(figures_path + '/' + bag_name + '/interpolation_raw')
            plt.savefig(figures_path + '/' + bag_name + '/interpolation_raw/' + str(time_since_start) + 's.png')
            plt.close(fig)

        # Get raw kriging variance, converting from Map msg to numpy array
        if 'kriging_variance_raw' in topic and t > start_time:
            kriging_variance_dict['time'] = t
            # Get rows from Map msg
            numpy_array = [row.data for row in msg.row]
            numpy_array = np.array(numpy_array)
            # print(numpy_array)
            # print(type(numpy_array))
            # print(numpy_array.shape)
            
            # numpy_array = np.array(msg.data).reshape(msg.info.height, msg.info.width)

            # # Shrink the numpy array back to the original size
            # numpy_array = cv2.resize(numpy_array, (int(numpy_array.shape[1]/20), int(numpy_array.shape[0]/20)), interpolation=cv2.INTER_NEAREST)

            kriging_variance_dict['variance_raw'] = numpy_array

            # Generate figure
            fig, ax = plt.subplots()
            ax.imshow(numpy_array, cmap='gray', origin='lower')

            # Calculate time since start in seconds
            time_since_start = np.round((t - start_time).to_sec())

            ax.set_title('Kriging Variance at Time: ' + str(time_since_start) + 's')
            fig.colorbar(ax.imshow(numpy_array, cmap='gray', origin='lower'), ax=ax)
            ax.set_xlabel('X')
            ax.set_ylabel('Y')

            # Plot robot trajectories
            for robot_name, robot_data in robots.items():
                ax.plot(robot_data['x'], robot_data['y'], color=robot_data['color'])

            # Plot sample positions
            for robot_name, robot_data in robots.items():
                ax.scatter(robot_data['samples_x'], robot_data['samples_y'], color=robot_data['color'], label=robot_name)

            ax.legend()
            # plt.show()
            if not os.path.exists(figures_path + '/' + bag_name):
                os.makedirs(figures_path + '/' + bag_name)
            if not os.path.exists(figures_path + '/' + bag_name + '/variance_raw'):
                os.makedirs(figures_path + '/' + bag_name + '/variance_raw')
            plt.savefig(figures_path + '/' + bag_name + '/variance_raw/' + str(time_since_start) + 's.png')
            plt.close(fig)
            

if __name__ == '__main__':
    bag_path = rp.get_path('sim_soil_mapping_mrs')+'/bags/metrics.bag'

    # Delete all figures (files ending with .png) in figures_path
    for root, dirs, files in os.walk(figures_path + '/interpolation_raw'):
        for file in files:
            if file.endswith('.png'):
                os.remove(os.path.join(root, file))

    for root, dirs, files in os.walk(figures_path + '/variance_raw'):
        for file in files:
            if file.endswith('.png'):
                os.remove(os.path.join(root, file))

    figs_from_bag(bag_path)