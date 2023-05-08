#!/usr/bin/env python3
__author__ = "Laurence Roberts-Elliott"

import random
import rosbag
import json
import cv2
import numpy as np
import matplotlib.pyplot as plt
from rospkg import RosPack

random.seed(0)

# Get path of bag files
bag_path = RosPack().get_path('stage_soil_mapping_mrs') + '/bags/'

bag = rosbag.Bag(bag_path+'3_robs_RR_dynamic_4.bag')

# Get timestamp of '/sim_time_initialized' message
for topic, msg, t in bag.read_messages():
    if 'sim_initialized' in topic:
        print("Sim initialized at: " + str(t))
        start_time = t
        break

robots = {} # dictionary to store robot trajectories
kriging_interpolations = [] # List of dicts to store kriging interpolations {interpolation: np.array, time: int}
kriging_variances = [] # List of dicts to store kriging variances {variance: np.array, time: int}

for topic, msg, t in bag.read_messages():
    kriging_interpolation_dict = {}
    kriging_variance_dict = {}

    # Get robot trajectories
    if 'amcl_pose' in topic and t > start_time:
        robot_name = topic.split('/')[1]
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
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

    # Get kriging interpolation, converting from OccupancyGrid to numpy array
    if 'interpolated_map' in topic and t > start_time:
        kriging_interpolation_dict['time'] = t
        numpy_array = np.array(msg.data).reshape(msg.info.height, msg.info.width)

        # Shrink the numpy array back to the original size
        numpy_array = cv2.resize(numpy_array, (int(numpy_array.shape[1]/20), int(numpy_array.shape[0]/20)), interpolation=cv2.INTER_NEAREST)

        kriging_interpolation_dict['interpolation'] = numpy_array

    # Get kriging variance, converting from OccupancyGrid to numpy array
    if 'kriging_variance' in topic and t > start_time:
        kriging_variance_dict['time'] = t
        numpy_array = np.array(msg.data).reshape(msg.info.height, msg.info.width)

        # Shrink the numpy array back to the original size
        numpy_array = cv2.resize(numpy_array, (int(numpy_array.shape[1]/20), int(numpy_array.shape[0]/20)), interpolation=cv2.INTER_NEAREST)

        kriging_variance_dict['variance'] = numpy_array


    # If kriging_interpolation_dict is not empty, append it to the list
    if kriging_interpolation_dict:
        kriging_interpolations.append(kriging_interpolation_dict)

    # If kriging_variance_dict is not empty, append it to the list
    if kriging_variance_dict:
        kriging_variances.append(kriging_variance_dict)
    

bag.close()

print("Number of kriging interpolations: " + str(len(kriging_interpolations)))
print("Kriging interpolations:\n" + str(kriging_interpolations))

# plot the trajectories of multiple robots in randomly generated colors
for robot_name, data in robots.items():
    color = data['color']
    plt.plot(data['x'], data['y'], color=color, label=robot_name)  
    plt.scatter(data['samples_x'], data['samples_y'], color=color, marker='x')
    plt.title('Robot Trajectories,\nSample Positions,\nand Kriging Interpolation')
    interpolation = kriging_interpolations[-1]['interpolation']
    interpolation = np.fliplr(interpolation)
    plt.imshow(interpolation, cmap='gray', origin='lower')

plt.colorbar()
plt.legend()
plt.show()
plt.close()

# Repeat but with kriging variance
for robot_name, data in robots.items():
    color = data['color']
    plt.plot(data['x'], data['y'], color=color, label=robot_name)
    plt.scatter(data['samples_x'], data['samples_y'], color=color, marker='x')
    plt.title('Robot Trajectories,\nSample Positions,\nand Kriging Variance')
    plt.imshow(kriging_variances[-1]['variance'], cmap='gray', origin='lower')

plt.colorbar()
plt.legend()
plt.show()
plt.close()
