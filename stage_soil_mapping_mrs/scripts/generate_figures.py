#!/usr/bin/env python3
__author__ = "Laurence Roberts-Elliott"

import random
import rosbag
import matplotlib.pyplot as plt
from rospkg import RosPack

random.seed(0)

# Get path of bag files
bag_path = RosPack().get_path('stage_soil_mapping_mrs') + '/bags/'

bag = rosbag.Bag(bag_path+'metrics.bag')
robots = {} # dictionary to store robot trajectories

for topic, msg, t in bag.read_messages():
    if 'amcl_pose' in topic:
        robot_name = topic.split('/')[1]
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if robot_name not in robots:
            robots[robot_name] = {'x': [], 'y': [], 'color': (random.random(), random.random(), random.random())}
        robots[robot_name]['x'].append(x)
        robots[robot_name]['y'].append(y)

bag.close()

# plot the trajectories of multiple robots in randomly generated colors
for robot_name, data in robots.items():
    color = data['color']
    plt.plot(data['x'], data['y'], color=color, label=robot_name)

plt.legend()
plt.show()