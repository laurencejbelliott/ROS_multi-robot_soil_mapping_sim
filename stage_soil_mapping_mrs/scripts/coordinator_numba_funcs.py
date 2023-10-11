#!/usr/bin/env python3
__author__ = "Laurence Roberts-Elliott"
import numpy as np
from numba import njit, prange

@njit
def calculate_path_cost(robot_position, task_position, robot_speed):
    # Calculate path cost
    # Euclidean distance between robot and task, multiplied by constant robot_speed
    # rospy.loginfo("Calculating path cost between " + str(robot_position) + " and " + str(task_position))
    return np.sqrt((robot_position[0] - task_position[0]) ** 2 + (robot_position[1] - task_position[1]) ** 2) * robot_speed

# @njit
def calculate_task_queue_cost(robot_position, task_queue, robot_speed):
    # Calculate task queue cost
    # Sum of path costs between robot and each task in task queue
    total_cost = 0
    # for task_ix in prange(len(task_queue)):
    for task_ix in range(len(task_queue)):
        task = task_queue[task_ix]
        total_cost = total_cost + calculate_path_cost(robot_position, task, robot_speed)
    return total_cost
