#!/usr/bin/env python3
__author__ = "Laurence Roberts-Elliott"

import bagpy
from bagpy import bagreader
import pandas as pd
import numpy as np
from rospkg import RosPack

if __name__ == '__main__':
    # Get the path to the bag file
    rp = RosPack()
    bag_path = rp.get_path('stage_soil_mapping_mrs') + '/bags/metrics.bag'
    bag_folder_path = rp.get_path('stage_soil_mapping_mrs') + '/bags/'

    # Read the bag file
    b = bagreader(bag_path)

    # Check topics
    topic_table = b.topic_table
    print(topic_table)

    # Get last step kriging RMSE
    rmse_msg = b.message_by_topic('/coordinator/RMSE')
    rmse_df = pd.read_csv(rmse_msg)
    print("Last step kriging RMSE: " + str(rmse_df.iloc[-1].data))

    # Get last step mean kriging variance
    avgVar_msg = b.message_by_topic('/coordinator/avgVar')
    avgVar_df = pd.read_csv(avgVar_msg)
    print("Last step mean kriging variance: " + str(avgVar_df.iloc[-1].data))

    # Get robot names from topic table
    robot_names = []
    for i in range(len(topic_table)):
        if "robot_" in topic_table.iloc[i].Topics:
            robot_names.append(topic_table.iloc[i].Topics.split("/")[1])
    robot_names = np.unique(robot_names)

    # Print per-robot metrics
    for name in robot_names:
        print(name + " metrics:")

        # Get last step robot total_distance
        total_distance_msg = b.message_by_topic('/' + name + '/metrics/total_distance')
        total_distance_df = pd.read_csv(total_distance_msg)
        print("Last step total distance: " + str(total_distance_df.iloc[-1].data))

        # Get last step robot total_idle_time
        total_idle_time_msg = b.message_by_topic('/' + name + '/metrics/total_idle_time')
        total_idle_time_df = pd.read_csv(total_idle_time_msg)
        print("Last step total idle time: " + str(total_idle_time_df.iloc[-1].data))

        # Get last step robot mean_task_completion_time
        mean_task_completion_time_msg = b.message_by_topic('/' + name + '/metrics/mean_task_completion_time')
        mean_task_completion_time_df = pd.read_csv(mean_task_completion_time_msg)
        print("Last step mean task completion time: " + str(mean_task_completion_time_df.iloc[-1].data))

    # Get global mean of per-robot metrics
    global_total_distance = 0
    global_total_idle_time = 0
    global_total_task_completion_time = 0

    for name in robot_names:
        # Get last step robot total_distance
        total_distance_msg = b.message_by_topic('/' + name + '/metrics/total_distance')
        total_distance_df = pd.read_csv(total_distance_msg)
        global_total_distance += total_distance_df.iloc[-1].data

        # Get last step robot total_idle_time
        total_idle_time_msg = b.message_by_topic('/' + name + '/metrics/total_idle_time')
        total_idle_time_df = pd.read_csv(total_idle_time_msg)

        # Ignore value if 0 or NaN
        if total_idle_time_df.iloc[-1].data != 0 or not np.isnan(total_idle_time_df.iloc[-1].data):
            global_total_idle_time += total_idle_time_df.iloc[-1].data

        # Get last step robot mean_task_completion_time
        mean_task_completion_time_msg = b.message_by_topic('/' + name + '/metrics/mean_task_completion_time')
        mean_task_completion_time_df = pd.read_csv(mean_task_completion_time_msg)

        # Ignore value if 0 or NaN
        if mean_task_completion_time_df.iloc[-1].data != 0 or not np.isnan(mean_task_completion_time_df.iloc[-1].data):
            global_total_task_completion_time += mean_task_completion_time_df.iloc[-1].data

print("Global mean of per-robot metrics:")
print("Last step total distance: " + str(global_total_distance / len(robot_names)))
print("Last step mean idle time: " + str(global_total_idle_time / len(robot_names)))
print("Last step mean of task completion time: " + str(global_total_task_completion_time / len(robot_names)))


# Put last step metrics in a DataFrame
metrics_df = pd.DataFrame(columns=['Metric', 'Robot Name', 'Value'])

for name in robot_names:
    # Get last step robot total_distance
    total_distance_msg = b.message_by_topic('/' + name + '/metrics/total_distance')
    total_distance_df = pd.read_csv(total_distance_msg)
    metrics_df = metrics_df.append({'Metric': 'Total distance travelled (m)', 'Robot Name': name, 'Value': total_distance_df.iloc[-1].data}, ignore_index=True)

    # Get last step robot total_idle_time
    total_idle_time_msg = b.message_by_topic('/' + name + '/metrics/total_idle_time')
    total_idle_time_df = pd.read_csv(total_idle_time_msg)
    metrics_df = metrics_df.append({'Metric': 'Total idle time (s)', 'Robot Name': name, 'Value': total_idle_time_df.iloc[-1].data}, ignore_index=True)

    # Get last step robot mean_task_completion_time
    mean_task_completion_time_msg = b.message_by_topic('/' + name + '/metrics/mean_task_completion_time')
    mean_task_completion_time_df = pd.read_csv(mean_task_completion_time_msg)
    metrics_df = metrics_df.append({'Metric': 'Mean task completion time (s)', 'Robot Name': name, 'Value': mean_task_completion_time_df.iloc[-1].data}, ignore_index=True)

# Append last step kriging RMSE
rmse_msg = b.message_by_topic('/coordinator/RMSE')
rmse_df = pd.read_csv(rmse_msg)
metrics_df = metrics_df.append({'Metric': 'Kriging RMSE (Root Mean Squared Error)', 'Robot Name': "NA", 'Value': rmse_df.iloc[-1].data}, ignore_index=True)

# Append last step mean kriging variance
avgVar_msg = b.message_by_topic('/coordinator/avgVar')
avgVar_df = pd.read_csv(avgVar_msg)
metrics_df = metrics_df.append({'Metric': 'Mean kriging variance', 'Robot Name': "NA", 'Value': avgVar_df.iloc[-1].data}, ignore_index=True)

# Save metrics DataFrame to csv
metrics_df.to_csv(bag_folder_path + "/metrics.csv", index=False)