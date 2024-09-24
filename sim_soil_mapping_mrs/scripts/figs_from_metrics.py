#!/usr/bin/env python3
__author__ = "Laurence Roberts-Elliott"

import os
import numpy as np
import matplotlib.pyplot as plt
from rospkg import RosPack

# Get path of bag files
rp = RosPack()
package_path = rp.get_path('sim_soil_mapping_mrs')
bag_folder_path = package_path + '/bags'

bag_paths = []
start_time = None

# Get the path to all .bag files in the folder
for path in os.listdir(bag_folder_path):
    if path.endswith(".bag"):
        # Strip '.bag' from end of path to get directory where metrics are stored
        metrics_dir = path[:-4]
        bag_paths.append(bag_folder_path + "/" + metrics_dir + "/")

# print(bag_paths)

# Plots metrics across time for each trial
for bag_path in bag_paths:
    print("Creating figures from metrics in " + bag_path + " directory...")

    condition_name = bag_path.split('/')[-2]
    print("Condition: " + condition_name)
    
    # Get csv files in directory
    csv_files = []
    for file in os.listdir(bag_path):
        if file.endswith(".csv"):
            csv_files.append(file)
    
    # Plot metric across time for each csv file
    for csv_file in csv_files:
        metric_name = csv_file[:-4]
        print("Plotting metric across time for " + metric_name + "...")
        data = np.genfromtxt(bag_path + csv_file, delimiter=',', names=True)
        # print(data.dtype.names)

        # Ignore values equal to -1
        data = data[data['data'] != -1]

        # If metric is RMSE or mean kriging variance, drop extremely high values
        if metric_name == "coordinator-RMSE" or metric_name == "coordinator-avgVar":
            data = data[data['data'] < 50]

        plt.plot(data['Time'], data['data'])
        plt.xlabel("Time (s)")
        plt.ylabel(metric_name)
        plt.title(metric_name + " over time")
        plt.savefig(package_path + '/figures/' + condition_name + '/' + metric_name + '_time.png')
        plt.close()
        # plt.show()

# Plot metrics across conditions for each metric to compare
metrics = [metric[:-4] for metric in csv_files]
print("Metrics: " + str(metrics))
legend = []
for metric in metrics:
    print("Plotting metric across conditions for " + metric + "...")
    csv_file = metric + ".csv"
    for bag_path in bag_paths:
        condition_name = bag_path.split('/')[-2]
        # print("Condition: " + condition_name)
        data = np.genfromtxt(bag_path + csv_file, delimiter=',', names=True)
        # print(data.dtype.names)

        # Ignore values equal to -1
        data = data[data['data'] != -1]

        # If metric is RMSE or mean kriging variance, drop extremely high values
        if metric == "coordinator-RMSE" or metric == "coordinator-avgVar":
            data = data[data['data'] < 50]

        # print(data)
        plt.plot(data['Time'], data['data'])
        legend.append(condition_name)
    plt.xlabel("Time (s)")
    plt.ylabel(metric)
    plt.legend(legend)
    plt.title(metric + " across conditions")
    plt.savefig(package_path + '/figures/' + metric + '_conditions.png')
    plt.close()
        # plt.show()


# Per-metric bar chart of last value across conditions
colors = ['b', 'g', 'r']
condition_names = [condition_name.split('/')[-2] for condition_name in bag_paths]
print("Conditions: " + str(condition_names))
conditions_shorthand = {
    "distance_over_variance_with_insertion_drop_low_var_tasks_True": "DOVTDCI",
    "euclidean_distance_with_insertion_drop_low_var_tasks_False": "EDCI",
    "euclidean_distance_with_insertion_drop_low_var_tasks_True": "EDCITD",
    "distance_over_variance_drop_low_var_tasks_True": "DOVTD"
}
for metric in metrics:
    print("Creating per-metric bar chart of last value for " + metric + "...")
    last_values = []
    for bag_path in bag_paths:
        condition_name = bag_path.split('/')[-2]
        # print("Condition: " + condition_name)
        data = np.genfromtxt(bag_path + metric + ".csv", delimiter=',', names=True)
        # print(data.dtype.names)

        # Ignore values equal to -1
        data = data[data['data'] != -1]

        last_values.append(data['data'][-1])
    # print(last_values)
    # condition_names = [condition_name.split('/')[-2] for condition_name in bag_paths]
    # print("Conditions: " + str(condition_names))

    condition_shorthand_ordered = [conditions_shorthand[condition[:-2]] for condition in condition_names]
    
    plt.bar(condition_shorthand_ordered, last_values, color=colors)
    plt.ylabel(metric)
    # plt.legend(condition_names)
    plt.title(metric + " last value across conditions")
    # plt.xticks(rotation=45, ha='right')

    plt.savefig(package_path + '/figures/' + metric + '_last_value.png', bbox_inches='tight')
    plt.close()
    # plt.show()