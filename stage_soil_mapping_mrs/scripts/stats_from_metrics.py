#!/usr/bin/env python3
__author__ = "Laurence Roberts-Elliott"

import os
import pandas as pd
import numpy as np
from rospkg import RosPack

# Define the directory where the CSV files are located
# directory = '.'
rp = RosPack()
directory = rp.get_path('stage_soil_mapping_mrs') + '/bags'

# Get a list of all CSV files in the directory
csv_files = [file for file in os.listdir(directory) if file.endswith('.csv')]

# Filter out CSV files that contain the string "means"
csv_files = [file for file in csv_files if (("means" not in file) and ("vars" not in file) and ("stds" not in file))]
print(csv_files)

conditions = [
    "distance_over_variance_drop_low_var_tasks_True",
    "distance_over_variance_drop_low_var_tasks_False",
    "distance_over_variance_with_insertion_drop_low_var_tasks_True",
    "distance_over_variance_with_insertion_drop_low_var_tasks_False",
    "euclidean_distance_drop_low_var_tasks_False",
    "euclidean_distance_drop_low_var_tasks_True",
    "euclidean_distance_with_insertion_drop_low_var_tasks_False",
    "euclidean_distance_with_insertion_drop_low_var_tasks_True"
]

# Filter the list of CSV files to only include those that match the conditions
csv_files = [file for file in csv_files if any(condition in file for condition in conditions)]
# print(csv_files)

# Calculate mean of variables within each condition
for condition in conditions:
    condition_dfs = []
    print(condition)
    # Initialize a dictionary to store the variables and their means
    variable_means = {condition: {}}
    variable_stds = {condition: {}}
    variable_vars = {condition: {}}
    condition_files = [file for file in csv_files if condition in file]

    # Iterate over each CSV file
    for file in condition_files:
        # print(file)
        # Read the CSV file into a pandas DataFrame
        df = pd.read_csv(os.path.join(directory, file))

        # Calculate maximum of the "Value" columns of records where the metric column is "Max goal queue length"
        max_goal_queue_length_max = df.loc[df['Metric'] == 'Max goal queue length', 'Value'].max()
        # print(max_goal_queue_length_max)
        
        # Get rows where the column "Robot Name" is NaN
        df = df[df['Robot Name'].isna()]
        
        # Append max goal queue length to the dataframe using concat
        new_row = {'Metric': 'Max goal queue length', 'Robot Name': np.NaN, 'Value': max_goal_queue_length_max}
        df.loc[len(df)] = new_row

        # print(df)
        condition_dfs.append(df)

    # Concatenate the DataFrames in the list into a single DataFrame
    condition_df = pd.concat(condition_dfs)
    # print(condition_df)

    # Iterate over each metric in the DataFrame
    for metric in condition_df['Metric'].unique():

        # Get the rows where the column "Metric" is equal to the current metric
        metric_df = condition_df.loc[condition_df['Metric'] == metric]
        
        # # Exclude outliers in current metric
        # # Calculate Q1 and Q3
        # Q1 = metric_df['Value'].quantile(0.25)
        # Q3 = metric_df['Value'].quantile(0.75)
        # # Calculate IQR
        # IQR = Q3 - Q1
        # # Filter out outliers
        # metric_df = metric_df[(metric_df['Value'] >= 0) & (metric_df['Value'] < 500)]

        # Calculate the mean, standard deviation, and variance of the "Value" column
        mean = metric_df['Value'].mean()
        std = metric_df['Value'].std()
        var = metric_df['Value'].var()

        # Add the statistics to the dictionaries
        variable_means[condition][metric] = mean
        variable_stds[condition][metric] = std
        variable_vars[condition][metric] = var

    # Convert the dictionaries to DataFrames
    variable_means_df = pd.DataFrame(variable_means)
    variable_stds_df = pd.DataFrame(variable_stds)
    variable_vars_df = pd.DataFrame(variable_vars)

    # Save the DataFrames to CSV files
    variable_means_df.to_csv(os.path.join(directory, condition + '_means.csv'))
    variable_stds_df.to_csv(os.path.join(directory, condition + '_stds.csv'))
    variable_vars_df.to_csv(os.path.join(directory, condition + '_vars.csv'))

    print("Saved to " + os.path.join(directory, condition + '_means.csv'))
    print("Saved to " + os.path.join(directory, condition + '_stds.csv'))
    print("Saved to " + os.path.join(directory, condition + '_vars.csv'))
    print()
        