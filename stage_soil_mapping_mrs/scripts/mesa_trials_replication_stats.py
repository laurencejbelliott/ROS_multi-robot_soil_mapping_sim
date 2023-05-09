#!/usr/bin/env python3
__author__ = "Laurence Roberts-Elliott"
import os
import rospkg
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

rospack = rospkg.RosPack()

csv_path = rospack.get_path('stage_soil_mapping_mrs') + '/bags/mesa_trials_replication/'

# Get list of csv files in `csv_path` that start with `3_robs`
condition_names = []
trial_nums = []
for path in os.listdir(csv_path):
    if path.startswith("3_robs") and path.endswith(".csv") and "avg" not in path:
        path_split = path.split("_")
        condition_name = "_".join(path_split[:-2])
        condition_names.append(condition_name)
        trial_num = path_split[-2]
        trial_nums.append(trial_num)
condition_names = np.unique(condition_names)
trial_nums = np.sort(np.int8(np.unique(trial_nums)))

# print(condition_names)
# print(trial_nums)

# Create dataframe to store average metrics for each condition
condition_avg_df = pd.DataFrame()
for condition in condition_names:
    trial_dfs = []
    for trial_num in trial_nums:
        # print("Condition: " + condition + ", Trial: " + str(trial_num))

        # Metrics to average across trials in the condition
        # Kriging RMSE (Root Mean Squared Error)
        # Mean kriging variance
        # Number of sample
        # Total distance travelled (m)
        # Total idle time (s)
        # Mean task completion time (s)

        csv_file = condition + "_" + str(trial_num) + "_metrics.csv"
        csv_df = pd.read_csv(csv_path + csv_file)

        # Filter to records where 'Robot Name' is NaN
        csv_df = csv_df[csv_df['Robot Name'].isna()]

        # print(csv_df)
        trial_dfs.append(csv_df)

    # Concatenate all trial dataframes into one dataframe
    condition_df = pd.concat(trial_dfs)
    # print(condition_df)

    # Average across trials
    print("\n"+condition)
    condition_avg_df = condition_df.groupby(['Metric']).mean()
    condition_avg_df.to_csv(csv_path + condition + "_avg.csv")

# The same again but excluding trials where the RMSE or MKV (Mean Kriging Variance) is an outlier
condition_avg_df = pd.DataFrame()
for condition in condition_names:
    trial_dfs = []
    for trial_num in trial_nums:
        # print("Condition: " + condition + ", Trial: " + str(trial_num))

        # Metrics to average across trials in the condition
        # Kriging RMSE (Root Mean Squared Error)
        # Mean kriging variance
        # Number of sample
        # Total distance travelled (m)
        # Total idle time (s)
        # Mean task completion time (s)

        csv_file = condition + "_" + str(trial_num) + "_metrics.csv"
        csv_df = pd.read_csv(csv_path + csv_file)

        # Filter to records where 'Robot Name' is NaN
        csv_df = csv_df[csv_df['Robot Name'].isna()]
        # print(csv_df.keys())

        # print(csv_df)

        # # Append to list of trial dataframes if the RMSE and MKV are not outliers
        # if not (csv_df.loc[csv_df['Metric'] == 'Kriging RMSE (Root Mean Squared Error)']['Value'] < 0).any() and not (csv_df.loc[csv_df['Metric'] == 'Kriging RMSE (Root Mean Squared Error)']['Value'] > 9999).any() \
        # and not (csv_df.loc[csv_df['Metric'] == 'Mean kriging variance']['Value'] < 0).any() and not (csv_df.loc[csv_df['Metric'] == 'Mean kriging variance']['Value'] > 9999).any():
        #     trial_dfs.append(csv_df)

        trial_dfs.append(csv_df)

    # Concatenate all trial dataframes into one dataframe
    condition_df = pd.concat(trial_dfs)
    print(condition_df)

    # Remove outliers in the 'RMSE' metric using the IQR method
    # https://www.kdnuggets.com/2017/02/removing-outliers-standard-deviation-python.html
    rmse_values = condition_df.loc[condition_df['Metric'] == 'Kriging RMSE (Root Mean Squared Error)']['Value']
    # print(list(rmse_values))
    q1 = rmse_values.quantile(0.25)
    q3 = rmse_values.quantile(0.75)
    iqr = q3 - q1

    condition_df = condition_df[~((condition_df['Metric'] == 'Kriging RMSE (Root Mean Squared Error)') & ((condition_df['Value'] < (q1 - 1.5 * iqr)) | (condition_df['Value'] > (q3 + 1.5 * iqr))))]

    # Remove outliers in the 'Mean kriging variance' metric using the IQR method
    mkv_values = condition_df.loc[condition_df['Metric'] == 'Mean kriging variance']['Value']
    # print(list(mkv_values))
    q1 = mkv_values.quantile(0.25)
    q3 = mkv_values.quantile(0.75)
    iqr = q3 - q1

    condition_df = condition_df[~((condition_df['Metric'] == 'Mean kriging variance') & ((condition_df['Value'] < (q1 - 1.5 * iqr)) | (condition_df['Value'] > (q3 + 1.5 * iqr))))]

    # Average across trials
    print("\n"+condition)
    condition_avg_df = condition_df.groupby(['Metric']).mean()
    condition_avg_df.to_csv(csv_path + condition + "_avg_no_outliers.csv")

# Find the best 2 conditions for each metric
metrics_dict = {
    'Kriging RMSE (Root Mean Squared Error)': {'top_conditions': [], 'top_values': []},
    'Mean kriging variance': {'top_conditions': [], 'top_values': []},
    'Number of samples': {'top_conditions': [], 'top_values': []},
    'Total distance travelled (m)': {'top_conditions': [], 'top_values': []},
    'Total idle time (s)': {'top_conditions': [], 'top_values': []},
    'Mean task completion time (s)': {'top_conditions': [], 'top_values': []}
}

# Get list of csv files in `csv_path` that contain `avg_no_outliers`
for path in os.listdir(csv_path):
    if path.startswith("3_robs") and path.endswith(".csv") and "avg_no_outliers" in path:
        # print(path)

        # Read csv file into dataframe
        csv_df = pd.read_csv(csv_path + path)
        # print(csv_df)

        # Get condition name from csv file name
        condition = path.split("_avg_no_outliers")[0]
        # print(condition)

        # Iterate through each metric
        for metric in metrics_dict.keys():
            # Get metric value
            metric_value = csv_df.loc[csv_df['Metric'] == metric]['Value'].values[0]
            # print(metric_value)

            # If the metric value is better (less than) than the current best value, replace it
            if len(metrics_dict[metric]['top_values']) < 2:
                metrics_dict[metric]['top_conditions'].append(condition)
                metrics_dict[metric]['top_values'].append(metric_value)
            elif metric_value < max(metrics_dict[metric]['top_values']):
                max_index = metrics_dict[metric]['top_values'].index(max(metrics_dict[metric]['top_values']))
                metrics_dict[metric]['top_conditions'][max_index] = condition
                metrics_dict[metric]['top_values'][max_index] = metric_value

# Sort the conditions and values by the best metric value
for metric in metrics_dict.keys():
    metrics_dict[metric]['top_conditions'] = [x for _,x in sorted(zip(metrics_dict[metric]['top_values'],metrics_dict[metric]['top_conditions']))]
    metrics_dict[metric]['top_values'] = sorted(metrics_dict[metric]['top_values'])

print("\nBest conditions for each metric:")
for metric in metrics_dict.keys():
    print(metric + ": " + str(metrics_dict[metric]['top_conditions']) + " (" + str(metrics_dict[metric]['top_values']) + ")")

# # Plot the best conditions for each metric
# for metric in metrics_dict.keys():
#     # Get the condition names and values for the metric
#     condition_names = metrics_dict[metric]['top_conditions']
#     condition_values = metrics_dict[metric]['top_values']

#     # Get the dataframes for the conditions
#     condition_dfs = []
#     for condition in condition_names:
#         # Get the csv file for the condition
#         csv_file = condition + "_avg_no_outliers.csv"
#         csv_df = pd.read_csv(csv_path + csv_file)

#         # Filter to records where 'Metric' is the metric
#         csv_df = csv_df[csv_df['Metric'] == metric]
#         # print(csv_df)

#         condition_dfs.append(csv_df)

#     # Concatenate all condition dataframes into one dataframe
#     metric_df = pd.concat(condition_dfs)
#     # print(metric_df)

#     # Plot the metric values
#     plt.figure()
#     plt.bar(condition_names, condition_values)
#     plt.title(metric)
#     plt.xlabel('Condition')
#     plt.ylabel(metric)
#     plt.xticks(rotation=45)
#     plt.tight_layout()
#     plt.savefig(csv_path + metric + ".png")
#     plt.show()

# 1-tailed Welch's t-test comparing the best 2 conditions for each metric
# https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.ttest_ind.html

# TODO: 1-tailed Welch's t-test comparing the best 2 conditions for each metric

# # Concatenate all condition dataframes into one dataframe
# for condition in metrics_dict['Kriging RMSE (Root Mean Squared Error)']['top_conditions']:

#     # Get the csv files for the trials in the condition