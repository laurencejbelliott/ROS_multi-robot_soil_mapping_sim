#!/usr/bin/env python3
__author__ = "Laurence Roberts-Elliott"

import os
import pandas as pd
import numpy as np
from rospkg import RosPack
from scipy.stats import f_oneway, normaltest, kruskal, tukey_hsd, wilcoxon
from matplotlib import pyplot as plt

# Define the directory where the CSV files are located
# directory = '.'
rp = RosPack()
directory = rp.get_path('stage_soil_mapping_mrs') + '/bags'

# Get a list of all CSV files in the directory
csv_files = [file for file in os.listdir(directory) if file.endswith('.csv')]

# Filter out CSV files that contain the string "means"
csv_files = [file for file in csv_files if (("means" not in file) and ("vars" not in file) and ("stds" not in file))]
# print(csv_files)

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

condition_shorthand = [
    "DOVTD",
    "DOV",
    "DOVCITD",
    "DOVCI",
    "ED",
    "EDTD",
    "EDCI",
    "EDCITD"
]

# Filter the list of CSV files to only include those that match the conditions
csv_files = [file for file in csv_files if any(condition in file for condition in conditions)]
# print(csv_files)

# Get list of metrics from the first CSV file
df = pd.read_csv(directory + '/' + csv_files[0])
metrics = list(df['Metric'])
print(metrics)
print(len(metrics))

# Exclude metrics where the value in the 'Robot Name' column is not NaN
df = df[df['Robot Name'].isnull()]
metrics = list(df['Metric'])
print(metrics)
print(len(metrics))

# Append global max goal queue length to the list of metrics
metrics.append('Global max goal queue length')

# Perform ANOVA and Tukey's HSD test for each metric
for metric in metrics:
    print("\nMetric: " + metric)
    # List of lists of observations of the current metric for each condition (indices of these lists map to the indices of the `conditions` list)
    observations = []

    # Dataframe for significance test results
    significance_test_results = pd.DataFrame(columns=condition_shorthand, index=condition_shorthand)

    # Set diagonal to NaN
    for i in range(0, len(conditions)):
        significance_test_results.iloc[i, i] = "NaN"

    for condition in conditions:
        # Get csv files for the condition
        condition_files = [file for file in csv_files if condition in file]
        # print(condition_files)
        condition_observations = []
        for condition_file in condition_files:
            # Read the CSV file into a pandas DataFrame
            df = pd.read_csv(directory + '/' + condition_file)
            # print(df)
        
        
            # Calculate global max goal queue length from per-robot max goal queue lengths
            # Get max goal queue length from each robot
            max_goal_queue_lengths = []
            max_goal_queue_lengths_df = df.loc[df['Metric'] == 'Max goal queue length']
            for i in range(len(max_goal_queue_lengths_df)):
                max_goal_queue_lengths.append(max_goal_queue_lengths_df.iloc[i]['Value'])
            # Calculate global max goal queue length
            global_max_goal_queue_length = max(max_goal_queue_lengths)

            # Exclude metrics where the value in the 'Robot Name' column is not NaN
            df = df[df['Robot Name'].isnull()]


            # Add the global max goal queue length to the DataFrame
            new_row = {'Metric': 'Global max goal queue length', 'Robot Name': np.NaN, 'Value': global_max_goal_queue_length}
            df.loc[len(df)] = new_row
            # print("\n" + str(df))

            # Get value where metric is the current metric
            metric_df = df.loc[df['Metric'] == metric]
            # print(metric_df['Value'].values[0])
            condition_observations.append(metric_df['Value'].values[0])
        
        # Perform normality test
        print("Normality test")
        k2_statistic, p_value = normaltest(condition_observations)
        print("K^2 statistic: " + str(k2_statistic))
        print("P-value: " + str(p_value))
        if p_value < 0.05:
            print("The data is not normally distributed")
        else:
            print("The data is normally distributed")

        # # Plot frequency distribution
        # plt.hist(condition_observations, bins=10)
        # plt.show()

        observations.append(condition_observations)
    
    # for c in range(0, len(conditions)):
    #     print(conditions[c] + ": " + str(observations[c]))

    # if metric is RMSE (not normally distributed), perform Kruskal-Wallis test
    if metric == 'Kriging RMSE (Root Mean Squared Error)':
        print("\nKruskal-Wallis test")

        h_statistic, p_value = kruskal(*observations)
        print("H-statistic: " + str(h_statistic))
        print("P-value: " + str(p_value))
        if p_value < 0.05:
            print("There is a significant difference between the means")

            # Perform Wilcoxon signed-rank test
            print("\n")
            print("Wilcoxon signed-rank test")
            for i in range(0, len(conditions)):
                for j in range(i+1, len(conditions)):
                    print(conditions[i] + " vs " + conditions[j])
                    w_statistic, p_value = wilcoxon(observations[i], observations[j])
                    print("W-statistic: " + str(w_statistic))
                    print("P-value: " + str(p_value))
                    if p_value < 0.05:
                        print("There is a significant difference between the means")
                        # Enter y in significance_test_results dataframe
                        significance_test_results.iloc[i, j] = 'y'
                        significance_test_results.iloc[j, i] = 'y'
                    else:
                        print("There is no significant difference between the means")
                        # Enter n in significance_test_results dataframe
                        significance_test_results.iloc[i, j] = 'n'
                        significance_test_results.iloc[j, i] = 'n'
                    print("\n")
        else:
            print("There is no significant difference between the means")
    else:
        # Perform ANOVA
        f_statistic, p_value = f_oneway(*observations)
        print("\nANOVA")
        print("F-statistic: " + str(f_statistic))
        print("P-value: " + str(p_value))

        # Is there a significant difference between the means?
        if p_value < 0.05:
            print("There is a significant difference between the means")

            # Perform Tukey's HSD test
            print("\n")
            print("Tukey's HSD (Honestly Significant Distance) test")
            result = tukey_hsd(*observations)
            # print(result)

            # Print pairwise comparisons
            for i in range(0, len(conditions)):
                for j in range(i+1, len(conditions)):
                    print(conditions[i] + " vs " + conditions[j])
                    print(result.pvalue[i][j])
                    if result.pvalue[i][j] < 0.05:
                        print("There is a significant difference between the means")
                        # Enter y in significance_test_results dataframe
                        significance_test_results.iloc[i, j] = 'y'
                        significance_test_results.iloc[j, i] = 'y'
                    else:
                        print("There is no significant difference between the means")
                        # Enter n in significance_test_results dataframe
                        significance_test_results.iloc[i, j] = 'n'
                        significance_test_results.iloc[j, i] = 'n'
                    print("\n")
        else:
            print("There is no significant difference between the means")
    significance_test_results.to_csv(directory + '/' + metric + '_significance_test_results.csv')

for i in range(0, len(conditions)):
    print(str(i) + ": " + condition_shorthand[i] + ": " + conditions[i])