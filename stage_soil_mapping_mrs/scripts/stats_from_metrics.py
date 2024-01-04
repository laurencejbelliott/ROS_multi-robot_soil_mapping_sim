import os
import pandas as pd
import numpy as np

# Define the directory where the CSV files are located
directory = '.'

# Get a list of all CSV files in the directory
csv_files = [file for file in os.listdir(directory) if file.endswith('.csv')]

# Filter out CSV files that contain the string "means"
csv_files = [file for file in csv_files if "means" not in file]

conditions = ["euclidean_distance_use_queue_sorting_False", "euclidean_distance_with_insertion_use_queue_sorting_False"]

# Filter the list of CSV files to only include those that match the conditions
csv_files = [file for file in csv_files if any(condition in file for condition in conditions)]
# print(csv_files)

# Calculate mean of variables within each condition
for condition in conditions:
    condition_dfs = []
    print(condition)
    # Initialize a dictionary to store the variables and their means
    variable_means = {condition: {}}
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
        
        # Exclude outliers in current metric
        # Calculate Q1 and Q3
        Q1 = metric_df['Value'].quantile(0.25)
        Q3 = metric_df['Value'].quantile(0.75)
        # Calculate IQR
        IQR = Q3 - Q1
        # Filter out outliers
        metric_df = metric_df[(metric_df['Value'] >= 0) & (metric_df['Value'] < 500)]

        
        # Calculate the mean of the "Value" column
        mean = metric_df['Value'].mean()
        # Add the mean to the dictionary
        variable_means[condition][metric] = mean

    # Convert the dictionary to a DataFrame
    variable_means_df = pd.DataFrame(variable_means)
    print(variable_means_df)

    # Save the DataFrame to a CSV file
    variable_means_df.to_csv(os.path.join(directory, condition + '_means.csv'))
    print("Saved to " + os.path.join(directory, condition + '_means.csv'))
    print()
        