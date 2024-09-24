#!/usr/bin/env python3
__author__ = 'Laurence Roberts-Elliott'
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from rospkg import RosPack

def create_box_plots(folder_path, match_string=None):
    # Get a list of all CSV files in the folder
    csv_files = [file for file in os.listdir(folder_path) if file.endswith('.csv')]
    if match_string != None:
        csv_files = [file for file in csv_files if match_string in file]
    print(csv_files)

    condtion_df = pd.DataFrame(columns=['Metric', 'Robot Name', 'Value'])
    # Iterate over each CSV file appending the data to a pandas DataFrame
    for file in csv_files:
        df = pd.read_csv(folder_path + '/' + file, header=0)
        # print(df.columns)
        # print(df)
        # condtion_df = condtion_df.append(df)
        condtion_df = pd.concat([condtion_df, df], ignore_index=True)
        
    # Drop records where the 'Robot Name' is not NA
    condtion_df = condtion_df[condtion_df['Robot Name'].isna()]

    # Drop the 'Robot Name' column
    condtion_df = condtion_df.drop(columns=['Robot Name'])

    # Drop the header row
    new_header = condtion_df.iloc[0] # Grab the first row for the header
    condtion_df = condtion_df[1:] # Take the data less the header row
    condtion_df.columns = new_header # Set the header row as the df header

    # Transpose the DataFrame to use the 'Metric' column's values as the column headers
    condtion_df = condtion_df.T

    condtion_df.rename(columns=condtion_df.iloc[0], inplace=True)
    print(condtion_df.columns)

    # Print the column names
    unique_columns = condtion_df.columns.unique()
    print("Unique columns: " + str(unique_columns))
    num_unique_columns = len(condtion_df.columns.unique())
    print("Number of unique columns: " + str(num_unique_columns))

    condtion_df_no_duplicate_columns = pd.DataFrame(columns=['Metric', 'Robot Name', 'Value'])
    condtion_df_values = condtion_df.iloc[1].to_numpy()
    print(condtion_df_values)

    # Add rows to the new DataFrame from slices of conditon_df_values where the slice is the length of the number of unique columns
    for i in range(0, len(condtion_df_values), num_unique_columns):
        print(i)
        print(i+num_unique_columns)
        print(condtion_df_values[i:i+num_unique_columns])
        len_of_returned_slice = len(condtion_df_values[i:i+num_unique_columns])
        print(len_of_returned_slice)

        # If the length of the returned slice is not equal to the number of unique columns then pad the slice with NaN values
        if len_of_returned_slice == num_unique_columns:
            condtion_df_no_duplicate_columns = pd.concat([condtion_df_no_duplicate_columns, pd.DataFrame([condtion_df_values[i:i+num_unique_columns]], columns=unique_columns)])

    # Drop 'Metric', 'Robot Name' and 'Value' columns
    condtion_df_no_duplicate_columns = condtion_df_no_duplicate_columns.drop(columns=['Metric', 'Robot Name', 'Value'])

    # Handle NaN values by replacing with the mean of the column
    condtion_df_no_duplicate_columns = condtion_df_no_duplicate_columns.fillna(condtion_df_no_duplicate_columns.mean())

    print(condtion_df_no_duplicate_columns)

    # Create a boxplot of the data across all CSV files
    condtion_df_no_duplicate_columns.boxplot()
    plt.show()
    plt.close()

    # Create a seperate boxplot for each column
    for column in condtion_df_no_duplicate_columns:
        condtion_df_no_duplicate_columns.boxplot(column=column)
        plt.show()
        plt.close()


if __name__ == '__main__':
    # Get the path to the folder containing the CSV files
    rp = RosPack()
    folder_path = rp.get_path('sim_soil_mapping_mrs') + '/bags'
    create_box_plots(folder_path, "distance_over_variance_with_insertion_use_queue_sorting_False_")
