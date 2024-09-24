#!/usr/bin/env python3
__author__ = "Laurence Roberts-Elliott"

from bagpy import bagreader
import pandas as pd
import numpy as np
from rospkg import RosPack
from rosbag.bag import ROSBagException

def metrics_from_bag(bag_path, bag_folder_path):
    # Read the bag file
    try:
        b = bagreader(bag_path)
    except ROSBagException as e:
        print("Error reading bag file: " + str(e))
        return

    # Check topics
    topic_table = b.topic_table
    print(topic_table)

    # Get last step kriging RMSE
    rmse_msg = b.message_by_topic('/coordinator/RMSE')
    try:
        rmse_df = pd.read_csv(rmse_msg)
        print("Last step kriging RMSE: " + str(rmse_df.iloc[-1].data))
        # If RMSE is greater than 100, skip metrics for this bag file (erroneous kriging interpolation due to PyKrige bug)
        if rmse_df.iloc[-1].data > 100:
            print("Skipping metrics for this bag file...")
            return
    except ValueError:
        print("No data for kriging RMSE")
        rmse_df = pd.DataFrame(columns=['Time', 'data'], data=[[np.nan, np.nan]])
        print("Last step kriging RMSE: " + str(rmse_df.iloc[-1].data))
        print("Skipping metrics for this bag file...")
        return

    # Get last step mean kriging variance
    avgVar_msg = b.message_by_topic('/coordinator/avgVar')
    try:
        avgVar_df = pd.read_csv(avgVar_msg)
        print("Last step mean kriging variance: " + str(avgVar_df.iloc[-1].data))
    except ValueError:
        print("No data for mean kriging variance")
        avgVar_df = pd.DataFrame(columns=['Time', 'data'], data=[[np.nan, np.nan]])
        print("Last step mean kriging variance: " + str(avgVar_df.iloc[-1].data))

    # Get last number of samples
    num_samples_msg = b.message_by_topic('/coordinator/numSamples')
    try:
        num_samples_df = pd.read_csv(num_samples_msg)
        print("Last step number of samples: " + str(num_samples_df.iloc[-1].data))
    except ValueError:
        print("No data for number of samples")
        num_samples_df = pd.DataFrame(columns=['Time', 'data'], data=[[np.nan, np.nan]])
        print("Last step number of samples: " + str(num_samples_df.iloc[-1].data))

    # Get last step mean TA fairness
    mean_TA_fairness_msg = b.message_by_topic('/coordinator/mean_ta_fairness')
    mean_TA_fairness_df = pd.read_csv(mean_TA_fairness_msg)
    mean_TA_fairness = mean_TA_fairness_df.iloc[-1].data
    print("Last step mean TA fairness: " + str(mean_TA_fairness))

    # Get last step total robot tasks count
    total_robot_tasks_count_msg = b.message_by_topic('/coordinator/robot_tasks_allocated_count_total')
    total_robot_tasks_count_df = pd.read_csv(total_robot_tasks_count_msg)
    total_robot_tasks_count = total_robot_tasks_count_df.iloc[-1].data
    print("Last step total robot tasks count: " + str(total_robot_tasks_count))

    # Get robot names from topic table
    robot_names = []
    for i in range(len(topic_table)):
        if "robot_" in topic_table.iloc[i].Topics:
            robot_names.append(topic_table.iloc[i].Topics.split("/")[1])
    robot_names = np.unique(robot_names)

    if "coordinator" in robot_names:
        robot_names = np.delete(robot_names, np.where(robot_names == "coordinator"))

    # Print per-robot metrics
    for name in robot_names:
        print(name + " metrics:")

        # Get last step robot total_distance
        total_distance_msg = b.message_by_topic('/' + name + '/metrics/total_distance')
        try:
            total_distance_df = pd.read_csv(total_distance_msg)
        except:
            print("No data for " + name + " total distance")
            total_distance_df = pd.DataFrame(columns=['Time', 'data'], data=[[np.nan, np.nan]])
        print("Last step total distance: " + str(total_distance_df.iloc[-1].data))

        # Get last step robot total_idle_time
        total_idle_time_msg = b.message_by_topic('/' + name + '/metrics/total_idle_time')
        try:
            total_idle_time_df = pd.read_csv(total_idle_time_msg)
        except:
            print("No data for " + name + " total idle time")
            total_idle_time_df = pd.DataFrame(columns=['Time', 'data'], data=[[np.nan, np.nan]])
        print("Last step total idle time: " + str(total_idle_time_df.iloc[-1].data))

        # Get last step robot mean_task_completion_time
        mean_task_completion_time_msg = b.message_by_topic('/' + name + '/metrics/mean_task_completion_time')
        try:
            mean_task_completion_time_df = pd.read_csv(mean_task_completion_time_msg)
        except:
            print("No data for " + name + " mean task completion time")
            mean_task_completion_time_df = pd.DataFrame(columns=['Time', 'data'], data=[[np.nan, np.nan]])
        print("Last step mean task completion time: " + str(mean_task_completion_time_df.iloc[-1].data))

        # Get last step robot TA fairness
        TA_fairness_msg = b.message_by_topic('/' + name + '/coordinator/ta_fairness')
        try:
            TA_fairness_df = pd.read_csv(TA_fairness_msg)
        except:
            print("No data for " + name + " TA fairness")
            TA_fairness_df = pd.DataFrame(columns=['Time', 'data'], data=[[np.nan, np.nan]])
        print("Last step TA fairness: " + str(TA_fairness_df.iloc[-1].data))

        # Get last step robot tasks count
        robot_tasks_count_msg = b.message_by_topic('/' + name + '/coordinator/tasks_allocated_count')
        try:
            robot_tasks_count_df = pd.read_csv(robot_tasks_count_msg)
        except:
            print("No data for " + name + " tasks count")
            robot_tasks_count_df = pd.DataFrame(columns=['Time', 'data'], data=[[np.nan, np.nan]])
        print("Last step tasks count: " + str(robot_tasks_count_df.iloc[-1].data))

        # Get last step robot goal queue length
        goal_queue_length_msg = b.message_by_topic('/' + name + '/metrics/goal_queue_length')
        try:
            goal_queue_length_df = pd.read_csv(goal_queue_length_msg)
        except:
            print("No data for " + name + " goal queue length")
            goal_queue_length_df = pd.DataFrame(columns=['Time', 'data'], data=[[np.nan, np.nan]])
        print("Last step goal queue length: " + str(goal_queue_length_df.iloc[-1].data))

        # Get last step robot max goal queue length
        max_goal_queue_length_msg = b.message_by_topic('/' + name + '/metrics/max_goal_queue_length')
        try:
            max_goal_queue_length_df = pd.read_csv(max_goal_queue_length_msg)
        except:
            print("No data for " + name + " max goal queue length")
            max_goal_queue_length_df = pd.DataFrame(columns=['Time', 'data'], data=[[np.nan, np.nan]])
        print("Last step max goal queue length: " + str(max_goal_queue_length_df.iloc[-1].data))

    # Get global mean of per-robot metrics
    global_total_distance = 0
    global_total_idle_time = 0
    global_total_task_completion_time = 0
    global_TA_fairness = mean_TA_fairness
    global_robot_tasks_count = 0
    global_goal_queue_length = 0
    global_max_goal_queue_length = 0

    for name in robot_names:
        # Get last step robot total_distance
        total_distance_msg = b.message_by_topic('/' + name + '/metrics/total_distance')
        
        try:
            total_distance_df = pd.read_csv(total_distance_msg)
        except:
            print("No data for " + name + " total distance")
            total_distance_df = pd.DataFrame(columns=['Time', 'data'], data=[[np.nan, np.nan]])

        if total_distance_df.iloc[-1].data == 0 or np.isnan(total_distance_df.iloc[-1].data):
            print("Ignoring " + name + ", 0 or NaN total distance")
        else:
            global_total_distance += total_distance_df.iloc[-1].data

        # Get last step robot total_idle_time
        total_idle_time_msg = b.message_by_topic('/' + name + '/metrics/total_idle_time')
        
        try:
            total_idle_time_df = pd.read_csv(total_idle_time_msg)
        except:
            print("No data for " + name + " total idle time")
            total_idle_time_df = pd.DataFrame(columns=['Time', 'data'], data=[[np.nan, np.nan]])

        # Ignore value if 0 or NaN
        if (total_idle_time_df.iloc[-1].data == 0) or (np.isnan(total_idle_time_df.iloc[-1].data)):
            print("Ignoring " + name + ", 0 or NaN total idle time")
        else:
            global_total_idle_time += total_idle_time_df.iloc[-1].data

        # Get last step robot mean_task_completion_time
        mean_task_completion_time_msg = b.message_by_topic('/' + name + 
'/metrics/mean_task_completion_time')
        try:
            mean_task_completion_time_df = pd.read_csv(mean_task_completion_time_msg)
        except:
            print("No data for " + name + " mean task completion time")
            mean_task_completion_time_df = pd.DataFrame(columns=['Time', 'data'], 
data=[[np.nan, np.nan]])

        # Ignore value if 0 or NaN
        if (mean_task_completion_time_df.iloc[-1].data == 0) or (np.isnan(mean_task_completion_time_df.iloc[-1].data)):
            print("Ignoring " + name + ", 0 or NaN mean task completion time")
        else:
            global_total_task_completion_time += mean_task_completion_time_df.iloc[-1].data

        # Get last step robot TA fairness
        TA_fairness_msg = b.message_by_topic('/' + name + '/coordinator/ta_fairness')
        try:
            TA_fairness_df = pd.read_csv(TA_fairness_msg)
        except:
            print("No data for " + name + " TA fairness")
            TA_fairness_df = pd.DataFrame(columns=['Time', 'data'], data=[[np.nan, np.nan]])

        # Ignore value if 0 or NaN
        if TA_fairness_df.iloc[-1].data != 0 or not np.isnan(TA_fairness_df.iloc[-1].data):
            global_TA_fairness += TA_fairness_df.iloc[-1].data
        else:
            print("Ignoring " + name + ", 0 or NaN TA fairness")

        # Get last step robot tasks count
        robot_tasks_count_msg = b.message_by_topic('/' + name + 
'/coordinator/tasks_allocated_count')
        try:
            robot_tasks_count_df = pd.read_csv(robot_tasks_count_msg)
        except:
            print("No data for " + name + " tasks count")
            robot_tasks_count_df = pd.DataFrame(columns=['Time', 'data'], data=[[np.nan, 
np.nan]])

        # Ignore value if 0 or NaN
        if robot_tasks_count_df.iloc[-1].data != 0 or not np.isnan(robot_tasks_count_df.iloc[-1].data):
            global_robot_tasks_count += robot_tasks_count_df.iloc[-1].data
        else:
            print("Ignoring " + name + ", 0 or NaN tasks count")

        # Get last step robot goal queue length
        goal_queue_length_msg = b.message_by_topic('/' + name + '/metrics/goal_queue_length')
        try:
            goal_queue_length_df = pd.read_csv(goal_queue_length_msg)
        except:
            print("No data for " + name + " goal queue length")
            goal_queue_length_df = pd.DataFrame(columns=['Time', 'data'], data=[[np.nan, 
np.nan]])

        # Ignore value if 0 or NaN
        if goal_queue_length_df.iloc[-1].data != 0 or not np.isnan(goal_queue_length_df.iloc[-1].data):
            global_goal_queue_length += goal_queue_length_df.iloc[-1].data
        else:
            print("Ignoring " + name + ", 0 or NaN goal queue length")

        # Get last step robot max goal queue length
        max_goal_queue_length_msg = b.message_by_topic('/' + name + 
'/metrics/max_goal_queue_length')
        try:
            max_goal_queue_length_df = pd.read_csv(max_goal_queue_length_msg)
        except:
            print("No data for " + name + " max goal queue length")
            max_goal_queue_length_df = pd.DataFrame(columns=['Time', 'data'], data=[[np.nan, 
np.nan]])

        # Ignore value if 0 or NaN
        if max_goal_queue_length_df.iloc[-1].data != 0 or not np.isnan(max_goal_queue_length_df.iloc[-1].data):
            global_max_goal_queue_length += max_goal_queue_length_df.iloc[-1].data
        else:
            print("Ignoring " + name + ", 0 or NaN max goal queue length")


    print("Global metrics:")
    last_step_total_distance = global_total_distance
    print("Last step total distance: " + str(last_step_total_distance))
    last_step_total_idle_time = global_total_idle_time
    print("Last step total idle time: " + str(last_step_total_idle_time))
    last_step_mean_task_completion_time = global_total_task_completion_time / len(robot_names)
    print("Last step mean of task completion time: " + 
str(last_step_mean_task_completion_time))
    print("Last step mean TA fairness: " + str(mean_TA_fairness))
    print("Last step total robot tasks count: " + str(total_robot_tasks_count))


    # Put last step metrics in a DataFrame
    metrics_df = pd.DataFrame(columns=['Metric', 'Robot Name', 'Value'])

    for name in robot_names:
        # Get last step robot total_distance
        total_distance_msg = b.message_by_topic('/' + name + '/metrics/total_distance')
        
        try:
            total_distance_df = pd.read_csv(total_distance_msg)
        except:
            print("No data for " + name + " total distance")
            total_distance_df = pd.DataFrame(columns=['Time', 'data'], data=[[np.nan, 0]])
        metrics_df = pd.concat([metrics_df, pd.DataFrame({'Metric': 'Total distance travelled (m)', 'Robot Name': name, 'Value': total_distance_df.iloc[-1].data}, index=[0])], ignore_index=True)

        # Get last step robot total_idle_time
        total_idle_time_msg = b.message_by_topic('/' + name + '/metrics/total_idle_time')
        try:
            total_idle_time_df = pd.read_csv(total_idle_time_msg)
        except:
            print("No data for " + name + " total idle time")
            total_idle_time_df = pd.DataFrame(columns=['Time', 'data'], data=[[np.nan, 0]])
        metrics_df = pd.concat([metrics_df, pd.DataFrame({'Metric': 'Total idle time (s)', 
'Robot Name': name, 'Value': total_idle_time_df.iloc[-1].data}, index=[0])], ignore_index=True)

        # Get last step robot mean_task_completion_time
        mean_task_completion_time_msg = b.message_by_topic('/' + name + 
'/metrics/mean_task_completion_time')
        try:
            mean_task_completion_time_df = pd.read_csv(mean_task_completion_time_msg)
        except:
            print("No data for " + name + " mean task completion time")
            mean_task_completion_time_df = pd.DataFrame(columns=['Time', 'data'], 
data=[[np.nan, 0]])
        metrics_df = pd.concat([metrics_df, pd.DataFrame({'Metric': 'Mean task completion time (s)', 'Robot Name': name, 'Value': mean_task_completion_time_df.iloc[-1].data}, index=[0])], ignore_index=True)

        # Get last step robot TA fairness
        TA_fairness_msg = b.message_by_topic('/' + name + '/coordinator/ta_fairness')
        try:
            TA_fairness_df = pd.read_csv(TA_fairness_msg)
        except:
            print("No data for " + name + " TA fairness")
            TA_fairness_df = pd.DataFrame(columns=['Time', 'data'], data=[[np.nan, 0]])
        metrics_df = pd.concat([metrics_df, pd.DataFrame({'Metric': 'TA fairness', 'Robot Name': name, 'Value': TA_fairness_df.iloc[-1].data}, index=[0])], ignore_index=True)

        # Get last step robot tasks count
        robot_tasks_count_msg = b.message_by_topic('/' + name + 
'/coordinator/tasks_allocated_count')
        try:
            robot_tasks_count_df = pd.read_csv(robot_tasks_count_msg)
        except:
            print("No data for " + name + " tasks count")
            robot_tasks_count_df = pd.DataFrame(columns=['Time', 'data'], data=[[np.nan, 0]])
        metrics_df = pd.concat([metrics_df, pd.DataFrame({'Metric': 'Tasks count', 'Robot Name': name, 'Value': robot_tasks_count_df.iloc[-1].data}, index=[0])], ignore_index=True)

        # Get last step robot goal queue length
        goal_queue_length_msg = b.message_by_topic('/' + name + '/metrics/goal_queue_length')
        try:
            goal_queue_length_df = pd.read_csv(goal_queue_length_msg)
        except:
            print("No data for " + name + " goal queue length")
            goal_queue_length_df = pd.DataFrame(columns=['Time', 'data'], data=[[np.nan, 0]])
        metrics_df = pd.concat([metrics_df, pd.DataFrame({'Metric': 'Goal queue length', 'Robot Name': name, 'Value': goal_queue_length_df.iloc[-1].data}, index=[0])], ignore_index=True)

        # Get last step robot max goal queue length
        max_goal_queue_length_msg = b.message_by_topic('/' + name + 
'/metrics/max_goal_queue_length')
        try:
            max_goal_queue_length_df = pd.read_csv(max_goal_queue_length_msg)
        except:
            print("No data for " + name + " max goal queue length")
            max_goal_queue_length_df = pd.DataFrame(columns=['Time', 'data'], data=[[np.nan, 
0]])
        metrics_df = pd.concat([metrics_df, pd.DataFrame({'Metric': 'Max goal queue length', 
'Robot Name': name, 'Value': max_goal_queue_length_df.iloc[-1].data}, index=[0])], 
ignore_index=True)

    # Append last step kriging RMSE
    rmse_msg = b.message_by_topic('/coordinator/RMSE')
    try:
        rmse_df = pd.read_csv(rmse_msg)
        metrics_df = pd.concat([metrics_df, pd.DataFrame({'Metric': 'Kriging RMSE (Root Mean Squared Error)', 'Robot Name': "NA", 'Value': rmse_df.iloc[-1].data}, index=[0])], ignore_index=True)
    except ValueError:
        metrics_df = pd.concat([metrics_df, pd.DataFrame({'Metric': 'Kriging RMSE (Root Mean Squared Error)', 'Robot Name': "NA", 'Value': np.nan}, index=[0])], ignore_index=True)

    # Append last step mean kriging variance
    avgVar_msg = b.message_by_topic('/coordinator/avgVar')
    try:
        avgVar_df = pd.read_csv(avgVar_msg)
        metrics_df = pd.concat([metrics_df, pd.DataFrame({'Metric': 'Mean kriging variance', 'Robot Name': "NA", 'Value': avgVar_df.iloc[-1].data}, index=[0])], ignore_index=True)
    except ValueError:
        metrics_df = pd.concat([metrics_df, pd.DataFrame({'Metric': 'Mean kriging variance', 'Robot Name': "NA", 'Value': np.nan}, index=[0])], ignore_index=True)

    # Append last step mean TA fairness
    mean_TA_fairness_msg = b.message_by_topic('/coordinator/mean_ta_fairness')
    mean_TA_fairness_df = pd.read_csv(mean_TA_fairness_msg)
    metrics_df = pd.concat([metrics_df, pd.DataFrame({'Metric': 'Mean TA fairness', 'Robot Name': "NA", 'Value': mean_TA_fairness_df.iloc[-1].data}, index=[0])], ignore_index=True)

    # Append last step total robot tasks count
    total_robot_tasks_count_msg = b.message_by_topic('/coordinator/robot_tasks_allocated_count_total')
    total_robot_tasks_count_df = pd.read_csv(total_robot_tasks_count_msg)
    metrics_df = pd.concat([metrics_df, pd.DataFrame({'Metric': 'Total robot tasks count', 'Robot Name': "NA", 'Value': total_robot_tasks_count_df.iloc[-1].data}, index=[0])], ignore_index=True)

    # Get number of samples and append to metrics_df
    num_samples_msg = b.message_by_topic('/coordinator/numSamples')
    try:
        num_samples_df = pd.read_csv(num_samples_msg)
        metrics_df = pd.concat([metrics_df, pd.DataFrame({'Metric': 'Number of samples', 'Robot Name': "NA", 'Value': num_samples_df.iloc[-1].data}, index=[0])], ignore_index=True)
    except ValueError:
        metrics_df = pd.concat([metrics_df, pd.DataFrame({'Metric': 'Number of samples', 'Robot Name': "NA", 'Value': np.nan}, index=[0])], ignore_index=True)

    # Append last step total distance
    metrics_df = pd.concat([metrics_df, pd.DataFrame({'Metric': 'Total distance travelled (m)', 'Robot Name': "NA", 'Value': last_step_total_distance}, index=[0])], ignore_index=True)

    # Append last step mean idle time
    metrics_df = pd.concat([metrics_df, pd.DataFrame({'Metric': 'Total idle time (s)', 'Robot Name': "NA", 'Value': last_step_total_idle_time}, index=[0])], ignore_index=True)

    # Append last step mean task completion time
    metrics_df = pd.concat([metrics_df, pd.DataFrame({'Metric': 'Mean task completion time (s)', 'Robot Name': "NA", 'Value': last_step_mean_task_completion_time}, index=[0])], ignore_index=True)

    # Save metrics DataFrame to csv
    csv_name = bag_path.split("/")[-1].split(".")[0] + "_metrics.csv"
    metrics_df.to_csv(bag_folder_path + "/" + csv_name, index=False)


if __name__ == '__main__':
    # Get the path to the bag file
    rp = RosPack()
    # bag_path = rp.get_path('sim_soil_mapping_mrs') + '/bags/metrics.bag'
    bag_path = rp.get_path('sim_soil_mapping_mrs') + '/bags/distance_over_variance_drop_low_var_tasks_True_1.bag'
    bag_folder_path = rp.get_path('sim_soil_mapping_mrs') + '/bags/'

    metrics_from_bag(bag_path, bag_folder_path)

    
