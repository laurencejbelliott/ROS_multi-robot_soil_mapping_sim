#!/usr/bin/env python3
from os.path import exists
import roslaunch
import rospy
import itertools
from rospkg import RosPack
from roslaunch.parent import ROSLaunchParent
from std_msgs.msg import Bool

rospack = RosPack()
package_path = rospack.get_path('sim_soil_mapping_mrs')

print(package_path)

trial_num = 1
num_robots = 3
ta_algo = "SSI"
sampling_algo = "random"
env_crop_factor = 1 # Environment cropping is not yet implemented
sampling_time_budget = 240 # seconds, as in Mesa sim trials
figures_path = package_path+'/figures/"'
simulator = "virtual_robots"
bid_function = "euclidean_distance_with_insertion"
use_queue_sorting = True
drop_low_variance_tasks = True


# Define sets of parameters to combine
trial_num_set = set([i for i in range(1, 11)])
print("Set of trial numbers: ", trial_num_set)

num_robots_set = set([num_robots])
print("Set of number of robots: ", num_robots_set)

ta_algo_set = set(['SSI'])
print("Set of task allocation algorithms: ", ta_algo_set)

sampling_algo_set = set(["dynamic"]) # No 'grid' sampling in Mesa trials
print("Set of sampling algorithms: ", sampling_algo_set)

env_crop_factor_set = set([env_crop_factor])
print("Set of environment cropping factors: ", env_crop_factor_set)

# bid_function_set = set(["insertion_weighted_euclidean_distance", "insertion_weighted_distance_over_variance"])
bid_function_set = set(["distance_over_variance", "distance_over_variance_with_insertion", "insertion_weighted_distance_over_variance"])
print("Set of bid functions: ", bid_function_set)

use_queue_sorting_set = set([False])
print("Set of use_queue_sorting states: ", use_queue_sorting_set)

drop_low_variance_tasks_set = set([True, False])
print("Set of drop_low_variance_tasks states: ", drop_low_variance_tasks_set)

# Create all combinations of trial_num, num_robots, ta_algo, sampling_algo, env_crop_factor
combinations = list(itertools.product(trial_num_set, num_robots_set, ta_algo_set, sampling_algo_set, env_crop_factor_set, bid_function_set, use_queue_sorting_set, drop_low_variance_tasks_set))
# Remove combinations that have no trial_num, num_robots, ta_algo, sampling_algo, env_crop_factor
combinations = [i for i in combinations if i[0] != i[2] != i[3] != i[4] != i[5] != i[6]]
total_trials = len(combinations)

print("\nRunning " + str(total_trials) + " simulated trials:\n", combinations)
trial_count = 0
for trial_params in combinations:
    trial_count += 1
    trial_num = trial_params[0]
    num_robots = trial_params[1]
    ta_algo = trial_params[2]
    sampling_algo = trial_params[3]
    env_crop_factor = trial_params[4]
    bid_function = trial_params[5]
    use_queue_sorting = trial_params[6]
    drop_low_variance_tasks = trial_params[7]

    # Check if trial has already run by checking if rosbag file exists
    bag_name = str(bid_function)+'_drop_low_var_tasks_'+str(drop_low_variance_tasks)+'_'+str(trial_num)
    trial_name = bag_name
    figures_path = package_path+'/figures/'+bag_name
    
    if exists(package_path+'/bags/'+bag_name+'.bag'):
        print("Trial ", bag_name," (", trial_count, "/", total_trials,") has already been run, skipping...")
        continue

    print("Running trial ", trial_count, "/", total_trials, ": ", trial_params)

    cli_args = [package_path+'/launch/mba_sim_'+str(num_robots)+'_rob_compaction_field.launch',
                'trial_num:='+str(trial_num),
                'num_robots:='+str(num_robots),
                'ta_algo:="'+ta_algo+'"',
                'sampling_algo:="'+sampling_algo+'"',
                'env_crop_factor:=1',
                'bag_name:="'+bag_name+'"',
                'sampling_time_budget:='+str(sampling_time_budget),
                'figures_path:='+'"'+figures_path+'"',
                'simulator:='+simulator,
                'bid_function:='+bid_function,
                'use_queue_sorting:='+str(use_queue_sorting),
                'drop_low_variance_tasks:='+str(drop_low_variance_tasks),
                'trial_name:="'+trial_name+'"'
                ]
    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

    parent = ROSLaunchParent("run_trials", roslaunch_file, is_core=True) # run_id can be any string
    parent.start()

    rospy.init_node('run_trials', anonymous=True)

    while not rospy.is_shutdown():
        # Wait for /sampling_time_budget_elapsed to be published as True
        sampling_completed = False
        try:
            sampling_completed = rospy.wait_for_message('/sampling_time_budget_elapsed', Bool, timeout=1).data
        except rospy.ROSException:
            pass
        if sampling_completed:
            parent.shutdown()
            break