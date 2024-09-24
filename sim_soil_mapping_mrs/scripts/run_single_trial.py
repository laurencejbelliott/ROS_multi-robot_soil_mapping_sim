#!/usr/bin/env python3
import roslaunch
import rospy
import os
from os.path import exists
from rospkg import RosPack
from roslaunch.parent import ROSLaunchParent
from std_msgs.msg import Bool

rospack = RosPack()
package_path = rospack.get_path('sim_soil_mapping_mrs')

print(package_path)

# Create bags directory if it does not exist
bags_dir = package_path + '/bags'
if not exists(bags_dir):
    os.makedirs(bags_dir)

trial_num = 3
num_robots = 3
ta_algo = "SSI"
sampling_algo = "dynamic" # "dynamic" or "grid"
env_crop_factor = 1 # Environment cropping is not yet implemented
bag_name = "test_run"
sampling_time_budget = 480 # seconds
simulator = "virtual_robots"
# bid_function = "distance_times_variance"
bid_function = "distance_over_variance_with_insertion"
use_queue_sorting = False
drop_low_variance_tasks = True
trial_name = bag_name

cli_args = [package_path+'/launch/mba_sim_'+str(num_robots)+'_rob_compaction_field.launch',
            'trial_num:='+str(trial_num),
            'num_robots:='+str(num_robots),
            'ta_algo:="'+ta_algo+'"',
            'sampling_algo:="'+sampling_algo+'"',
            'env_crop_factor:=1',
            'bag_name:="'+bag_name+'"',
            'sampling_time_budget:='+str(sampling_time_budget),
            'figures_path:="'+package_path+'/figures/test_run"',
            'simulator:='+simulator,
            'bid_function:='+bid_function,
            'use_queue_sorting:='+str(use_queue_sorting),
            'drop_low_variance_tasks:='+str(drop_low_variance_tasks),
            'trial_name:="'+trial_name+'"'
            ]

roslaunch_args = cli_args[1:]
roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

parent = ROSLaunchParent("run_trials", roslaunch_file, is_core=True)     # run_id can be any string
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