#!/usr/bin/env python3
import roslaunch
import rospy
import itertools
from rospkg import RosPack
from roslaunch.parent import ROSLaunchParent
from std_msgs.msg import Bool

rospack = RosPack()
package_path = rospack.get_path('stage_soil_mapping_mrs')

print(package_path)

trial_num = 1
num_robots = 3
ta_algo = "SSI"
sampling_algo = "random"
env_crop_factor = 1 # Environment cropping is not yet implemented

# cli_args = [package_path+'/launch/simplified_nav_'+str(num_robots)+'_rob_compaction_field.launch',
#             'trial_num:='+str(trial_num),
#             'num_robots:='+str(num_robots),
#             'ta_algo:="'+ta_algo+'"',
#             'sampling_algo:="'+sampling_algo+'"',
#             'env_crop_factor:=1',
#             'bag_name:="'+str(num_robots)+'_robs_'+str(ta_algo)+'_'+str(sampling_algo)+'_'+str(trial_num)+'"'
#             ]
# roslaunch_args = cli_args[1:]
# roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

# # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

# parent = ROSLaunchParent("run_trials", roslaunch_file, is_core=True)     # run_id can be any string
# parent.start()

# rospy.init_node('run_trials', anonymous=True)

# while not rospy.is_shutdown():
#     # Wait for /sampling_time_budget_elapsed to be published as True
#     sampling_completed = False
#     try:
#         sampling_completed = rospy.wait_for_message('/sampling_time_budget_elapsed', Bool, timeout=1).data
#     except rospy.ROSException:
#         pass
#     if sampling_completed:
#         parent.shutdown()
#         break

# Define sets of parameters to combine
trial_num_set = set([i for i in range(1, 11)])
print("Set of trial numbers: ", trial_num_set)

num_robots_set = set([num_robots])
print("Set of number of robots: ", num_robots_set)

ta_algo_set = set(['RR', 'SSI'])
print("Set of task allocation algorithms: ", ta_algo_set)

sampling_algo_set = set(["grid", "random"])
print("Set of sampling algorithms: ", sampling_algo_set)

env_crop_factor_set = set([1])
print("Set of environment cropping factors: ", env_crop_factor_set)

# Create all combinations of trial_num, num_robots, ta_algo, sampling_algo, env_crop_factor
combinations = list(itertools.product(trial_num_set, num_robots_set, ta_algo_set, sampling_algo_set, env_crop_factor_set))
# Remove combinations that have no trial_num, num_robots, ta_algo, sampling_algo, env_crop_factor
combinations = [i for i in combinations if i[0] != i[1] != i[2] != i[3] != i[4]]


print("Set of trial numbers, num_robots, ta_algo, sampling_algo, env_crop_factor combinations:\n", combinations)

for trial_params in combinations:
    trial_num = trial_params[0]
    num_robots = trial_params[1]
    ta_algo = trial_params[2]
    sampling_algo = trial_params[3]
    env_crop_factor = trial_params[4]

    cli_args = [package_path+'/launch/simplified_nav_'+str(num_robots)+'_rob_compaction_field.launch',
                'trial_num:='+str(trial_num),
                'num_robots:='+str(num_robots),
                'ta_algo:="'+ta_algo+'"',
                'sampling_algo:="'+sampling_algo+'"',
                'env_crop_factor:=1',
                'bag_name:="'+str(num_robots)+'_robs_'+str(ta_algo)+'_'+str(sampling_algo)+'_'+str(trial_num)+'"'
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