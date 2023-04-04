#!/usr/bin/env python3
import roslaunch
import rospy
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
sampling_time_budget = 240 # seconds

cli_args = [package_path+'/launch/simplified_nav_'+str(num_robots)+'_rob_compaction_field.launch',
            'trial_num:='+str(trial_num),
            'num_robots:='+str(num_robots),
            'ta_algo:="'+ta_algo+'"',
            'sampling_algo:="'+sampling_algo+'"',
            'env_crop_factor:=1',
            'bag_name:="metrics"',
            'sampling_time_budget:='+str(sampling_time_budget)
            ]
roslaunch_args = cli_args[1:]
roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

# uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

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