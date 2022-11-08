#!/usr/bin/python
__author__ = "Laurence Roberts-Elliott"
import rospy
import math
import actionlib
import pickle
from std_msgs.msg import String
# TODO: Add rospy_message_converter to dependencies
from rospy_message_converter import message_converter
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseArray


def pen_sample(msg):
    global mock_pen_data

    pose_x = msg.position.x
    pose_y = msg.position.y

    data_x = math.floor(pose_x)
    data_y = math.floor(pose_y)

    # TODO: Namespace so that multiple robots can receive sampling tasks
    client = actionlib.SimpleActionClient('robot_1/move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = pose_x
    goal.target_pose.pose.position.y = pose_y
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    rospy.loginfo("Sending navigation goal for sampling position...")
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Cannot reach navigation server!")
    else:
        state = client.get_state()
        if state == 3:
            # If nav. was successful, stop for 5 seconds and read data indexed by position
            rospy.loginfo("Taking penetrometer reading...")
            rospy.sleep(5)
            mock_reading = mock_pen_data[data_y, data_x]
            reading_dict = {'x': pose_x, 'y': pose_y, 'reading': mock_reading}
            rospy.loginfo(str(reading_dict))
            reading_msg = message_converter.convert_dictionary_to_ros_message('std_msgs/String',
                                                                              {'data': str(reading_dict)})
            mock_pen_reading_pub.publish(reading_msg)


        else:
            rospy.logerr("Navigation goal failed!")


# Subscribe to the sampling location topic, and navigate to
# and sample from the given location
if __name__ == "__main__":
    node_name = "sim_soil_sampling"
    amcl_pose_array = PoseArray()

    rospy.init_node(node_name)

    # Load ground-truth penetrometer data
    with open('../include/stage_soil_mapping_mrs/ground_truth/interpolated_jaime_compaction_0cm_kpas.pickle',
              'rb') as f:
        mock_pen_data = pickle.load(f)

    rospy.Subscriber(node_name+"/pen_sample_pose", Pose, callback=pen_sample)
    mock_pen_reading_pub = rospy.Publisher(node_name + "/mock_pen_reading", String, queue_size=1)
    rospy.spin()
