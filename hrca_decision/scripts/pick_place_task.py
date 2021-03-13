#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import String

from hrca_action.utilities import *
from hrca_action.panda_arm import PandaArm 
from hrca_msgs.msg import RobotTaskAction, RobotTaskFeedback, RobotTaskResult, RobotTaskGoal
from hrca_msgs.srv import *
import actionlib

if __name__ == '__main__':
    rospy.init_node("pick_place_task")

    print u"Resetting gazebo world"
    rospy.wait_for_service('/gazebo/reset_world')
    reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    reset_world()

    # Perform action
    pick_poses = []
    pick_poses.append(create_pose_stamped(create_pose(0.55, 0, 0.02, 0, 1, 0, 0), "panda_link0"))
    pick_poses.append(create_pose_stamped(create_pose(0.55, -0.1, 0.528, 0.9, -0.41, 0, 0), "panda_link0"))

    place_poses = []
    place_poses.append(create_pose_stamped(create_pose(0.55, -0.4, 0.02, 0, 1, 0, 0), "panda_link0"))
    place_poses.append(create_pose_stamped(create_pose(0.55, -0.4, 0.55, 0.9, -0.41, 0, 0), "panda_link0"))


    rospy.loginfo("Waiting for panda arm server")
    client = actionlib.SimpleActionClient(u'panda_arm_server', RobotTaskAction)
    client.wait_for_server()
    rospy.loginfo("Succesfully connected to panda arm server")

    goal = RobotTaskGoal()

    item_index = 0

    goal.action = u"pick_and_place"
    goal.pose1 = pick_poses[item_index]
    goal.pose2 = place_poses[item_index]
    goal.object_name = u"object_" + unicode(item_index)

    client.send_goal(goal)
    client.wait_for_result()
    