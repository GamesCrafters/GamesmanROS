#!/usr/bin/env python3
import sys
import rospy
import moveit_commander

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("go_home", anonymous=True)

move_group = moveit_commander.MoveGroupCommander("arm_group")

joint_goal = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # replace with your actual home pose
move_group.go(joint_goal, wait=True)
move_group.stop()