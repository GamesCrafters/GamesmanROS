#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler, quaternion_multiply
import time
from mycobot_communication.msg import MycobotGripperStatus
from math import pi

# Initialize MoveIt and ROS nodes
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("moveit_trajectory_planner", anonymous=True)

# Initialize robot commander and scene interface
robot = moveit_commander.RobotCommander()
# scene = moveit_commander.PlanningSceneInterface()

# Initialize MoveGroupCommander for your arm (replace 'arm_group' with your MoveGroup name if different)
group_name = "arm_group"  # Ensure this matches your MoveIt configuration
move_group = moveit_commander.MoveGroupCommander(group_name)
move_group.set_planner_id("LIN")

gripper = rospy.Publisher("/mycobot/gripper_status", MycobotGripperStatus, queue_size=10)

# Function to move to a specified (x, y, z) position
def plan_to_xyz(x, y, z):
    current_state = robot.get_current_state()
    move_group.set_start_state(current_state)
    # move_group.set_start_state_to_current_state()
    # move_group.setGoalPositionTolerance(0.01)
    # move_group.setGoalOrientationTolerance(0.05)
    # move_group.set_goal_position_tolerance(0.01)  # relaxed position tolerance (1 cm)
    # move_group.set_goal_orientation_tolerance(0.05)  # relaxed orientation tolerance (~2-3 degrees)


    # Set up a Pose target at the desired location
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    # Apply a 180-degree rotation around X-axis (downward)
    q1 = quaternion_from_euler(pi, 0, 0)  # Pi radians (180 degrees) around X
    q2 = quaternion_from_euler(0, 0, pi/4)
    q3 = quaternion_from_euler(-pi/16, 0, 0)      # 45° about Y
    q12 = quaternion_multiply(q1, q2)
    q = quaternion_multiply(q12, q3)


    pose_goal.orientation.x = round(q[0], 6)
    pose_goal.orientation.y = round(q[1], 6)
    pose_goal.orientation.z = round(q[2], 6)
    pose_goal.orientation.w = round(q[3], 6)

    # Set the target pose for the MoveGroup
    move_group.set_pose_target(pose_goal)

    # Plan the trajectory to the target pose
    plan = move_group.plan()

    # Check if planning succeeded
    if not plan[0]:
        rospy.logwarn("Planning failed for the target pose.")
        return False

    # Ask for confirmation before executing
    # rospy.loginfo("Trajectory planned. Check it in RVIZ.")
    # user_input = input("Enter 'y' if the trajectory looks safe on RVIZ: ")
    user_input = 'y'

    # Execute if confirmed
    if user_input.strip().lower() == 'y':
        move_group.execute(plan[1], wait=True)
        rospy.loginfo("Trajectory executed successfully.")
        return True
    else:
        rospy.logwarn("Trajectory execution canceled by user.")
        return False

# current_state = robot.get_current_state()
# move_group.set_start_state(current_state)
# # Example usage
# target_x, target_y, target_z = 0.029, 0.196, 0.25  # Replace with your target coordinates
# success = plan_to_xyz(target_x, target_y, target_z)
# time.sleep(1)

def gripper_status(state):
    gripper_states = {"open" : 1, "close" : 0}
    gripper_status = MycobotGripperStatus()
    gripper_status.Status = gripper_states[state]
    gripper.publish(gripper_status)
    gripper.publish(gripper_status)
    time.sleep(1)
    gripper.publish(gripper_status)
    gripper.publish(gripper_status)
    time.sleep(1)
    gripper.publish(gripper_status)
    gripper.publish(gripper_status)
    gripper.publish(gripper_status)
    gripper.publish(gripper_status)
    gripper.publish(gripper_status)
    gripper.publish(gripper_status)
    gripper.publish(gripper_status)
    


# time.sleep(5)
# target_x, target_y, target_z = 0.052, 0.263, 0.123  # Replace with your target coordinates
# success = plan_to_xyz(target_x, target_y, target_z)
# time.sleep(1)
# gripper_status = MycobotGripperStatus()
# gripper_status.Status = 0
# gripper.publish(gripper_status)

# target_x, target_y, target_z = 0.029, 0.196, 0.25  # Replace with your target coordinates
# success = plan_to_xyz(target_x, target_y, target_z)
# time.sleep(1)
# gripper_status = MycobotGripperStatus()
# gripper_status.Status = 0
# gripper.publish(gripper_status)

# time.sleep(5)
# target_x, target_y, target_z = 0.006, 0.265, 0.123  # Replace with your target coordinates
# success = plan_to_xyz(target_x, target_y, target_z)
# time.sleep(1)
# gripper_status = MycobotGripperStatus()
# gripper_status.Status = 1
# gripper.publish(gripper_status)

# if success:
#     rospy.loginfo("Trajectory planned and executed successfully!")
# else:
#     rospy.logwarn("Failed to plan or execute the trajectory to the target position.")

# # Shutdown MoveIt and ROS nodes
# moveit_commander.roscpp_shutdown()
