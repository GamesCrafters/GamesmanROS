"""
MoveIt 2 demo for the MyCobot 280 on mock hardware (no robot needed).

Starts: robot_state_publisher, ros2_control (mock_components) with
joint_state_broadcaster + joint_trajectory_controller, move_group (OMPL), RViz.

  ros2 launch mycobot_280_moveit_config demo.launch.py
"""

import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

PKG = 'mycobot_280_moveit_config'


def _path(*parts):
    return os.path.join(get_package_share_directory(PKG), *parts)


def _load_yaml(*parts):
    with open(_path(*parts), 'r') as f:
        return yaml.safe_load(f)


def _load_text(*parts):
    with open(_path(*parts), 'r') as f:
        return f.read()


def generate_launch_description():
    robot_description = {
        'robot_description': xacro.process_file(
            _path('config', 'mycobot.urdf.xacro')).toxml()
    }
    robot_description_semantic = {
        'robot_description_semantic': _load_text('config', 'mycobot.srdf')
    }
    robot_description_kinematics = {
        'robot_description_kinematics': _load_yaml('config', 'kinematics.yaml')
    }
    robot_description_planning = {
        'robot_description_planning': _load_yaml('config', 'joint_limits.yaml')
    }

    # OMPL is the default (and only) planning pipeline.
    ompl_planning_pipeline_config = {
        'move_group': _load_yaml('config', 'ompl_planning.yaml')
    }

    moveit_controllers = {
        'moveit_simple_controller_manager': _load_yaml(
            'config', 'moveit_controllers.yaml')['moveit_simple_controller_manager'],
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='both',
        parameters=[robot_description, _path('config', 'ros2_controllers.yaml')],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', _path('config', 'moveit.rviz')],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        move_group,
        rviz,
    ])
