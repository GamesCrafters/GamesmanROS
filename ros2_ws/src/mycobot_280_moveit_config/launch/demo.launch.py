"""
MoveIt 2 demo for the MyCobot 280 on mock hardware (no robot needed).

Starts: robot_state_publisher, ros2_control (mock_components) with
joint_state_broadcaster + joint_trajectory_controller, move_group, RViz.

Two planning pipelines are enabled:
  ompl                             joint-space planning (RRTConnect) — fallback
  pilz_industrial_motion_planner   straight-line LIN planning — used first

  ros2 launch mycobot_280_moveit_config demo.launch.py
  ros2 launch mycobot_280_moveit_config demo.launch.py hardware:=true   # real arm

With hardware:=true the mock ros2_control stack is NOT started: the real arm's
driver (mycobot_topics) publishes /joint_states via joint_state_bridge, and
joint_trajectory serves the follow_joint_trajectory action (both start from
gamesmanros.launch.py hardware:=true).

moveit_parameters() is shared: gamesmanros.launch.py loads this file and passes
the same parameters to arm_controller's MoveItPy instance.
"""

import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'mycobot_280_moveit_config'

OMPL_PIPELINE = 'ompl'
PILZ_PIPELINE = 'pilz_industrial_motion_planner'


def _path(*parts):
    return os.path.join(get_package_share_directory(PKG), *parts)


def _load_yaml(*parts):
    with open(_path(*parts), 'r') as f:
        return yaml.safe_load(f)


def _load_text(*parts):
    with open(_path(*parts), 'r') as f:
        return f.read()


def moveit_parameters():
    """Every MoveIt parameter a node needs to plan and execute, as launch dicts."""
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

    # Joint limits + Pilz Cartesian limits share one parameter namespace.
    planning_limits = _load_yaml('config', 'joint_limits.yaml')
    planning_limits.update(_load_yaml('config', 'pilz_cartesian_limits.yaml'))
    robot_description_planning = {'robot_description_planning': planning_limits}

    # Each pipeline's settings live under a namespace named after the pipeline.
    planning_pipelines = {
        'planning_pipelines': {'pipeline_names': [OMPL_PIPELINE, PILZ_PIPELINE]},
        'default_planning_pipeline': OMPL_PIPELINE,
        OMPL_PIPELINE: _load_yaml('config', 'ompl_planning.yaml'),
        PILZ_PIPELINE: _load_yaml('config', 'pilz_planning.yaml'),
    }

    moveit_controllers = {
        'moveit_simple_controller_manager': _load_yaml(
            'config', 'moveit_controllers.yaml')['moveit_simple_controller_manager'],
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }
    trajectory_execution = {
        'moveit_manage_controllers': True,
        # MoveIt aborts a trajectory that runs longer than its planned duration x scaling
        # + margin. On the real arm joint_trajectory deliberately waits for the arm to
        # arrive, which routinely exceeds that, so the monitor is off (joint_trajectory
        # bounds the wait itself with `settle_timeout`).
        'trajectory_execution.execution_duration_monitoring': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        # 0.01 rad is tighter than a real servo arm holds its pose (sag, backlash,
        # 4 Hz joint states), which makes MoveIt refuse to start a trajectory.
        'trajectory_execution.allowed_start_tolerance': 0.05,
    }
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    return [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        planning_pipelines,
        moveit_controllers,
        trajectory_execution,
        planning_scene_monitor_parameters,
        _load_yaml('config', 'moveit_py.yaml'),   # MoveItPy; harmless to move_group
    ]


def generate_launch_description():
    hardware_arg = DeclareLaunchArgument(
        'hardware', default_value='false',
        description='true = real arm: do not start the mock ros2_control stack')
    mock_only = UnlessCondition(LaunchConfiguration('hardware'))

    params = moveit_parameters()
    robot_description = params[0]

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
        condition=mock_only,
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=mock_only,
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=mock_only,
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=params,
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', _path('config', 'moveit.rviz')],
        parameters=params[:5],     # description, semantic, kinematics, limits, pipelines
    )

    return LaunchDescription([
        hardware_arg,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        move_group,
        rviz,
    ])
