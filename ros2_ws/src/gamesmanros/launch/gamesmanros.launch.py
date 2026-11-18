"""
GamesmanROS launch file.

Bringup order:
  1. joint_state_bridge  — runs on Pi; bridges MyCobot degrees → /joint_states
  2. MoveIt 2            — ros2_control + move_group + RViz via
                           `ros2 launch mycobot_280_moveit_config demo.launch.py` (not here);
                           arm_controller's own MoveItPy params are loaded below
  3. arm_controller      — MoveIt 2 motion action server
  4. gripper_controller  — gripper action server
  5. vision_node         — AR tag TF2 listener (optional, set vision:=true)
     mycobot_topics + joint_trajectory — only with hardware:=true (real arm)
  6. robot_control       — Type class executor, depends on arm + gripper
  7. game_manager        — UWAPI game loop, depends on robot_control

Usage:
  ros2 launch gamesmanros gamesmanros.launch.py
  ros2 launch gamesmanros gamesmanros.launch.py vision:=true
  ros2 launch gamesmanros gamesmanros.launch.py game_id:=dao
  ros2 launch gamesmanros gamesmanros.launch.py hardware:=true port:=/dev/ttyAMA0 baud:=1000000
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue


def _moveit_parameters():
    """MoveIt parameters for arm_controller's MoveItPy instance.

    Loaded from mycobot_280_moveit_config's demo.launch.py so that move_group and
    arm_controller are always configured identically (both pipelines: ompl + pilz).
    """
    path = os.path.join(get_package_share_directory('mycobot_280_moveit_config'),
                        'launch', 'demo.launch.py')
    spec = importlib.util.spec_from_file_location('mycobot_280_demo_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.moveit_parameters()


def generate_launch_description():
    vision_arg = DeclareLaunchArgument(
        'vision', default_value='false',
        description='Enable vision node (requires AR tag detection backend)')

    game_id_arg = DeclareLaunchArgument(
        'game_id', default_value='',
        description='Pre-select game ID (leave empty for interactive selection)')

    variant_id_arg = DeclareLaunchArgument(
        'variant_id', default_value='regular',
        description='Board variant; vision_node fetches cell layout for (game_id, variant_id)')

    eef_orientation_arg = DeclareLaunchArgument(
        'eef_orientation', default_value='full',
        description="arm_controller gripper pose: 'full' (rotated+tilted) or 'down' (straight down)")

    hardware_arg = DeclareLaunchArgument(
        'hardware', default_value='false',
        description='Real arm: also start mycobot_topics and joint_trajectory')
    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ttyUSB0',
        description='Arm serial port (Pi-mounted arm: /dev/ttyAMA0)')
    baud_arg = DeclareLaunchArgument(
        'baud', default_value='115200',
        description='Arm baud rate (Pi-mounted arm: 1000000)')

    vision_enabled = LaunchConfiguration('vision')
    hardware       = LaunchConfiguration('hardware')
    eef_orientation = LaunchConfiguration('eef_orientation')
    game_id        = LaunchConfiguration('game_id')
    variant_id     = LaunchConfiguration('variant_id')

    joint_state_bridge = Node(
        package='gamesmanros',
        executable='joint_state_bridge',
        name='joint_state_bridge',
        output='screen',
    )

    arm_controller = Node(
        package='gamesmanros',
        executable='arm_controller',
        name='arm_controller',
        output='screen',
        parameters=_moveit_parameters() + [{'game_id': game_id,
                                            'eef_orientation': eef_orientation}],
    )

    mycobot_topics = Node(
        package='gamesmanros',
        executable='mycobot_topics',
        name='mycobot_topics',
        output='screen',
        condition=IfCondition(hardware),
        parameters=[{'port': LaunchConfiguration('port'),
                     'baud': ParameterValue(LaunchConfiguration('baud'), value_type=int)}],
    )

    joint_trajectory = Node(
        package='gamesmanros',
        executable='joint_trajectory',
        name='joint_trajectory',
        output='screen',
        condition=IfCondition(hardware),
    )

    gripper_controller = Node(
        package='gamesmanros',
        executable='gripper_controller',
        name='gripper_controller',
        output='screen',
    )

    vision_node = Node(
        package='gamesmanros',
        executable='vision_node',
        name='vision_node',
        output='screen',
        condition=IfCondition(vision_enabled),
        parameters=[{
            'camera_index': 0,
            # Cell layout is fetched from UWAPI for this (game, variant), so
            # irregular and non-grid boards need no extra configuration.
            'game':         game_id,
            'variant':      variant_id,
            'snap_radius':  0.12,
            'debug_view':   False,
            # TODO(calibrate): assign real tag IDs. These must not overlap
            # corner_ids (default [0,1,2,3]) — see docs/MERGE-ISSUES.md.
            'p1_tag_ids':   [],
            'p2_tag_ids':   [],
        }],
    )

    robot_control = Node(
        package='gamesmanros',
        executable='robot_control',
        name='robot_control',
        output='screen',
        parameters=[{'game_id': game_id}],
    )

    game_manager = LifecycleNode(
        package='gamesmanros',
        executable='game_manager',
        name='game_manager',
        namespace='',
        output='screen',
    )

    return LaunchDescription([
        vision_arg,
        game_id_arg,
        variant_id_arg,
        hardware_arg,
        port_arg,
        baud_arg,
        eef_orientation_arg,
        joint_state_bridge,
        mycobot_topics,
        joint_trajectory,
        arm_controller,
        gripper_controller,
        vision_node,
        robot_control,
        game_manager,
    ])
