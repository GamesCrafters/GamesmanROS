"""
GamesmanROS launch file.

Bringup order:
  1. joint_state_bridge  — runs on Pi; bridges MyCobot degrees → /joint_states
  2. MoveIt 2            — loaded separately via moveit launch (not here)
  3. arm_controller      — MoveIt 2 motion action server
  4. gripper_controller  — gripper action server
  5. vision_node         — AR tag TF2 listener (optional, set vision:=true)
  6. robot_control       — Type class executor, depends on arm + gripper
  7. game_manager        — UWAPI game loop, depends on robot_control

Usage:
  ros2 launch gamesmanros gamesmanros.launch.py
  ros2 launch gamesmanros gamesmanros.launch.py vision:=true
  ros2 launch gamesmanros gamesmanros.launch.py game_id:=dao
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    vision_arg = DeclareLaunchArgument(
        'vision', default_value='false',
        description='Enable vision node (requires AR tag detection backend)')

    game_id_arg = DeclareLaunchArgument(
        'game_id', default_value='',
        description='Pre-select game ID (leave empty for interactive selection)')

    vision_enabled = LaunchConfiguration('vision')
    game_id        = LaunchConfiguration('game_id')

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
            'grid_rows':    4,
            'grid_cols':    4,
            'debug_view':   False,
            # HSV ranges tuned for default red/blue pieces — override at runtime
            'p1_hsv_lower': [0,   120, 70],
            'p1_hsv_upper': [10,  255, 255],
            'p2_hsv_lower': [100, 120, 70],
            'p2_hsv_upper': [130, 255, 255],
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
        joint_state_bridge,
        arm_controller,
        gripper_controller,
        vision_node,
        robot_control,
        game_manager,
    ])
