from setuptools import setup
import os
from glob import glob

package_name = 'gamesmanros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GamesmanROS',
    maintainer_email='pedorex43@gmail.com',
    description='GamesmanROS — generalized game-playing robotic system',
    license='MIT',
    entry_points={
        'console_scripts': [
            'game_manager = gamesmanros.game_manager:main',
            'robot_control = gamesmanros.robot_control:main',
            'arm_controller = gamesmanros.arm_controller:main',
            'gripper_controller = gamesmanros.gripper_controller:main',
            'joint_state_bridge = gamesmanros.joint_state_bridge:main',
            'vision_node = gamesmanros.vision_node:main',
        ],
    },
)
