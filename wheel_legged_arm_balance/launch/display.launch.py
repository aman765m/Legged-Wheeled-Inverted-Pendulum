"""Visualise the robot model in RViz with joint sliders (no simulation)."""
import os
import re

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('wheel_legged_arm_balance')
    desc = xacro.process_file(os.path.join(share, 'urdf', 'wheel_legged_robot.urdf.xacro'),
                              mappings={'use_gazebo': 'false'}).toxml()
    desc = re.sub(r'<!--.*?-->', '', desc, flags=re.S)
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': desc}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'display.rviz')]),
    ])
