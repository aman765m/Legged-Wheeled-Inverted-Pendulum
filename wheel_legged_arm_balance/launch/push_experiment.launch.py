"""Translational push experiment (Sec. V-A, Fig. 6/7, Table 2).

ros2 launch wheel_legged_arm_balance push_experiment.launch.py force:=29 arm_mode:=active
The launch shuts down by itself once the logger has finished (sim time `duration`).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node

PKG = 'wheel_legged_arm_balance'


def generate_launch_description():
    share = get_package_share_directory(PKG)
    args = [
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('arm_mode', default_value='active'),
        DeclareLaunchArgument('payload_mass', default_value='0.36'),
        DeclareLaunchArgument('gain_source', default_value='paper'),
        DeclareLaunchArgument('wheel_torque_limit', default_value='4.5'),
        DeclareLaunchArgument('world', default_value='flat', description='flat | rough | track | path'),
        DeclareLaunchArgument('K_x', default_value=''),
        DeclareLaunchArgument('force', default_value='27.0'),
        DeclareLaunchArgument('push_duration', default_value='1.0'),
        DeclareLaunchArgument('push_time', default_value='10.0'),
        DeclareLaunchArgument('start_jitter', default_value='0.0'),
        DeclareLaunchArgument('duration', default_value='20.0'),
        DeclareLaunchArgument('output_dir', default_value=os.path.expanduser('~/wlbr_results')),
        DeclareLaunchArgument('run_name', default_value='push'),
    ]
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'sim.launch.py')),
        launch_arguments={'gui': LC('gui'), 'rviz': LC('rviz'), 'arm_mode': LC('arm_mode'),
                          'payload_mass': LC('payload_mass'),
                          'gain_source': LC('gain_source'),
                          'world': LC('world'),
                          'wheel_torque_limit': LC('wheel_torque_limit'),
                          'K_x': LC('K_x')}.items())
    push = Node(package=PKG, executable='push_disturbance.py', output='screen',
                parameters=[{'use_sim_time': True, 'force': LC('force'),
                             'duration': LC('push_duration'), 'start_time': LC('push_time'),
                             'start_jitter': LC('start_jitter')}])
    logger = Node(package=PKG, executable='experiment_logger.py', output='screen',
                  parameters=[{'use_sim_time': True, 'output_dir': LC('output_dir'),
                               'run_name': LC('run_name'), 'duration': LC('duration'),
                               'event_time': LC('push_time')}])
    stop = RegisterEventHandler(OnProcessExit(target_action=logger,
                                              on_exit=[EmitEvent(event=Shutdown())]))
    return LaunchDescription(args + [sim, push, logger, stop])
