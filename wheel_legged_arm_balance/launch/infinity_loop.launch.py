"""Infinity-loop trajectory experiment (Sec. V-B, Fig. 8).

normal speed : ros2 launch wheel_legged_arm_balance infinity_loop.launch.py v_max:=0.3 w_max:=0.3
high speed   : ros2 launch wheel_legged_arm_balance infinity_loop.launch.py v_max:=0.7 w_max:=0.6
on the track : ros2 launch wheel_legged_arm_balance infinity_loop.launch.py world:=track
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
from launch.substitutions import PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node

PKG = 'wheel_legged_arm_balance'


def generate_launch_description():
    share = get_package_share_directory(PKG)
    args = [
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('arm_mode', default_value='active'),
        DeclareLaunchArgument('payload_mass', default_value='0.36'),
        DeclareLaunchArgument('gain_source', default_value='paper'),
        DeclareLaunchArgument('wheel_torque_limit', default_value='4.5'),
        DeclareLaunchArgument('world', default_value='flat', description='flat | rough | track | path'),
        DeclareLaunchArgument('v_max', default_value='0.3'),
        DeclareLaunchArgument('w_max', default_value='0.3'),
        DeclareLaunchArgument('size', default_value='4.0'),
        DeclareLaunchArgument('laps', default_value='1.0'),
        DeclareLaunchArgument('duration', default_value='120.0'),
        DeclareLaunchArgument('output_dir', default_value=os.path.expanduser('~/wlbr_results')),
        DeclareLaunchArgument('run_name', default_value='loop'),
        # robot: path starts at the robot's spawn pose; world: path fixed in the world
        # (the figure-8 track of world:=track). Defaults to world on the track.
        DeclareLaunchArgument('path_anchor', default_value=PythonExpression(
            ["'world' if '", LC('world'), "' == 'track' else 'robot'"])),
    ]
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'sim.launch.py')),
        launch_arguments={'gui': LC('gui'), 'rviz': LC('rviz'), 'arm_mode': LC('arm_mode'),
                          'payload_mass': LC('payload_mass'),
                          'gain_source': LC('gain_source'),
                          'world': LC('world'),
                          'wheel_torque_limit': LC('wheel_torque_limit')}.items())
    tracker = Node(package=PKG, executable='infinity_loop_tracker.py', output='screen',
                   parameters=[{'use_sim_time': True, 'max_linear_velocity': LC('v_max'),
                                'max_angular_velocity': LC('w_max'), 'size': LC('size'),
                                'laps': LC('laps'), 'path_anchor': LC('path_anchor'),
                                'target_csv': PathJoinSubstitution(
                                    [LC('output_dir'), [LC('run_name'), '_target.csv']])}])
    logger = Node(package=PKG, executable='experiment_logger.py', output='screen',
                  parameters=[{'use_sim_time': True, 'output_dir': LC('output_dir'),
                               'run_name': LC('run_name'), 'duration': LC('duration'),
                               'event_time': 5.0}])
    # stop when the loop is completed (or the logger reaches `duration`)
    stop = [RegisterEventHandler(OnProcessExit(target_action=a, on_exit=[EmitEvent(event=Shutdown())]))
            for a in (tracker, logger)]
    return LaunchDescription(args + [sim, tracker, logger] + stop)
