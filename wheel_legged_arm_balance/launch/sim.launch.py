"""Gazebo Classic simulation of the wheel-legged robot with arm (ROS 2 Humble).

ros2 launch wheel_legged_arm_balance sim.launch.py [gui:=true] [rviz:=true]
    [arm_mode:=active|fixed] [payload_mass:=0.36] [gain_source:=paper|computed]
    [wheel_torque_limit:=4.5] [K_x:=6.0]
"""
import math
import os
import re
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'wheel_legged_arm_balance'
WORLDS = {'flat': 'flat_ground.world', 'rough': 'rough_terrain.world', 'track': 'figure8_track.world'}


def resolve_world(share, world):
    """Map flat|rough|track to the package worlds and fill in the __PKG_SHARE__ placeholder."""
    path = os.path.join(share, 'worlds', WORLDS[world]) if world in WORLDS else world
    with open(path) as f:
        sdf = f.read()
    if '__PKG_SHARE__' not in sdf:
        return path
    fd, out = tempfile.mkstemp(prefix='wlbr_world_', suffix='.world')
    with os.fdopen(fd, 'w') as f:
        f.write(sdf.replace('__PKG_SHARE__', share))
    return out


def _setup(context):
    share = get_package_share_directory(PKG)
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    payload = float(lc('payload_mass'))

    # static equilibrium: the wheel-axis -> CoM line must be vertical (Table 1 values)
    m_c, l, r_w, h_s = 7.5, 0.5914, 0.1016, 0.23
    m1, m2, q1, q2 = 0.6, 1.13, math.pi / 3, -2 * math.pi / 3
    x_arm = (m1 * 0.3 * math.cos(q1) + (m2 + payload) * (0.3 * math.cos(q1) + 0.3 * math.cos(q1 + q2)))
    m_tot = m_c + m1 + m2 + payload
    pitch0 = -math.atan2(x_arm / m_tot, (m_c * l + (m1 + m2 + payload) * (l + h_s)) / m_tot)
    spawn_z = r_w + l * math.cos(pitch0) + 0.002
    spawn_dx = l * math.sin(pitch0)
    yaw0 = float(lc('yaw'))

    # controllers.yaml with the launch-time overrides
    with open(os.path.join(share, 'config', 'controllers.yaml')) as f:
        ctrl = yaml.safe_load(f)
    lqr = ctrl['lqr_balance_controller']['ros__parameters']
    lqr['payload_mass'] = payload
    # Gazebo Classic reports the IMU orientation relative to the spawn pose
    imu_ref = [0.0, pitch0, yaw0]
    lqr['imu_reference_rpy'] = imu_ref
    lqr['gain_source'] = lc('gain_source')
    lqr['torque_limit'] = float(lc('wheel_torque_limit'))
    lqr['model']['lump_arm'] = lc('lump_arm').lower() == 'true'
    ctrl['rac_arm_controller']['ros__parameters']['payload_mass'] = payload
    fd, ctrl_file = tempfile.mkstemp(prefix='wlbr_controllers_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(ctrl, f)

    robot_description = xacro.process_file(
        os.path.join(share, 'urdf', 'wheel_legged_robot.urdf.xacro'),
        mappings={'payload_mass': str(payload), 'controllers_file': ctrl_file,
                  'use_gazebo': 'true'}).toxml()
    # XML comments break the --param parsing of gazebo_ros2_control
    robot_description = re.sub(r'<!--.*?-->', '', robot_description, flags=re.S)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': resolve_world(share, lc('world')), 'gui': lc('gui'), 'pause': 'true',
                          'verbose': lc('verbose')}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
               output='screen')
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                 arguments=['-topic', 'robot_description', '-entity', 'wheel_legged_robot',
                            '-timeout', '180',
                            '-x', str(float(lc('x')) + spawn_dx * math.cos(yaw0)),
                            '-y', str(float(lc('y')) + spawn_dx * math.sin(yaw0)),
                            '-z', str(spawn_z), '-P', str(pitch0), '-Y', str(yaw0)])
    controllers = ['joint_state_broadcaster', 'imu_sensor_broadcaster',
                   'lqr_balance_controller', 'rac_arm_controller']
    spawner = Node(package='controller_manager', executable='spawner', output='screen',
                   arguments=controllers + ['--inactive', '--controller-manager-timeout', '120'])
    starter = Node(package=PKG, executable='sim_starter.py', output='screen',
                   parameters=[{'controllers': controllers}])

    fsm_cfg = os.path.join(share, 'config', 'arm_fsm.yaml')
    fsm = Node(package=PKG, executable='arm_fsm_node', name='arm_fsm', output='screen',
               parameters=[fsm_cfg, {'use_sim_time': True, 'mode': lc('arm_mode'),
                                     'imu_reference_rpy': imu_ref}] +
                          ([{'K_x': float(lc('K_x'))}] if lc('K_x') else []),
               remappings=[('imu', '/imu_sensor_broadcaster/imu'),
                           ('cmd_vel', '/lqr_balance_controller/cmd_vel'),
                           ('arm_reference', '/rac_arm_controller/reference')])
    cmp = Node(package=PKG, executable='cmp_estimator_node', name='cmp_estimator', output='screen',
               parameters=[fsm_cfg, {'use_sim_time': True}])
    rviz = Node(package='rviz2', executable='rviz2', output='log',
                arguments=['-d', os.path.join(share, 'rviz', 'sim.rviz')],
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(lc('rviz')))
    return [gazebo, rsp, spawn, spawner, starter, fsm, cmp, rviz]


def generate_launch_description():
    share = get_package_share_directory(PKG)
    args = [
        DeclareLaunchArgument('gui', default_value='true', description='Start gzclient'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('world', default_value='flat',
                              description='flat | rough | track | path to a .world file'),
        DeclareLaunchArgument('arm_mode', default_value='active', description='active | fixed'),
        DeclareLaunchArgument('payload_mass', default_value='0.36',
                              description='mass of the can held by the arm [kg] (0.36 / 0.72)'),
        DeclareLaunchArgument('gain_source', default_value='paper', description='paper | computed'),
        DeclareLaunchArgument('lump_arm', default_value='false'),
        DeclareLaunchArgument('K_x', default_value='', description='override of the FSM arm gain'),
        DeclareLaunchArgument('wheel_torque_limit', default_value='4.5',
                              description='saturation of the wheel torques [Nm]'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]
    # everything is local: never wait for the online model database
    no_db = SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', '')
    return LaunchDescription(args + [no_db, OpaqueFunction(function=_setup)])
