#!/usr/bin/env python3
# ============================================================
# slam.launch.py — 地図作成専用起動
#
# 使い方:
#   ros2 launch th_bringup slam.launch.py
#   # RViz2 で /map を表示しながらロボットを手動で走らせる
#   # 完了後:
#   ros2 run nav2_map_server map_saver_cli -f ~/maps/th_map
# ============================================================
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

BRINGUP_DIR = get_package_share_directory('th_bringup')
SLAM_DIR    = get_package_share_directory('slam_toolbox')


def generate_launch_description():
    slam_yaml = os.path.join(BRINGUP_DIR, 'config', 'slam_params.yaml')
    calib_yaml = os.path.join(BRINGUP_DIR, 'config', 'calib.yaml')

    esp32_params = [os.path.join(
        get_package_share_directory('th_esp32_bridge'), 'config', 'params.yaml')]
    if os.path.exists(calib_yaml):
        esp32_params.append(calib_yaml)

    return LaunchDescription([
        # RPLIDAR
        Node(package='sllidar_ros2', executable='sllidar_node',
             parameters=[{'serial_port': '/dev/lidar', 'serial_baudrate': 256000,
                          'frame_id': 'laser_link', 'angle_compensate': True}],
             output='screen'),

        # lidar_filter
        Node(package='th_perception', executable='lidar_filter.py',
             name='lidar_filter', output='screen'),

        # esp32_bridge
        Node(package='th_esp32_bridge', executable='esp32_bridge.py',
             name='esp32_bridge', parameters=esp32_params, output='screen'),

        # EKF
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node',
             parameters=[os.path.join(BRINGUP_DIR, 'config', 'ekf_params.yaml')],
             output='screen'),

        # twist_mux (地図作成中の手動操作用)
        Node(package='twist_mux', executable='twist_mux',
             parameters=[os.path.join(
                 get_package_share_directory('th_safety'), 'config', 'twist_mux.yaml')],
             remappings=[('cmd_vel_out', '/cmd_vel')], output='screen'),

        # safety_monitor
        Node(package='th_safety', executable='safety_monitor',
             name='safety_monitor', output='screen'),

        # rosbridge (タブレットから手動操作するため)
        Node(package='rosbridge_server', executable='rosbridge_websocket',
             parameters=[{'port': 9090}], output='screen'),

        # SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(SLAM_DIR, 'launch', 'online_async_launch.py')),
            launch_arguments={
                'slam_params_file': slam_yaml,
                'use_sim_time': 'false',
            }.items(),
        ),
    ])
