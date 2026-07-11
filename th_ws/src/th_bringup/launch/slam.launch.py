#!/usr/bin/env python3
# ============================================================
# slam.launch.py — 地図作成専用起動
#
# 使い方:
#   ros2 launch th_bringup slam.launch.py
#   # RViz2 で /map を表示しながらロボットを手動で走らせる
#   # 完了後:
#   ros2 run nav2_map_server map_saver_cli -f ~/maps/th_map
#
#   lidar_source:=local    USB直結のsllidar_nodeを起動 (デフォルト)
#   lidar_source:=network  ラズパイ等が配信する/scanを使用 (ローカル起動なし。
#                          Pi側とROS_DOMAIN_IDを一致させること)
# ============================================================
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

BRINGUP_DIR = get_package_share_directory('th_bringup')
SLAM_DIR    = get_package_share_directory('slam_toolbox')
DESC_DIR    = get_package_share_directory('th_description')


def generate_launch_description():
    slam_yaml = os.path.join(BRINGUP_DIR, 'config', 'slam_params.yaml')
    calib_yaml = os.path.join(BRINGUP_DIR, 'config', 'calib.yaml')

    esp32_params = [os.path.join(
        get_package_share_directory('th_esp32_bridge'), 'config', 'params.yaml')]
    if os.path.exists(calib_yaml):
        esp32_params.append(calib_yaml)

    lidar_source = LaunchConfiguration('lidar_source')
    lidar_is_local = PythonExpression(["'", lidar_source, "' == 'local'"])

    # base_link → laser_link 等の固定 TF を配信する。これが無いと
    # SLAM Toolbox がスキャンを座標変換できず地図が作れない。
    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(DESC_DIR, 'urdf', 'th_robot.urdf.xacro')]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('lidar_source', default_value='local',
                              description='local=USB直結sllidar_node起動 / '
                                          'network=ラズパイ等が配信する/scanを使用'),

        # robot_state_publisher / joint_state_publisher (URDF → TF)
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher',
             parameters=[{'robot_description': robot_description}],
             output='screen'),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='joint_state_publisher', output='screen'),

        # RPLIDAR (lidar_source:=local の場合のみ起動)
        Node(package='sllidar_ros2', executable='sllidar_node',
             condition=IfCondition(lidar_is_local),
             parameters=[{'serial_port': '/dev/lidar', 'serial_baudrate': 256000,
                          'frame_id': 'laser_link', 'angle_compensate': True}],
             output='screen'),
        LogInfo(
            condition=UnlessCondition(lidar_is_local),
            msg='lidar_source=network: ローカルsllidar_nodeは起動しません。'
                'ラズパイ側の/scanを受信するにはROS_DOMAIN_IDが一致している必要があります。',
        ),

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
