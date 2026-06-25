#!/usr/bin/env python3
# ============================================================
# bringup.launch.py — TH システム フル起動
#
# 起動オプション:
#   use_stub:=true    試験員トラッカーをスタブに切替 (デフォルト: false)
#   imu_enabled:=true IMU 入力を EKF に追加 (デフォルト: false)
#   map_yaml:=<path>  使用する地図ファイル (デフォルト: 空=SLAM マッピングモード)
# ============================================================
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                             IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap

BRINGUP_DIR  = get_package_share_directory('th_bringup')
NAV2_DIR     = get_package_share_directory('nav2_bringup')
SLAM_DIR     = get_package_share_directory('slam_toolbox')


def generate_launch_description():
    # ── 引数 ──────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('use_stub',    default_value='false',
                              description='試験員トラッカーをスタブで代替'),
        DeclareLaunchArgument('imu_enabled', default_value='false',
                              description='IMU を EKF に追加'),
        DeclareLaunchArgument('map_yaml',    default_value='',
                              description='地図 YAML パス (空=SLAM モード)'),
        DeclareLaunchArgument('log_level',   default_value='info'),
    ]

    use_stub    = LaunchConfiguration('use_stub')
    imu_enabled = LaunchConfiguration('imu_enabled')
    map_yaml    = LaunchConfiguration('map_yaml')

    # ── 設定ファイルパス ──────────────────────────────────
    nav2_yaml   = os.path.join(BRINGUP_DIR, 'config', 'nav2_params.yaml')
    ekf_yaml    = os.path.join(BRINGUP_DIR, 'config', 'ekf_params.yaml')
    slam_yaml   = os.path.join(BRINGUP_DIR, 'config', 'slam_params.yaml')
    twist_yaml  = os.path.join(get_package_share_directory('th_safety'),
                               'config', 'twist_mux.yaml')
    # キャリブ値 YAML (apply_calib で生成、存在しない場合は無視される)
    calib_yaml  = os.path.join(BRINGUP_DIR, 'config', 'calib.yaml')

    nodes = []

    # ── 1. micro-ROS Agent (ESP32 通信) ──────────────────
    nodes.append(Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/esp32', '-b', '115200',
                   '--ros-args', '--log-level', 'warn'],
        output='screen',
    ))

    # ── 2. RPLIDAR S1 ────────────────────────────────────
    nodes.append(Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'serial_port':     '/dev/lidar',
            'serial_baudrate': 256000,
            'frame_id':        'laser_link',
            'angle_compensate': True,
            'scan_mode':       'Standard',
        }],
        output='screen',
    ))

    # ── 3. lidar_filter (死角マスク) ─────────────────────
    nodes.append(Node(
        package='th_perception',
        executable='lidar_filter.py',
        name='lidar_filter',
        parameters=[os.path.join(BRINGUP_DIR, 'config', 'perception_params.yaml')],
        output='screen',
    ))

    # ── 4. esp32_bridge ───────────────────────────────────
    # calib.yaml が存在する場合は上書き
    esp32_params = [os.path.join(get_package_share_directory('th_esp32_bridge'),
                                 'config', 'params.yaml')]
    if os.path.exists(calib_yaml):
        esp32_params.append(calib_yaml)

    nodes.append(Node(
        package='th_esp32_bridge',
        executable='esp32_bridge',
        name='esp32_bridge',
        parameters=esp32_params,
        output='screen',
    ))

    # ── 5. robot_localization (EKF) ──────────────────────
    nodes.append(Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_yaml],
        output='screen',
    ))

    # ── 6. safety_monitor ─────────────────────────────────
    nodes.append(Node(
        package='th_safety',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[os.path.join(
            get_package_share_directory('th_safety'),
            'config', 'safety_monitor.yaml')],
        output='screen',
    ))

    # ── 7. twist_mux ──────────────────────────────────────
    nodes.append(Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_yaml],
        remappings=[('cmd_vel_out', '/cmd_vel')],
        output='screen',
    ))

    # ── 8. mode_manager ───────────────────────────────────
    nodes.append(Node(
        package='th_mode_manager',
        executable='mode_manager',
        name='mode_manager',
        output='screen',
    ))

    # ── 9. 試験員トラッカー (本番 or スタブ) ──────────────
    # 本番: person_tracker (ML 実装完成後に切替)
    nodes.append(Node(
        package='th_perception',
        executable='person_tracker_stub.py',
        name='person_tracker',
        condition=IfCondition(use_stub),
        parameters=[{'pattern': 'walk_forward', 'initial_x': 1.5}],
        output='screen',
    ))
    # 本番実装はここに追加:
    # nodes.append(Node(
    #     package='th_perception',
    #     executable='person_tracker',
    #     name='person_tracker',
    #     condition=UnlessCondition(use_stub),
    #     output='screen',
    # ))

    # ── 10. person_predictor ──────────────────────────────
    nodes.append(Node(
        package='th_perception',
        executable='person_predictor.py',
        name='person_predictor',
        output='screen',
    ))

    # ── 11. follow_planner ────────────────────────────────
    nodes.append(Node(
        package='th_planning',
        executable='follow_planner.py',
        name='follow_planner',
        parameters=[os.path.join(BRINGUP_DIR, 'config', 'planning_params.yaml')],
        output='screen',
    ))

    # ── 12. panel_navigator ───────────────────────────────
    nodes.append(Node(
        package='th_planning',
        executable='panel_navigator.py',
        name='panel_navigator',
        parameters=[{
            'panels_yaml': os.path.join(BRINGUP_DIR, 'config', 'panels.yaml'),
        }],
        output='screen',
    ))

    # ── 13. manual_command_handler ────────────────────────
    nodes.append(Node(
        package='th_planning',
        executable='manual_command_handler.py',
        name='manual_command_handler',
        output='screen',
    ))

    # ── 14. rosbridge (タブレット WebSocket) ──────────────
    nodes.append(Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': 9090,
            'address': '',          # 全 IF で Listen
            'ssl': False,
        }],
        output='screen',
    ))

    # ── 15. Nav2 ──────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(NAV2_DIR, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map':             map_yaml,
            'use_sim_time':    'false',
            'params_file':     nav2_yaml,
            'autostart':       'true',
        }.items(),
    )
    nodes.append(nav2_launch)

    # ── 16. SLAM Toolbox ─────────────────────────────────
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(SLAM_DIR, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': slam_yaml,
            'use_sim_time':     'false',
        }.items(),
    )
    nodes.append(slam_launch)

    return LaunchDescription(args + nodes)
