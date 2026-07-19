#!/usr/bin/env python3
# ============================================================
# bringup.launch.py — TH システム フル起動
#
# 起動オプション:
#   use_stub:=true    試験員トラッカーをスタブに切替 (デフォルト: false)
#   imu_enabled:=true IMU 入力を EKF に追加 (デフォルト: false)
#   map_yaml:=<path>  使用する地図ファイル (デフォルト: 空=SLAM マッピングモード)
#   lidar_source:=local    USB直結のsllidar_nodeを起動 (デフォルト)
#   lidar_source:=network  ラズパイ等が配信する/scanを使用 (ローカル起動なし。
#                          Pi側とROS_DOMAIN_IDを一致させること)
# ============================================================
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                             IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration,
                                   PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue

BRINGUP_DIR  = get_package_share_directory('th_bringup')
DESC_DIR     = get_package_share_directory('th_description')
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
        DeclareLaunchArgument('lidar_source', default_value='local',
                              description='local=USB直結sllidar_node起動 / '
                                          'network=ラズパイ等が配信する/scanを使用'),
    ]

    use_stub     = LaunchConfiguration('use_stub')
    imu_enabled  = LaunchConfiguration('imu_enabled')
    map_yaml     = LaunchConfiguration('map_yaml')
    lidar_source = LaunchConfiguration('lidar_source')
    lidar_is_local = PythonExpression(["'", lidar_source, "' == 'local'"])

    # ── 設定ファイルパス ──────────────────────────────────
    nav2_yaml   = os.path.join(BRINGUP_DIR, 'config', 'nav2_params.yaml')
    ekf_yaml    = os.path.join(BRINGUP_DIR, 'config', 'ekf_params.yaml')
    slam_yaml   = os.path.join(BRINGUP_DIR, 'config', 'slam_params.yaml')
    twist_yaml  = os.path.join(get_package_share_directory('th_safety'),
                               'config', 'twist_mux.yaml')
    # キャリブ値 YAML (apply_calib で生成、存在しない場合は無視される)
    calib_yaml  = os.path.join(BRINGUP_DIR, 'config', 'calib.yaml')

    nodes = []

    # ── 1. robot_state_publisher / joint_state_publisher (URDF → TF) ─
    # base_link → laser_link 等の固定 TF を配信する。これが無いと SLAM /
    # Nav2 / leg_detection がスキャンを座標変換できない。
    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(DESC_DIR, 'urdf', 'th_robot.urdf.xacro')]),
        value_type=str)
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    ))
    nodes.append(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    ))

    # ── 2. RPLIDAR S1 (lidar_source:=local の場合のみ起動) ─
    nodes.append(Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        condition=IfCondition(lidar_is_local),
        parameters=[{
            'serial_port':     '/dev/lidar',
            'serial_baudrate': 256000,
            'frame_id':        'laser_link',
            'angle_compensate': True,
            'scan_mode':       'Standard',
        }],
        output='screen',
    ))
    nodes.append(LogInfo(
        condition=UnlessCondition(lidar_is_local),
        msg='lidar_source=network: ローカルsllidar_nodeは起動しません。'
            'ラズパイ側の/scanを受信するにはROS_DOMAIN_IDが一致している必要があります。',
    ))

    # ── 3. lidar_filter (死角マスク) ─────────────────────
    # lidar_source:=network の場合、これがラズパイ配信の /scan を実際に
    # 購読する唯一のノード。ESP32 SoftAP 越しだと DDS のマルチキャスト
    # 参加者発見(SPDP)がホスト間で成立しないことがある実機検証済みの事象
    # (2026-07-17: unicast UDP 疎通は正常なのに /scan が discover されない
    #  ことを確認)。ラズパイをユニキャストピアとして与えることで解消する
    # (network 時のみ。このプロセス単体にのみ適用し、コンテナ全体の環境変数
    # にすると mode_manager 等ローカルノード間の発見まで巻き添えで壊れる
    # ことを確認済みのためそれは避ける)。lidar_is_local と同じ条件分岐で
    # sllidar_node と対にしている。
    fastdds_profile_yaml = os.path.join(BRINGUP_DIR, 'config', 'fastdds_profile.xml')
    nodes.append(Node(
        package='th_perception',
        executable='lidar_filter.py',
        name='lidar_filter',
        parameters=[os.path.join(BRINGUP_DIR, 'config', 'perception_params.yaml')],
        additional_env={'FASTRTPS_DEFAULT_PROFILES_FILE': fastdds_profile_yaml},
        condition=UnlessCondition(lidar_is_local),
        output='screen',
    ))
    nodes.append(Node(
        package='th_perception',
        executable='lidar_filter.py',
        name='lidar_filter',
        parameters=[os.path.join(BRINGUP_DIR, 'config', 'perception_params.yaml')],
        condition=IfCondition(lidar_is_local),
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
        executable='esp32_bridge.py',
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
    # スタブ: person_tracker_stub.py
    nodes.append(Node(
        package='th_perception',
        executable='person_tracker_stub.py',
        name='person_tracker',
        condition=IfCondition(use_stub),
        parameters=[{'pattern': 'walk_forward', 'initial_x': 1.5}],
        output='screen',
    ))
    # 本番: human_kenchi (DR-SPAAM + PersonTracker, leg モード) + person_tracker_bridge.py
    # /scan_filtered (死角マスク済み) を入力にし、following_position を /person/status に変換する。
    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('leg_detection_bringup'),
            'launch', 'leg_detection.launch.py')),
        launch_arguments={
            'scan_topic':   '/scan_filtered',
            'target_frame': 'base_link',
            'scan_frame':   'laser_link',
            'odom_frame':   'odom',
            'use_rviz':     'false',
            'autostart':    'true',
        }.items(),
        condition=UnlessCondition(use_stub),
    ))
    nodes.append(Node(
        package='th_perception',
        executable='person_tracker_bridge.py',
        name='person_tracker_bridge',
        condition=UnlessCondition(use_stub),
        output='screen',
    ))

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

    # ── 11b. follow_planner_mapless (MAP不要の純粋軌跡追従モード。FOLLOWING_MAPLESS 時のみ動作)
    nodes.append(Node(
        package='th_planning',
        executable='follow_planner_mapless.py',
        name='follow_planner_mapless',
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
