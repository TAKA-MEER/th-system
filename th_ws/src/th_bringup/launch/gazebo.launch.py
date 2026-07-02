#!/usr/bin/env python3
"""
gazebo.launch.py — Gazebo シミュレーション起動ファイル
==========================================================
実機モード (sim:=false) とシミュレーションモード (sim:=true) を
単一の launch ファイルで切り替える。
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    TimerAction, LogInfo, OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, LaunchConfiguration, PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

BRINGUP_DIR  = get_package_share_directory('th_bringup')
DESC_DIR     = get_package_share_directory('th_description')
NAV2_DIR     = get_package_share_directory('nav2_bringup')
SLAM_DIR     = get_package_share_directory('slam_toolbox')
SAFETY_DIR   = get_package_share_directory('th_safety')

def _set_sim_time(context, *args, **kwargs):
    is_sim = LaunchConfiguration('sim').perform(context).lower() in ('true', '1', 'yes')
    return [SetParameter(name='use_sim_time', value=is_sim)]  # Python bool を渡す

def generate_launch_description():

    # ── 引数定義 ─────────────────────────────────────────────
    declared_args = [
        DeclareLaunchArgument('sim',        default_value='true',
            description='true=Gazebo シミュレーション, false=実機'),
        DeclareLaunchArgument('slam',       default_value='true',
            description='true=SLAM マッピング, false=既存地図でナビ'),
        DeclareLaunchArgument('map_yaml',   default_value='',
            description='既存地図 YAML パス (slam:=false 時に使用)'),
        DeclareLaunchArgument('use_stub',   default_value='false',
            description='true=試験員トラッカースタブを使用'),
        DeclareLaunchArgument('obstacle',   default_value='true',
            description='true=ランダム移動する障害物(wanderer)を動かす(回避検証用)'),
        DeclareLaunchArgument('imu_enabled',default_value='false',
            description='true=IMU を EKF に追加'),
        DeclareLaunchArgument('rviz',       default_value='true',
            description='true=RViz2 を起動'),
        DeclareLaunchArgument('log_level',  default_value='info'),
        DeclareLaunchArgument('world',
            default_value=os.path.join(BRINGUP_DIR, 'worlds', 'panel_room_no_actor.world'),
            description='Gazebo ワールドファイル'),
        DeclareLaunchArgument('robot_x',    default_value='-4.0'),
        DeclareLaunchArgument('robot_y',    default_value='-3.0'),
        DeclareLaunchArgument('robot_yaw',  default_value='0.0'),
    ]

    sim        = LaunchConfiguration('sim')
    slam       = LaunchConfiguration('slam')
    map_yaml   = LaunchConfiguration('map_yaml')
    use_stub   = LaunchConfiguration('use_stub')
    rviz       = LaunchConfiguration('rviz')
    world      = LaunchConfiguration('world')
    robot_x    = LaunchConfiguration('robot_x')
    robot_y    = LaunchConfiguration('robot_y')
    robot_yaw  = LaunchConfiguration('robot_yaw')

    # ── URDF / robot_description ─────────────────────────────
    urdf_file = os.path.join(DESC_DIR, 'urdf', 'th_robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    # ── 設定ファイルパス ─────────────────────────────────────
    nav2_params_sim  = os.path.join(BRINGUP_DIR, 'config', 'nav2_params_sim.yaml')
    nav2_params_real = os.path.join(BRINGUP_DIR, 'config', 'nav2_params.yaml')
    slam_params_sim  = os.path.join(BRINGUP_DIR, 'config', 'slam_params_sim.yaml')
    slam_params_real = os.path.join(BRINGUP_DIR, 'config', 'slam_params.yaml')
    safety_sim       = os.path.join(BRINGUP_DIR, 'config', 'safety_monitor_sim.yaml')
    safety_real      = os.path.join(SAFETY_DIR,  'config', 'safety_monitor.yaml')
    twist_yaml       = os.path.join(SAFETY_DIR,  'config', 'twist_mux.yaml')
    ekf_yaml         = os.path.join(BRINGUP_DIR, 'config', 'ekf_params.yaml')
    calib_yaml       = os.path.join(BRINGUP_DIR, 'config', 'calib.yaml')
    planning_yaml    = os.path.join(BRINGUP_DIR, 'config', 'planning_params.yaml')
    panels_yaml      = os.path.join(BRINGUP_DIR, 'config', 'panels.yaml')
    perc_yaml        = os.path.join(BRINGUP_DIR, 'config', 'perception_params.yaml')
    rviz_cfg         = os.path.join(BRINGUP_DIR, 'config', 'rviz', 'th_sim.rviz')

    # ── use_sim_time を全ノードに伝播 ────────────────────────
    sim_time_action = OpaqueFunction(function=_set_sim_time)

    # ════════════════════════════════════════════════════════
    # 共通ノード（実機・シミュレーション共通）
    # ════════════════════════════════════════════════════════
    common_nodes = [
        # robot_state_publisher（URDF → TF）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        # twist_mux
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            parameters=[twist_yaml],
            remappings=[('cmd_vel_out', '/cmd_vel')],
            output='screen',
        ),
        # mode_manager
        Node(
            package='th_mode_manager',
            executable='mode_manager',
            name='mode_manager',
            output='screen',
        ),
        # lidar_filter（/scan → /scan_filtered）
        Node(
            package='th_perception',
            executable='lidar_filter.py',
            name='lidar_filter',
            parameters=[perc_yaml],
            output='screen',
        ),
        # person_predictor
        Node(
            package='th_perception',
            executable='person_predictor.py',
            name='person_predictor',
            output='screen',
        ),
        # follow_planner
        Node(
            package='th_planning',
            executable='follow_planner.py',
            name='follow_planner',
            parameters=[planning_yaml],
            output='screen',
        ),
        # panel_navigator
        Node(
            package='th_planning',
            executable='panel_navigator.py',
            name='panel_navigator',
            parameters=[{'panels_yaml': panels_yaml}],
            output='screen',
        ),
        # manual_command_handler
        Node(
            package='th_planning',
            executable='manual_command_handler.py',
            name='manual_command_handler',
            output='screen',
        ),
        # rosbridge
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{'port': 9090}],
            output='screen',
        ),
    ]

    # ════════════════════════════════════════════════════════
    # シミュレーション専用ノード（sim:=true）
    # ════════════════════════════════════════════════════════

    # Gazebo 本体
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world':   world,
            'verbose': 'false',
            'pause':   'false',
        }.items(),
        condition=IfCondition(sim),
    )

    # ロボットを Gazebo にスポーン
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'th_robot',
            '-x',    robot_x,
            '-y',    robot_y,
            '-Y',    robot_yaw,
        ],
        output='screen',
        condition=IfCondition(sim),
    )

    # safety_monitor: シミュレーション設定
    safety_sim_node = Node(
        package='th_safety',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[safety_sim],
        output='screen',
        condition=IfCondition(sim),
    )

    # safety_monitor: 実機設定
    safety_real_node = Node(
        package='th_safety',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[safety_real],
        output='screen',
        condition=UnlessCondition(sim),
    )

    # Gazebo Actor → /person/status 中継（シミュレーション用）
    person_relay = Node(
        package='th_perception',
        executable='gazebo_person_relay.py',
        name='gazebo_person_relay',
        parameters=[{
            'actor_name':       'inspector',
            'robot_name':       'th_robot',
            'base_frame':       'base_link',
            'max_detect_range': 12.0,  # 部屋対角最大~12.8m をカバー (デフォルト8mだと西端ロボット→東端試験員が圏外)
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('sim')),
    )

    # 試験員シリンダーを巡回移動させる（panel_room_no_actor.world 用）
    person_mover = Node(
        package='th_perception',
        executable='person_mover.py',
        name='person_mover',
        parameters=[{'pattern': 'patrol', 'model_name': 'inspector'}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('sim')),
    )

    # スタブ（use_stub=true の場合）
    person_stub = Node(
        package='th_perception',
        executable='person_tracker_stub.py',
        name='person_tracker',
        parameters=[{'pattern': 'walk_forward', 'initial_x': 1.5}],
        output='screen',
        condition=IfCondition(use_stub),
    )

    # ランダム移動障害物 "wanderer" を動かす（回避動作検証用、シミュレーションのみ）
    # collision を持つ円柱なので LiDAR に映り Nav2 が回避する。
    obstacle_mover = Node(
        package='th_perception',
        executable='obstacle_mover.py',
        name='obstacle_mover',
        parameters=[{
            'model_name': 'wanderer',
            'move_speed': 0.5,
            'x_min': -4.0, 'x_max': 4.0,
            'y_min': -3.0, 'y_max': 3.0,
        }],
        output='screen',
        condition=IfCondition(PythonExpression(
            ["'", sim, "' == 'true' and '",
             LaunchConfiguration('obstacle'), "' == 'true'"])),
    )

    # ════════════════════════════════════════════════════════
    # 実機専用ノード（sim:=false）
    # ════════════════════════════════════════════════════════

    lidar_node = Node(
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
        condition=UnlessCondition(sim),
    )

    esp32_bridge = Node(
        package='th_esp32_bridge',
        executable='esp32_bridge.py',
        name='esp32_bridge',
        parameters=[
            os.path.join(get_package_share_directory('th_esp32_bridge'),
                         'config', 'params.yaml'),
        ] + ([calib_yaml] if os.path.exists(calib_yaml) else []),
        output='screen',
        condition=UnlessCondition(sim),
    )

    # EKF（実機のみ: Gazebo は diff_drive plugin が /odom を発行する）
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_yaml],
        output='screen',
        condition=UnlessCondition(sim),
    )

    # ════════════════════════════════════════════════════════
    # Nav2 + SLAM（実機・シミュレーション共通、設定ファイルのみ異なる）
    # ════════════════════════════════════════════════════════

    # Nav2 (ナビゲーション部分のみ) — シミュレーション
    nav2_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(NAV2_DIR, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file':  nav2_params_sim,
            'autostart':    'true',
        }.items(),
        condition=IfCondition(sim),
    )

    # Nav2 (ナビゲーション部分のみ) — 実機
    nav2_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(NAV2_DIR, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'False',
            'params_file':  nav2_params_real,
            'autostart':    'true',
        }.items(),
        condition=UnlessCondition(sim),
    )

    # Localization (AMCL + map_server) — シミュレーション
    loc_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(NAV2_DIR, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map':          map_yaml,
            'use_sim_time': 'True',
            'params_file':  nav2_params_sim,
            'autostart':    'true',
        }.items(),
        condition=IfCondition(PythonExpression(["'", sim, "' == 'true' and '", slam, "' == 'false'"])),
    )

    # Localization (AMCL + map_server) — 実機
    loc_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(NAV2_DIR, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map':          map_yaml,
            'use_sim_time': 'False',
            'params_file':  nav2_params_real,
            'autostart':    'true',
        }.items(),
        condition=IfCondition(PythonExpression(["'", sim, "' == 'false' and '", slam, "' == 'false'"])),
    )

    # SLAM — シミュレーション
    slam_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(SLAM_DIR, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': slam_params_sim,
            'use_sim_time':     'True',
        }.items(),
        condition=IfCondition(PythonExpression(["'", sim, "' == 'true' and '", slam, "' == 'true'"])),
    )

    # SLAM — 実機
    slam_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(SLAM_DIR, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': slam_params_real,
            'use_sim_time':     'False',
        }.items(),
        condition=IfCondition(PythonExpression(["'", sim, "' == 'false' and '", slam, "' == 'true'"])),
    )

    # ════════════════════════════════════════════════════════
    # RViz2
    # ════════════════════════════════════════════════════════
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
        condition=IfCondition(rviz),
    )

    # ════════════════════════════════════════════════════════
    # 起動順序の制御
    # Gazebo が完全に起動してからロボットをスポーンする
    # ════════════════════════════════════════════════════════
    spawn_delay = TimerAction(
        period=4.5,   # Gazebo 完全起動 + LiDAR/person_relay 安定化に余裕を持たせる
        actions=[spawn_robot],
    )

    return LaunchDescription(
        declared_args + [
            sim_time_action,
            LogInfo(msg=['[th_bringup] sim=', sim, ' slam=', slam]),

            # Gazebo（シミュレーション時のみ）
            gazebo_node,
            spawn_delay,

            # URDF / robot_state_publisher
            *common_nodes,

            # safety_monitor（設定が異なる）
            safety_sim_node,
            safety_real_node,

            # 試験員ソース（Gazebo 中継 + シリンダー移動 or スタブ）
            person_relay,
            person_mover,
            person_stub,

            # ランダム移動障害物（回避検証用）
            obstacle_mover,

            # 実機ノード
            lidar_node,
            esp32_bridge,
            ekf_node,

            # Nav2 + SLAM / Localization
            nav2_sim,
            nav2_real,
            loc_sim,
            loc_real,
            slam_sim,
            slam_real,

            # RViz2
            rviz_node,
        ]
    )