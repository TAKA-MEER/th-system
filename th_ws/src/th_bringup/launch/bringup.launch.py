#!/usr/bin/env python3
# ============================================================
# bringup.launch.py — TH システム フル起動
#
# 起動オプション:
#   use_stub:=true    試験員トラッカーをスタブに切替 (デフォルト: false)
#   imu_enabled:=true IMU 入力を EKF に追加 (デフォルト: false。要ファーム更新)
#   map_yaml:=<path>  使用する地図ファイル (デフォルト: 空=SLAM マッピングモード)
#   lidar_source:=local    USB直結のsllidar_nodeを起動 (デフォルト)
#   lidar_source:=network  ラズパイ等が配信する/scanを使用 (ローカル起動なし。
#                          Pi側とROS_DOMAIN_IDを一致させること)
# ============================================================
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                             IncludeLaunchDescription, LogInfo, OpaqueFunction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration,
                                   PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue

# params_generation.py はこの launch ファイルと同じディレクトリに置く（WP-PARAM-02）。
# th_bringup は ament_cmake_python パッケージではない（launch/ は colcon が丸ごと
# share へコピーするだけ）ため、通常の import では見つからない。自分の隣に sys.path を
# 通して読む（gazebo.launch.py も同じ手段を使う）。
sys.path.insert(0, os.path.dirname(__file__))
from params_generation import GENERATED_DIR, REGISTRY_NODES, make_opaque_function  # noqa: E402

BRINGUP_DIR  = get_package_share_directory('th_bringup')
DESC_DIR     = get_package_share_directory('th_description')
NAV2_DIR     = get_package_share_directory('nav2_bringup')


def generate_launch_description():
    # ── 引数 ──────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('use_stub',    default_value='false',
                              description='試験員トラッカーをスタブで代替'),
        # 既定 false: ジャイロ単位の修正(2026-08-06、esp32/src/imu.cpp)を含む
        # ファームウェアを書き込み、実機でヨーレートを検証するまでは有効にしない。
        # 未修正のファームは angular_velocity を dps で送ってくるため、rad/s
        # 規定として読む EKF が 57.3 倍のヨーレートを信じてオドメトリが壊れる。
        # 検証手順は docs/architecture.md「IMU (DSR1603 / BNO055) 追加」参照。
        DeclareLaunchArgument('imu_enabled', default_value='false',
                              description='IMU (DSR1603/BNO055) の vyaw を EKF に追加。'
                                          'ジャイロ単位修正済みファームの書き込みと'
                                          '実機検証が済んでから true にすること'),
        DeclareLaunchArgument('map_yaml',    default_value='',
                              description='地図 YAML パス (空=SLAM モード)'),
        DeclareLaunchArgument('log_level',   default_value='info'),
        DeclareLaunchArgument('lidar_source', default_value='local',
                              description='local=USB直結sllidar_node起動 / '
                                          'network=ラズパイ等が配信する/scanを使用'),
        # WP-PARAM-02 §3.3: 既定は最大値（=どの段階でも全ブロッキング placeholder を
        # 検査する。DD-6「段階を指定し忘れると起動できない」は意図した設計。§11）。
        DeclareLaunchArgument('stage', default_value='8',
                              description='現在の開発段階（1〜8）。A8 の判定に使う。'
                                          '既定は最大値＝指定を必須にする（DD-6）'),
    ]

    use_stub     = LaunchConfiguration('use_stub')
    imu_enabled  = LaunchConfiguration('imu_enabled')
    map_yaml     = LaunchConfiguration('map_yaml')
    lidar_source = LaunchConfiguration('lidar_source')
    lidar_is_local = PythonExpression(["'", lidar_source, "' == 'local'"])
    map_is_empty = PythonExpression(["'", map_yaml, "' == ''"])

    # ── 設定ファイルパス ──────────────────────────────────
    nav2_yaml   = os.path.join(BRINGUP_DIR, 'config', 'nav2_params.yaml')
    # imu_enabled:=true → エンコーダ+IMU、false(既定) → エンコーダのみ
    ekf_yaml_imu    = os.path.join(BRINGUP_DIR, 'config', 'ekf_params.yaml')
    ekf_yaml_no_imu = os.path.join(BRINGUP_DIR, 'config', 'ekf_params_no_imu.yaml')
    ekf_yaml    = PythonExpression(
        ["'", ekf_yaml_imu, "' if '", imu_enabled, "' == 'true' else '", ekf_yaml_no_imu, "'"])
    slam_yaml   = os.path.join(BRINGUP_DIR, 'config', 'slam_params.yaml')
    # twist_mux は generated/twist_mux.yaml だけを読む（G-3。下記 7.）ので、
    # 静的な th_safety/config/twist_mux.yaml へのパスはもう組み立てない。
    # キャリブ値 YAML (apply_calib で生成、存在しない場合は無視される)
    calib_yaml  = os.path.join(BRINGUP_DIR, 'config', 'calib.yaml')

    nodes = []

    # ── 0. パラメータ生成（WP-PARAM-02。G-1: 他の全ノードより前に同期実行する） ──
    # registry.yaml から ROS2 パラメータ YAML と twist_mux.yaml を
    # /root/th_data/generated/ へ書く。アサーション違反なら例外を投げて launch を
    # 止める（G-2）。bringup.launch.py は実機専用launchなので sim は常に False。
    params_generation_action = OpaqueFunction(function=make_opaque_function(
        nodes=REGISTRY_NODES, stage_arg_name='stage', sim_arg_name=None, sim_default=False))
    nodes.append(params_generation_action)

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
    # blind_angle_ranges は registry.yaml では class:c/placeholder（DEBT-2。未実測）。
    # perception_params.yaml を土台にし、generated/lidar_filter.yaml を後段に重ねて
    # blind_angle_ranges だけ registry 側で上書きする（G-4）。person_predictor のセクションは
    # perception_params.yaml 側にしか無いが、lidar_filter ノードは自分の名前空間
    # (lidar_filter:) しか読まないので無関係。
    lidar_filter_params = [os.path.join(BRINGUP_DIR, 'config', 'perception_params.yaml'),
                           os.path.join(GENERATED_DIR, 'lidar_filter.yaml')]
    nodes.append(Node(
        package='th_perception',
        executable='lidar_filter.py',
        name='lidar_filter',
        parameters=lidar_filter_params,
        additional_env={'FASTRTPS_DEFAULT_PROFILES_FILE': fastdds_profile_yaml},
        condition=UnlessCondition(lidar_is_local),
        output='screen',
    ))
    nodes.append(Node(
        package='th_perception',
        executable='lidar_filter.py',
        name='lidar_filter',
        parameters=lidar_filter_params,
        condition=IfCondition(lidar_is_local),
        output='screen',
    ))

    # ── 4. esp32_bridge ───────────────────────────────────
    # 静的な params.yaml (publish_tf 等の配線情報。DetailedDesign-reuse.md §2.6
    # 「publish_tf: false は絶対に変えない」) を土台にし、registry 由来の
    # wheel_radius_scale / esp32_watchdog_ms / cmd_vel_stale_ms / esp32_ws_port を
    # generated/esp32_bridge.yaml で重ねる（G-4）。calib.yaml はさらにその後段
    # （校正済みの実測値が最優先）。
    esp32_params = [os.path.join(get_package_share_directory('th_esp32_bridge'),
                                 'config', 'params.yaml'),
                    os.path.join(GENERATED_DIR, 'esp32_bridge.yaml')]
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
    nodes.append(LogInfo(
        condition=IfCondition(imu_enabled),
        msg='imu_enabled=true: ekf_params.yaml (エンコーダ+IMUのvyaw) を使用します。'
            'ジャイロ単位修正(2026-08-06)を含むファームウェアが書き込まれていることを'
            '確認してください。未修正だと角速度が dps で届き、EKF が 57.3 倍の'
            'ヨーレートを信じてオドメトリが壊れます。'
            'キャリブレーションは ros2 run th_calibration imu_calib_check.py で確認。',
    ))
    nodes.append(LogInfo(
        condition=UnlessCondition(imu_enabled),
        msg='imu_enabled=false (既定): ekf_params_no_imu.yaml (エンコーダのみ) を使用します。'
            'クローラの超信地旋回スリップによる yaw 誤差は補正されません。',
    ))
    nodes.append(Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_yaml],
        output='screen',
    ))

    # ── 6. safety_monitor ─────────────────────────────────
    # 静的な safety_monitor.yaml (check_period_ms 等、まだ registry 化していない値)
    # を土台にし、registry 由来のタイムアウト類を generated/safety_monitor.yaml で
    # 重ねる（G-4）。
    nodes.append(Node(
        package='th_safety',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[os.path.join(
            get_package_share_directory('th_safety'), 'config', 'safety_monitor.yaml'),
            os.path.join(GENERATED_DIR, 'safety_monitor.yaml')],
        output='screen',
    ))

    # ── 7. twist_mux ──────────────────────────────────────
    # generated/twist_mux.yaml は params_generation.py の OpaqueFunction が
    # ロック/トピックの構造ごと組み直した完全な設定（G-3）。静的な th_safety/config/
    # twist_mux.yaml はもう読まない（数値の実体は registry.yaml のみ。二重管理を避ける）。
    nodes.append(Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[os.path.join(GENERATED_DIR, 'twist_mux.yaml')],
        remappings=[('cmd_vel_out', '/cmd_vel')],
        output='screen',
    ))

    # ── 7b. params_audit（WP-PARAM-02。生成結果を読んで /system/params_status を
    # publish し、/params/get・/params/set・/params/save を提供する） ───
    nodes.append(Node(
        package='th_params',
        executable='params_audit.py',
        name='params_audit',
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

    # ── 12b. summon_navigator ──────────────────────────────
    nodes.append(Node(
        package='th_planning',
        executable='summon_navigator.py',
        name='summon_navigator',
        parameters=[os.path.join(BRINGUP_DIR, 'config', 'planning_params.yaml')],
        output='screen',
    ))

    # ── 13. manual_command_handler ────────────────────────
    nodes.append(Node(
        package='th_planning',
        executable='manual_command_handler.py',
        name='manual_command_handler',
        output='screen',
    ))

    # ── 13b. config_manager (WebUI 設定パネル: パラメータ調整の仲介) ──
    nodes.append(Node(
        package='th_config_manager',
        executable='config_manager.py',
        name='config_manager',
        output='screen',
    ))

    # ── 13c. slam_control (WebUI: 地図作成 開始/停止の仲介) ────
    nodes.append(Node(
        package='th_config_manager',
        executable='slam_control.py',
        name='slam_control',
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

    # ── 15. Nav2 (ナビゲーション部分のみ。map_server/AMCL は含まない) ──
    # localization (SLAM または AMCL) は下記 16/17 で map_yaml の有無により分岐する。
    # 両方を無条件起動すると map→odom TF を取り合って SLAM 走行が機能しなくなるため
    # (2026-07-23 修正: 従来は nav2_bringup/bringup_launch.py = フル AMCL+map_server
    #  スタックと SLAM Toolbox を同時に起動しており、これが原因だった)。
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(NAV2_DIR, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time':    'false',
            'params_file':     nav2_yaml,
            'autostart':       'true',
        }.items(),
    )
    nodes.append(nav2_launch)

    # ── 16. SLAM Toolbox (map_yaml が空 = デフォルト。実機は毎回このモード) ──
    # 地図作成自体の開始/停止は WebUI 経由の slam_control ノードが
    # pause_new_measurements をトグルして制御する（VISION.md §7 参照）。
    # online_async_launch.py (= async_slam_toolbox_node) は使わない。
    # map_and_localization_slam_toolbox_node は /slam_toolbox/set_localization_mode
    # (std_srvs/SetBool) を持ち、mapping ⇄ localization を同一プロセス内で
    # ランタイム切替できる。これが「地図作成停止 = 地図は凍結・自己位置推定は継続」
    # の要件を満たす唯一の手段 (VISION.md §8)。
    #
    # 旧実装は async ノード + pause_new_measurements だったが、これはスキャン処理
    # そのものを止めるため停止後は map→odom が凍結しデッドレコニングになる
    # (2026-08-07 実機で確認)。
    #
    # ノード名は slam_toolbox のまま維持する。サービス名 (/slam_toolbox/*) と
    # slam_params.yaml のパラメータキーが変わらないようにするため。
    # なお本ノードは slam_toolbox の experimental/ 配下の実装である。
    #
    # respawn: 2026-08-07 実機で SIGSEGV (exit code -11) で落ちるのを確認した。
    # 落ちたままだと map→odom が消えて自己位置が失われ、それに気づかず走り
    # 続けることになる。dr_spaam_ros と同じ理由で自動再起動させる。
    # 再起動後は mapping モードに戻るため、slam_control がサービスの再出現を
    # 検知して停止状態を再適用する。
    nodes.append(Node(
        package='slam_toolbox',
        executable='map_and_localization_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_yaml, {'use_sim_time': False}],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(map_is_empty),
    ))

    # ── 17. AMCL + map_server (map_yaml 指定時のみ。UI には出さない休眠経路) ──
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(NAV2_DIR, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map':          map_yaml,
            'use_sim_time': 'false',
            'params_file':  nav2_yaml,
            'autostart':    'true',
        }.items(),
        condition=UnlessCondition(map_is_empty),
    )
    nodes.append(localization_launch)

    return LaunchDescription(args + nodes)
