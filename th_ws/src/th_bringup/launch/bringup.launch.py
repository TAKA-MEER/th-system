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

# WP-PARAM-02: registry.yaml → /root/th_data/generated/*.yaml のパラメータ生成
# ヘルパー。CMakeLists.txt が launch/ 以下をまるごと share にインストールするので
# このファイルと同じディレクトリに居るが、ROS2 launch は importlib で個別ロード
# するだけでこのディレクトリを sys.path に入れないため、自分でパスを通す。
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from params_generation import GENERATED_DIR, make_opaque_function  # noqa: E402

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
        DeclareLaunchArgument('stage', default_value='1',
                              description='params_generation.py が registry.yaml を'
                                          '解決するステージ番号 (WP-PARAM-02)'),
    ]

    use_stub     = LaunchConfiguration('use_stub')
    imu_enabled  = LaunchConfiguration('imu_enabled')
    map_yaml     = LaunchConfiguration('map_yaml')
    lidar_source = LaunchConfiguration('lidar_source')
    stage        = LaunchConfiguration('stage')
    lidar_is_local = PythonExpression(["'", lidar_source, "' == 'local'"])

    # ── 段階で重いスタックを出し分ける（N-27 の対処 (a)） ──────────
    # Nav2 のライフサイクル起動と DR-SPAAM のモデルロードが同時に走ると、
    # PC 側が一過性に数百 ms ストールする。実機で obstacle_limiter の 20Hz 出力が
    # limiter_dead_ms(250ms) を超えて途切れ、LIMITER_DEAD(CRITICAL) → ESTOP が
    # ラッチした（2026-08-31・DetailedDesign-open.md N-27。起動 18 秒後、
    # startup_grace_sec=3 の外なので猶予では防げない）。
    #
    # 段階 1（手押し・手動ジョグ）と段階 2（安全チェーン）はどちらも Nav2 も
    # 人物検知も使わない。使い始めるのは Nav2 が段階 3（WP-TRANSIT-01）、
    # 人物検知が段階 4 から。**要らないものを起動しない**ことで、安全側の
    # しきい値（limiter_dead_ms）を緩めずにストールそのものを無くす。
    #
    # connectivity_checker の required_nodes は [esp32_bridge, lidar_filter] だけ
    # なので、これらを止めても evt.link_ok の成立には影響しない（registry.yaml）。
    nav2_enabled       = PythonExpression(["int('", stage, "') >= 3"])
    perception_enabled = PythonExpression(["int('", stage, "') >= 4"])

    # ── 設定ファイルパス ──────────────────────────────────
    nav2_yaml   = os.path.join(BRINGUP_DIR, 'config', 'nav2_params.yaml')
    # imu_enabled:=true → エンコーダ+IMU、false(既定) → エンコーダのみ
    ekf_yaml_imu    = os.path.join(BRINGUP_DIR, 'config', 'ekf_params.yaml')
    ekf_yaml_no_imu = os.path.join(BRINGUP_DIR, 'config', 'ekf_params_no_imu.yaml')
    ekf_yaml    = PythonExpression(
        ["'", ekf_yaml_imu, "' if '", imu_enabled, "' == 'true' else '", ekf_yaml_no_imu, "'"])
    slam_yaml   = os.path.join(BRINGUP_DIR, 'config', 'slam_params.yaml')
    # キャリブ値 YAML (apply_calib で生成、存在しない場合は無視される)
    calib_yaml  = os.path.join(BRINGUP_DIR, 'config', 'calib.yaml')

    nodes = []

    # ── 0. パラメータ生成 (WP-PARAM-02) ──────────────────────
    # registry.yaml → /root/th_data/generated/*.yaml を、ノードを1つも起動する前に
    # 同期生成する（G-1）。アサーション違反なら例外で launch ごと止まる（G-2）。
    params_generation_action = OpaqueFunction(
        function=make_opaque_function(sim_default=False))
    nodes.append(params_generation_action)

    # 何を省いたかを起動ログに残す。省略は仕様であって故障ではない、と
    # その場で分かるようにする（N-27 の対処 (a) を入れた副作用で
    # 「Nav2 が上がらない」を不具合と誤認するのを防ぐ）。
    nodes.append(LogInfo(msg=PythonExpression([
        "'stage=", stage, ": Nav2/SLAM=' + ('起動' if int('", stage,
        "') >= 3 else '省略(段階3から)') + ' / 人物検知=' + "
        "('起動' if int('", stage, "') >= 4 else '省略(段階4から)')"])))

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
        # 静的ファイルを土台にし、registry.yaml 由来の生成ファイルを後段に重ねる
        # (G-4)。placeholder のキーはサニタイズで落ちるため、給値されていない値は
        # 土台の静的値がそのまま残る。
        parameters=[os.path.join(BRINGUP_DIR, 'config', 'perception_params.yaml'),
                    os.path.join(GENERATED_DIR, 'lidar_filter.yaml')],
        additional_env={'FASTRTPS_DEFAULT_PROFILES_FILE': fastdds_profile_yaml},
        condition=UnlessCondition(lidar_is_local),
        output='screen',
    ))
    nodes.append(Node(
        package='th_perception',
        executable='lidar_filter.py',
        name='lidar_filter',
        parameters=[os.path.join(BRINGUP_DIR, 'config', 'perception_params.yaml'),
                    os.path.join(GENERATED_DIR, 'lidar_filter.yaml')],
        condition=IfCondition(lidar_is_local),
        output='screen',
    ))

    # ── 4. esp32_bridge ───────────────────────────────────
    # calib.yaml が存在する場合は上書き。registry.yaml 由来の生成ファイルは
    # 最後段に重ねる (G-4)。
    esp32_params = [os.path.join(get_package_share_directory('th_esp32_bridge'),
                                 'config', 'params.yaml')]
    if os.path.exists(calib_yaml):
        esp32_params.append(calib_yaml)
    esp32_params.append(os.path.join(GENERATED_DIR, 'esp32_bridge.yaml'))

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
    # enabled_targets (O-7): registry.yaml の既定値は空リストであり、かつ
    # export.py は空リストを生成物からサニタイズして落とす（D3）ため、段階ごとの
    # 実際の値は registry 経由では渡らない。ここで launch から明示的に渡す
    # (DetailedDesign-names.md §7.3 の note「段階ごとに launch から渡す」の実体)。
    #
    # 実際に publisher が存在する対象だけを有効にする（O-7「publisher が
    # できるまで有効にしない」）。
    #
    # limiter（WP-TEST-01 の実装中に発見・追加。2026-08-27）: このコメントは
    # 元々「WP-SAFE-01 単体の時点では WP-SAFE-03/obstacle_limiter が未実装なので
    # limiter を入れられない」としていたが、**WP-SAFE-03 は既に実装済み**
    # （obstacle_limiter は上の「7b. obstacle_limiter」で無条件に起動しており、
    # `/safety/limiter_status` を実際に20Hzで発行している）。コメントの更新が
    # 漏れていたと判断し、limiter を追加する。DetailedDesign-safety.md §10 #11
    # の自動化（`obstacle_limiter` を SIGKILL → 重大フォルト検出）は
    # `targetEnabled("limiter")` がゲートしているため、これが無いと実機でも
    # obstacle_limiter のプロセス死亡を safety_monitor が一切検出できない
    # （DEBT-4 が実質的に塞がっていない状態だった）。
    #
    # mux（MUX_DEAD。`/cmd_vel_muxed` の remap 先も WP-SAFE-03 で完了済みなので
    # 同様に有効化できる可能性が高い）は**このパケットの範囲外**として意図的に
    # 触れていない——故障注入12「/cmd_vel の途絶」は別パケットの担当であり、
    # mux 検出との相互作用まで含めた検証はそちら側の判断に委ねる。
    SAFETY_ENABLED_TARGETS = ['lidar', 'esp32', 'runaway', 'state', 'firmware', 'limiter']
    nodes.append(Node(
        package='th_safety',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[os.path.join(
            get_package_share_directory('th_safety'),
            'config', 'safety_monitor.yaml'),
            os.path.join(GENERATED_DIR, 'safety_monitor.yaml'),
            {'enabled_targets': SAFETY_ENABLED_TARGETS}],
        output='screen',
    ))

    # ── 7. twist_mux ──────────────────────────────────────
    # 静的 th_safety/config/twist_mux.yaml は読まない。generated/twist_mux.yaml が
    # 階層構造(locks/topics)を完全に持つ唯一の情報源 (G-3, 二重管理の防止)。
    # WP-SAFE-03: 出力先を /cmd_vel_muxed に変更（後段に obstacle_limiter が入る。
    # /cmd_vel を publish してよいのは obstacle_limiter だけになった）。
    nodes.append(Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[os.path.join(GENERATED_DIR, 'twist_mux.yaml')],
        remappings=[('cmd_vel_out', '/cmd_vel_muxed')],
        output='screen',
    ))

    # ── 7b. obstacle_limiter ────────────────────────────────
    # WP-SAFE-03: /cmd_vel_muxed → /cmd_vel の最終段速度リミッタ。/cmd_vel の
    # publisher はこのノードだけ（CLAUDE.md「速度指令の流れ」参照）。
    # dev_mode は渡さない（names.md §1.3。safety_monitor と同じ構造的な保証）。
    # 起動時に base_link<-laser_link TF を有界リトライで取得できないと
    # 起動失敗する（obstacle_limiter.cpp。素通しで動かさない設計）。
    nodes.append(Node(
        package='th_safety',
        executable='obstacle_limiter',
        name='obstacle_limiter',
        parameters=[os.path.join(GENERATED_DIR, 'obstacle_limiter.yaml')],
        output='screen',
    ))

    # ── 7c. jog_gate ────────────────────────────────────────
    # WP-SAFE-04: /cmd_vel_manual_raw → /cmd_vel_manual の手動ジョグゲート。
    # /cmd_vel_manual の publisher はこのノードだけ（WebUI は /cmd_vel_manual_raw
    # へ publish。O-6）。attributes.yaml（th_state と同じファイル）を読み、
    # /system/state が新鮮かつ jog 許可のときだけ通す。通さないときは沈黙する
    # （ゼロを撃たない。J-1）。generated/jog_gate.yaml が state_stale_ms を運ぶ。
    nodes.append(Node(
        package='th_safety',
        executable='jog_gate',
        name='jog_gate',
        parameters=[os.path.join(GENERATED_DIR, 'jog_gate.yaml')],
        output='screen',
    ))

    # ── 8. mode_manager ───────────────────────────────────
    nodes.append(Node(
        package='th_mode_manager',
        executable='mode_manager',
        name='mode_manager',
        output='screen',
    ))

    # ── 8b. state_manager / connectivity_checker (WP-STATE-02/03) ──────
    # 新FSM (system/state)。旧FSM (mode_manager / robot/mode) と並走する。
    # トピック名は衝突しない（/safety/fault は両者が購読するのみで書き込みは
    # しない）。生成ファイルのみを使う（静的な土台ファイルは存在しない）。
    nodes.append(Node(
        package='th_state',
        executable='state_manager.py',
        name='state_manager',
        parameters=[os.path.join(GENERATED_DIR, 'state_manager.yaml')],
        output='screen',
    ))
    nodes.append(Node(
        package='th_state',
        executable='connectivity_checker.py',
        name='connectivity_checker',
        parameters=[os.path.join(GENERATED_DIR, 'connectivity_checker.yaml'),
                    {'sim': False}],
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
        # 段階 4 以上でのみ起動する（N-27 の対処 (a)。上の perception_enabled 参照）。
        condition=IfCondition(PythonExpression(
            ["'", use_stub, "' != 'true' and int('", stage, "') >= 4"])),
    ))
    nodes.append(Node(
        package='th_perception',
        executable='person_tracker_bridge.py',
        name='person_tracker_bridge',
        condition=IfCondition(PythonExpression(
            ["'", use_stub, "' != 'true' and int('", stage, "') >= 4"])),
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

    # ── 13a. route_recorder（教示経路の記録。WP-TRANSIT / demo-teach-replay）──
    nodes.append(Node(
        package='th_planning',
        executable='route_recorder.py',
        name='route_recorder',
        output='screen',
    ))

    # ── 13a'. replay_runner（教示再生の走行。WP-TRANSIT / demo-teach-replay）─
    nodes.append(Node(
        package='th_planning',
        executable='replay_runner.py',
        name='replay_runner',
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
    # WP-SAFE-03 / N-17: nav2_bringup 純正の navigation_launch.py は使わず、
    # th_bringup/launch/navigation_launch.py（ローカルフォーク。ファイル冒頭の
    # コメント参照）を使う。behavior_server が /cmd_vel に直接 publish して
    # 安全チェーンを迂回する問題をここで塞ぐ（gazebo.launch.py と同じ対処）。
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(BRINGUP_DIR, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time':    'false',
            'params_file':     nav2_yaml,
            'autostart':       'true',
        }.items(),
        # 段階 3（WP-TRANSIT-01）以上でのみ起動する（N-27 の対処 (a)）。
        condition=IfCondition(nav2_enabled),
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
        # 段階 3 以上 かつ map_yaml が空のときだけ（N-27 の対処 (a)）。
        condition=IfCondition(PythonExpression(
            ["'", map_yaml, "' == '' and int('", stage, "') >= 3"])),
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
        # 段階 3 以上 かつ map_yaml 指定ありのときだけ（N-27 の対処 (a)）。
        condition=IfCondition(PythonExpression(
            ["'", map_yaml, "' != '' and int('", stage, "') >= 3"])),
    )
    nodes.append(localization_launch)

    return LaunchDescription(args + nodes)
