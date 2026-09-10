#!/usr/bin/env python3
# ============================================================
# bringup.launch.py — TH システム フル起動
#
# 起動オプション:
#   use_stub:=true    試験員トラッカーをスタブに切替 (デフォルト: false)
#   imu_enabled:=false  IMU の vyaw 融合を切る (デフォルト: true。WS-9V)
#   map_yaml:=<path>  使用する地図ファイル (デフォルト: 空=SLAM マッピングモード)
#   lidar_source:=local    USB直結のsllidar_nodeを起動 (デフォルト)
#   lidar_source:=network  ラズパイ等が配信する/scanを使用 (ローカル起動なし。
#                          Pi側とROS_DOMAIN_IDを一致させること)
# ============================================================
import os
import sys
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                             IncludeLaunchDescription, LogInfo, OpaqueFunction,
                             TimerAction)
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
from prelaunch_guard import make_guard_opaque_function  # noqa: E402

BRINGUP_DIR  = get_package_share_directory('th_bringup')
DESC_DIR     = get_package_share_directory('th_description')
NAV2_DIR     = get_package_share_directory('nav2_bringup')


def generate_launch_description():
    # ── 引数 ──────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('use_stub',    default_value='false',
                              description='試験員トラッカーをスタブで代替'),
        # WS-9V (2026-09-04): 既定 true。クローラは超信地旋回でトラックが滑り、
        # エンコーダだけのオドメトリは L 字コーナーで yaw が飛ぶ。特徴の無い長い
        # 廊下ではその誤差がそのまま伸び、slam_toolbox の localization（探索窓
        # ±0.25m）では窓の外に出て補正できない（実機 2026-09-04 の長距離再生で
        # 地図と scan が明確にずれた）。EKF が融合するのはジャイロ vyaw のみ・
        # BNO055 の gyro は fully calibrated・dps→rad/s 修正(2026-08-06)済み・
        # ekf_params.yaml の imu0 欠落(2026-09-02)も修正済みで、有効化の条件は整った。
        # ジャイロ単位未修正のファームの個体で動かすときだけ imu_enabled:=false。
        DeclareLaunchArgument('imu_enabled', default_value='true',
                              description='IMU (DSR1603/BNO055) の vyaw を EKF に融合（既定 true）。'
                                          'ジャイロ単位未修正のファームの個体では false にする'),
        DeclareLaunchArgument('map_yaml',    default_value='',
                              description='地図 YAML パス (空=SLAM モード)'),
        DeclareLaunchArgument('log_level',   default_value='info'),
        DeclareLaunchArgument('lidar_source', default_value='local',
                              description='local=USB直結sllidar_node起動 / '
                                          'network=ラズパイ等が配信する/scanを使用'),
        DeclareLaunchArgument('stage', default_value='1',
                              description='params_generation.py が registry.yaml を'
                                          '解決するステージ番号 (WP-PARAM-02)'),
        # WS-8B（教示・再生の地図フレーム追従）: stage<3 でも slam_toolbox を
        # mapping モードで起動し、教示中も再生中も map→odom を連続補正する。
        # Nav2 planner/controller は起動しない（stage ゲート据え置き＝N-27 回避）。
        DeclareLaunchArgument('enable_route_slam', default_value='false',
                              description='教示・再生用に slam_toolbox を mapping '
                                          'モードで起動する (WS-8B。stage<3 でも可)'),
        # WP-ONSITE-P0: stage>=4 で DR-SPAAM を Nav2 起動と重ねないための遅延（N-27）。
        DeclareLaunchArgument('perception_start_delay', default_value='8.0',
                              description='DR-SPAAM/person_tracker_bridge の起動を'
                                          'この秒数だけ遅らせる (N-27: Nav2 lifecycle と'
                                          'モデルロードの同時実行によるCPUストール回避)'),
    ]

    use_stub     = LaunchConfiguration('use_stub')
    imu_enabled  = LaunchConfiguration('imu_enabled')
    map_yaml     = LaunchConfiguration('map_yaml')
    lidar_source = LaunchConfiguration('lidar_source')
    stage        = LaunchConfiguration('stage')
    enable_route_slam = LaunchConfiguration('enable_route_slam')
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
    # WP-ONSITE-01: 試験場内ピン登録 (pin_registrar)。Nav2/人物トラッカーと同じ
    # 段階 3 で上がる（ピンの map 座標は SLAM の map→base_link TF が要るため）。
    onsite_enabled      = PythonExpression(["int('", stage, "') >= 3"])
    # WS-9E: 人物データを使うノードは、/person/status の publisher が起動する
    # ときだけ立てる。旧 FOLLOW 系ノード（follow_planner* / person_predictor）は
    # 出力先の /cmd_vel_retreat を twist_mux が購読しておらず（WP-SAFE-03 以降）、
    # 新設計で廃止対象（DetailedDesign-reuse.md）。
    # 2026-09-10（WS-9AB 系）: 従来 stage>=4 でも立てていたが、stage>=4 は
    # 試験場内デモ（FOLLOW を使わない）専用で、旧 3 ノードが各 10〜12% の CPU を
    # 食うだけだった。stub 経由の FOLLOW 検証（use_stub:=true）でのみ立てる。
    person_logic_enabled = PythonExpression(["'", use_stub, "' == 'true'"])

    # ── 設定ファイルパス ──────────────────────────────────
    nav2_yaml   = os.path.join(BRINGUP_DIR, 'config', 'nav2_params.yaml')
    # imu_enabled:=true(既定) → エンコーダ+IMU の vyaw、false → エンコーダのみ
    ekf_yaml_imu    = os.path.join(BRINGUP_DIR, 'config', 'ekf_params.yaml')
    ekf_yaml_no_imu = os.path.join(BRINGUP_DIR, 'config', 'ekf_params_no_imu.yaml')
    ekf_yaml    = PythonExpression(
        ["'", ekf_yaml_imu, "' if '", imu_enabled, "' == 'true' else '", ekf_yaml_no_imu, "'"])
    slam_yaml   = os.path.join(BRINGUP_DIR, 'config', 'slam_params.yaml')
    # キャリブ値 YAML (apply_calib で生成、存在しない場合は無視される)
    calib_yaml  = os.path.join(BRINGUP_DIR, 'config', 'calib.yaml')

    nodes = []

    # ── -1. 前回起動の後始末 (2026-09-05) ──────────────────────
    # 前回の bringup/gazebo launch を止め忘れたまま次を起動すると、esp32_bridge の
    # ポート衝突や slam_toolbox の資源の奪い合い(セグフォルト無限再起動)を起こす
    # （実機で発生・prelaunch_guard.py のモジュールdocstring参照）。ノードを1つも
    # 起動する前に、自分以外の生き残りを自動で止める。
    nodes.append(OpaqueFunction(function=make_guard_opaque_function()))

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
        "'stage=", stage, ": Nav2=' + ('起動' if int('", stage,
        "') >= 3 else '省略(段階3から)') + ' / SLAM=' + "
        "('起動' if (int('", stage, "') >= 3 or '", enable_route_slam,
        "'.lower() in ('true','1')) else '省略') + ' / 人物検知=' + "
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
    # 購読する唯一のノード。
    #
    # 2026-07-17: 旧 ESP32 SoftAP（192.168.4.x）越しだと DDS の参加者発見(SPDP)
    # がホスト間で成立しないことがあり、`fastdds_profile.xml` でラズパイ
    # (192.168.4.2) をユニキャスト初期ピアに与えて対処していた。
    #
    # 2026-09-01: ネットワークが 192.168.5.x に変わり、その固定 IP（存在しない
    # サブネット）が逆に discovery を壊して /scan_filtered が完全無音になった。
    # 現行 AP はマルチキャスト discovery が正常なので additional_env を外した。
    # **別 AP でマルチキャストが不安定なら**、`config/fastdds_profile.xml` の
    # <address> を現ラズパイの IP に直し、この Node に
    # `additional_env={'FASTRTPS_DEFAULT_PROFILES_FILE': '<...>/fastdds_profile.xml'}`
    # を戻すこと（network 時のみ・このプロセス単体に絞る。コンテナ全体に
    # 適用すると同一ホスト内ノード間の発見まで壊れる — 2026-07-17 検証済み）。
    nodes.append(Node(
        package='th_perception',
        executable='lidar_filter.py',
        name='lidar_filter',
        # 静的ファイルを土台にし、registry.yaml 由来の生成ファイルを後段に
        # 重ねる (G-4)。placeholder のキーはサニタイズで落ちる。
        parameters=[os.path.join(BRINGUP_DIR, 'config', 'perception_params.yaml'),
                    os.path.join(GENERATED_DIR, 'lidar_filter.yaml')],
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
        msg='imu_enabled=true (既定): ekf_params.yaml (エンコーダ+IMUのvyaw) を使用します。'
            'ジャイロ単位修正(2026-08-06)を含むファームウェアが書き込まれていることを'
            '確認してください。未修正だと角速度が dps で届き、EKF が 57.3 倍の'
            'ヨーレートを信じてオドメトリが壊れます（その場合は imu_enabled:=false）。'
            'キャリブレーションは ros2 run th_calibration imu_calib_check.py で確認。',
    ))
    nodes.append(LogInfo(
        condition=UnlessCondition(imu_enabled),
        msg='imu_enabled=false: ekf_params_no_imu.yaml (エンコーダのみ) を使用します。'
            'クローラの超信地旋回スリップによる yaw 誤差は補正されません'
            '（長距離・廊下の再生では地図とずれます）。',
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
    # WAIVER(demo): W-06 — 'runaway'（DRIVE_RUNAWAY）を一時的に外している。
    #   WiFi 経由の /esp32/wheel_feedback は受信ギャップ（500ms 超）が起きるため、
    #   走行のたびに safety_monitor が「新鮮な指令 vs 古い実測」を比べて誤発火し、
    #   手動教示・教示再生のデモが成立しない（実機フィードバック 2026-09-01）。
    #   物理非常停止と ESP32 ウォッチドッグ（600ms）は有効なので真の暴走は止まる。
    #   特例解除時に wheel_feedback の鮮度ゲート＋回頭中の Case A 除外＋実測較正で
    #   runaway_hold_ms を右サイズ化してから 'runaway' を戻す。docs/plan/EXCEPTION-LEDGER.md W-06。
    SAFETY_ENABLED_TARGETS = ['lidar', 'esp32', 'state', 'firmware', 'limiter']
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
    #
    # WP-ONSITE-P0: DR-SPAAM のモデルロードを起動から perception_start_delay 秒だけ
    # 遅らせる。N-27（Nav2 のライフサイクル起動と DR-SPAAM のモデルロードが同じ時間帯に
    # 走ると PC 側が一過性にストールし LIMITER_DEAD → ESTOP）は 2026-08-31 に対処 (a)
    # ＝「段階で出し分け」で回避したが、試験場内デモは stage:=4 で Nav2 と DR-SPAAM を
    # 両方立てるため N-27 の条件が戻る。critical_fault_hold_ms(300ms) が当時の 260ms
    # ストールは吸収するが、両者の重なりそのものを減らして裕度を稼ぐ。
    perception_start_delay = LaunchConfiguration('perception_start_delay')
    _perception_real = PythonExpression(
        ["'", use_stub, "' != 'true' and int('", stage, "') >= 4"])
    nodes.append(TimerAction(
        period=perception_start_delay,
        actions=[
            IncludeLaunchDescription(
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
            ),
            Node(
                package='th_perception',
                executable='person_tracker_bridge.py',
                name='person_tracker_bridge',
                output='screen',
            ),
        ],
        condition=IfCondition(_perception_real),
    ))

    # ── 10. person_predictor ──────────────────────────────
    # WS-9E: 出力先の /cmd_vel_retreat は WP-SAFE-03 以降 twist_mux が購読せず、
    # 入力 /person/status も stage:=1 / use_stub:=false では来ない（廃止対象）。
    # 常時起動は CPU を無駄に食うだけなので /person/status の publisher が居る時だけゲート。
    nodes.append(Node(
        package='th_perception',
        executable='person_predictor.py',
        name='person_predictor',
        condition=IfCondition(person_logic_enabled),
        output='screen',
    ))

    # ── 11. follow_planner ────────────────────────────────
    # WS-9E: /cmd_vel_retreat が twist_mux に購読されず、/person/status も
    # stage:=1 / use_stub:=false では来ない（廃止対象）。person_logic_enabled でゲート。
    nodes.append(Node(
        package='th_planning',
        executable='follow_planner.py',
        name='follow_planner',
        condition=IfCondition(person_logic_enabled),
        parameters=[os.path.join(BRINGUP_DIR, 'config', 'planning_params.yaml')],
        output='screen',
    ))

    # ── 11b. follow_planner_mapless (MAP不要の純粋軌跡追従モード。FOLLOWING_MAPLESS 時のみ動作)
    # WS-9E: 上と同じく output は /cmd_vel_retreat（購読なし）・input は /person/status
    # （stage:=1 / use_stub:=false では来ない。廃止対象）。person_logic_enabled でゲート。
    nodes.append(Node(
        package='th_planning',
        executable='follow_planner_mapless.py',
        name='follow_planner_mapless',
        condition=IfCondition(person_logic_enabled),
        parameters=[os.path.join(BRINGUP_DIR, 'config', 'planning_params.yaml')],
        output='screen',
    ))

    # ── 12. panel_navigator ───────────────────────────────
    # 2026-09-10（WS-9AB 系）: 旧試験場内ナビ。th_onsite の venue_navigator が
    # 置換済み（DetailedDesign-reuse.md）。onsite（stage>=3）では立てない
    # ── 旧 /robot/mode でしか動かず venue_navigator と別アクション（NavigateToPose
    # vs compute_path_to_pose/follow_path）なのでゴール衝突は無いが CPU を食う。
    nodes.append(Node(
        package='th_planning',
        executable='panel_navigator.py',
        name='panel_navigator',
        parameters=[{
            'panels_yaml': os.path.join(BRINGUP_DIR, 'config', 'panels.yaml'),
        }],
        output='screen',
        condition=UnlessCondition(onsite_enabled),
    ))

    # ── 12b. summon_navigator ──────────────────────────────
    nodes.append(Node(
        package='th_planning',
        executable='summon_navigator.py',
        name='summon_navigator',
        parameters=[os.path.join(BRINGUP_DIR, 'config', 'planning_params.yaml')],
        output='screen',
        condition=UnlessCondition(onsite_enabled),
    ))

    # ── 13. manual_command_handler ────────────────────────
    nodes.append(Node(
        package='th_planning',
        executable='manual_command_handler.py',
        name='manual_command_handler',
        output='screen',
    ))

    # WS-8B: enable_route_slam のときだけ教示・再生を map フレーム追従にする
    # （TF リスナ・/route/robot_pose もこのときだけ有効。通常起動では挙動不変）。
    use_map_frame = ParameterValue(
        PythonExpression(["'", enable_route_slam, "'.lower() in ('true', '1')"]),
        value_type=bool)

    # WS-9K-B: 地図セッション ID。地図は bringup ごとに作り直されるので、
    # 「この bringup の起動」を 1 つの地図セッションとみなし、1 回だけ生成して
    # route_recorder と replay_runner の両方へ同じ値を渡す（各ノードが自分で作ると
    # 起動時刻が僅かにずれて一致しない）。route_recorder はこの ID を map フレーム
    # 経路に刻み、replay_runner は一致しない map 経路の再生を拒否する。
    map_session_id = f'sess_{int(time.time() * 1000)}'

    # ── 13a. route_recorder（教示経路の記録。WP-TRANSIT / demo-teach-replay）──
    nodes.append(Node(
        package='th_planning',
        executable='route_recorder.py',
        name='route_recorder',
        parameters=[{'use_map_frame': use_map_frame,
                     'map_session_id': map_session_id}],
        output='screen',
    ))

    # ── 13a'. replay_runner（教示再生の走行。WP-TRANSIT / demo-teach-replay）─
    nodes.append(Node(
        package='th_planning',
        executable='replay_runner.py',
        name='replay_runner',
        parameters=[{'use_map_frame': use_map_frame,
                     'map_session_id': map_session_id}],
        output='screen',
    ))

    # ── 13a''. map_downsampler（/map を表示用に間引いて /route/map_view へ配信。
    #    WS-9G）──
    # N-27 の「要らないものを起動しない」方針に沿い、enable_route_slam（＝/map が
    # 出る）ときだけ起動する。それ以外で動かす意味が無い（use_map_frame が
    # enable_route_slam から作られているのと同じ条件を使う）。
    nodes.append(Node(
        package='th_planning',
        executable='map_downsampler.py',
        name='map_downsampler',
        condition=IfCondition(PythonExpression(
            ["'", enable_route_slam, "'.lower() in ('true', '1')"])),
        output='screen',
    ))

    # ── 13a'''. map_downsampler（試験場内用。会場は狭いので間引かない）──
    # /route/map_view（factor=4）は校舎 1 周級の教示・再生用。試験場内は狭いので
    # factor=1（=間引かない）の別トピックを onsite 画面に配る。publish 周期は
    # /route/map_view と同じ 2s に絞ってあるので無線への追加負荷は 1 枚ぶん。
    # brief-onsite-ux2 F-2: /route/map_view と map_downsampler の既定 factor は
    # 変えない（S-13/S-14 が使う）。この 2 個目のインスタンスは S-20/S-21 専用。
    nodes.append(Node(
        package='th_planning',
        executable='map_downsampler.py',
        name='onsite_map_downsampler',
        parameters=[{'factor': 1, 'output_topic': '/onsite/map_view'}],
        condition=IfCondition(onsite_enabled),
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
    # WS-8B: enable_route_slam のときは起動時に localization モードへ倒さず、
    # slam_toolbox を既定の mapping モードのまま走らせる（教示・再生で map→odom を
    # 連続補正するため）。
    nodes.append(Node(
        package='th_config_manager',
        executable='slam_control.py',
        name='slam_control',
        parameters=[{
            'startup_mapping': ParameterValue(
                PythonExpression(
                    ["'", enable_route_slam, "'.lower() in ('true', '1')"]),
                value_type=bool),
            # WP-ONSITE-D: 試験場内地図（slot:VENUE）の保存先。CL-M-9。
            'venue_map_dir': '/root/th_data/venue',
        }],
        output='screen',
    ))

    # ── 13d. pin_registrar（試験場内 2 点指示・ピン登録。WP-ONSITE-01）──
    # /system/effect の begin_two_point / place_pin / reject_register を受け、
    # 対象の map 姿勢を venue/pins.yaml に永続化する。ピンの map 座標を得るには
    # SLAM の map→base_link TF が要るため、Nav2 と同じ段階 3 から起動する。
    nodes.append(LogInfo(
        condition=IfCondition(onsite_enabled),
        msg=PythonExpression(["'pin_registrar: onsite_enabled=' + ",
                              "('起動' if int('", stage, "') >= 3 else '省略(段階3から)')"]),
    ))
    nodes.append(Node(
        package='th_onsite',
        executable='pin_registrar.py',
        name='pin_registrar',
        condition=IfCondition(onsite_enabled),
        output='screen',
    ))

    # ── 13e. venue_navigator（盤前移動・向き合わせ。WP-ONSITE-02）──
    # PANEL_NAV / SUMMON の NAV→ALIGN。Nav2 の FollowPath を使うため段階 3。
    nodes.append(Node(
        package='th_onsite',
        executable='venue_navigator.py',
        name='venue_navigator',
        condition=IfCondition(onsite_enabled),
        output='screen',
    ))

    # ── 13f. wait_clear_gate（退避待ちゲート。WP-ONSITE-03）──
    # SUMMON/WAIT_CLEAR で試験員の退避を待って evt.clear_ok / evt.clear_timeout。
    nodes.append(Node(
        package='th_onsite',
        executable='wait_clear_gate.py',
        name='wait_clear_gate',
        condition=IfCondition(onsite_enabled),
        output='screen',
    ))

    # ── 13g. home_declarer（待機場所の宣言。WP-ONSITE-E）──
    # /onsite/declare_home サービスと /onsite/home_declared を提供する。
    # SLAM の map→base_link TF と待機場所ピンを照合するため段階 3 から起動する。
    # パラメータは全てノード内に既定値があるため launch からは渡さない。
    nodes.append(Node(
        package='th_onsite',
        executable='home_declarer.py',
        name='home_declarer',
        condition=IfCondition(onsite_enabled),
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

    # ── 16. SLAM Toolbox (map_yaml が空 = デフォルト) ──
    # WS-9N (2026-09-03): enable_route_slam でも
    # map_and_localization_slam_toolbox_node を使う。
    #
    # WS-8B は async_slam_toolbox_node を選んだ。当時のワークフローは「教示も再生も
    # 連続 mapping」だけで、mapping ⇄ localization の切替が要らなかったため。
    # WS-9L でワークフローが「教示 → 地図を凍結して保存 → 再生前に読み直して
    # 自己位置推定」に変わったのに、ノードの選択を見直していなかった。
    #
    # async_slam_toolbox_node に出来るのは「地図作成（ついでに自己位置推定）」か
    # 「pause_new_measurements で全部止める（自己位置推定も止まる）」の二択で、
    # 「地図を凍結したまま自己位置推定だけ続ける」が出来ない。その結果、再生中も
    # 地図作成が動き続け、ずれた自己位置に新しいスキャンを描き足して地図が汚れ、
    # 推定がさらにずれる悪循環になった（2026-09-03 実機: 再生の READY で
    # map→base_link が経路の始点から 1.52 m / 41.8° ずれ、/map が 0.5 Hz で
    # 更新され続けていた）。
    #
    # map_and_localization_slam_toolbox_node は /slam_toolbox/set_localization_mode
    # (std_srvs/SetBool) で同一プロセスのまま切替でき、「地図作成停止 = 地図凍結・
    # 自己位置推定継続」の要件を満たす唯一の手段 (VISION.md §8)。
    # 2026-09-03 に実機コンテナで単体起動を確認済み（35 秒生存・/map を publish・
    # set_localization_mode / deserialize_map / serialize_map / pause_new_measurements
    # がすべて存在）。respawn: 2026-08-07 実機で SIGSEGV を確認しているため残す。
    nodes.append(Node(
        package='slam_toolbox',
        executable='map_and_localization_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_yaml, {'use_sim_time': False}],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(PythonExpression(
            ["'", map_yaml, "' == '' and ('", enable_route_slam,
             "'.lower() in ('true', '1') or int('", stage, "') >= 3)"])),
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
