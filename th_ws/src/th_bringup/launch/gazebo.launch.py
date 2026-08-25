#!/usr/bin/env python3
"""
gazebo.launch.py — Gazebo シミュレーション起動ファイル
==========================================================
実機モード (sim:=false) とシミュレーションモード (sim:=true) を
単一の launch ファイルで切り替える。

シナリオプリセット:
  scenario:=<name> で config/scenarios/<name>.yaml を読み込み、
  ワールド・地図・スポーン位置・人物経路・障害物・プランナ上書きを
  一括設定する。優先順位: CLI 明示指定 > シナリオプリセット > 従来デフォルト。
  scenario 未指定時は従来 (panel_room + patrol) と同一挙動。

  例: ros2 launch th_bringup gazebo.launch.py scenario:=narrow_room
"""

import glob
import os
import sys
import yaml
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

# WP-PARAM-02: registry.yaml → /root/th_data/generated/*.yaml のパラメータ生成
# ヘルパー。launch ファイルと同じディレクトリに share インストールされるが
# sys.path には自動で乗らないため、自分でパスを通してから import する。
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from params_generation import GENERATED_DIR, make_opaque_function  # noqa: E402

BRINGUP_DIR  = get_package_share_directory('th_bringup')
DESC_DIR     = get_package_share_directory('th_description')
NAV2_DIR     = get_package_share_directory('nav2_bringup')
SLAM_DIR     = get_package_share_directory('slam_toolbox')
SAFETY_DIR   = get_package_share_directory('th_safety')

def _set_sim_time(context, *args, **kwargs):
    is_sim = LaunchConfiguration('sim').perform(context).lower() in ('true', '1', 'yes')
    return [SetParameter(name='use_sim_time', value=is_sim)]  # Python bool を渡す


# ── シナリオ未指定時の従来デフォルト ─────────────────────────
_LEGACY = {
    'world':     'panel_room_no_actor.world',
    'robot_x':   -4.0,
    'robot_y':   -3.0,
    'robot_yaw':  0.0,
    'slam':       True,
    'map_yaml':   '',
    'obstacle':   True,
}


def _load_scenario(name: str) -> dict:
    """config/scenarios/<name>.yaml を読み込む。未存在なら候補一覧付きでエラー。"""
    scen_dir = os.path.join(BRINGUP_DIR, 'config', 'scenarios')
    path = os.path.join(scen_dir, name + '.yaml')
    if not os.path.exists(path):
        avail = sorted(
            os.path.splitext(os.path.basename(p))[0]
            for p in glob.glob(os.path.join(scen_dir, '*.yaml')))
        raise RuntimeError(
            f"シナリオ '{name}' が見つかりません "
            f"({path})。利用可能なシナリオ: {', '.join(avail) or '(なし)'}")
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    return data.get('scenario', {}) or {}


def _scenario_setup(context, *args, **kwargs):
    """シナリオ依存アクションを構築する OpaqueFunction。

    優先順位: CLI 明示指定 > シナリオプリセット > 従来デフォルト。
    launch 引数 world / robot_x / robot_y / robot_yaw / map_yaml は
    センチネル ''、slam / obstacle は 'auto' が「未指定」を表す。
    """
    is_sim    = LaunchConfiguration('sim').perform(context).lower() in ('true', '1', 'yes')
    scen_name = LaunchConfiguration('scenario').perform(context).strip()
    scen      = _load_scenario(scen_name) if scen_name else {}

    def cli(name, sentinel=''):
        v = LaunchConfiguration(name).perform(context)
        return None if v == sentinel else v

    def pick(cli_name, scen_value, legacy, sentinel=''):
        v = cli(cli_name, sentinel)
        if v is not None:
            return v
        return legacy if scen_value is None else scen_value

    # ── 有効値の解決 ─────────────────────────────────────────
    world_v = pick('world', scen.get('world'), _LEGACY['world'])
    if not os.path.isabs(world_v):
        world_v = os.path.join(BRINGUP_DIR, 'worlds', world_v)

    spawn = scen.get('robot_spawn') or {}
    rx   = float(pick('robot_x',   spawn.get('x'),   _LEGACY['robot_x']))
    ry   = float(pick('robot_y',   spawn.get('y'),   _LEGACY['robot_y']))
    ryaw = float(pick('robot_yaw', spawn.get('yaw'), _LEGACY['robot_yaw']))

    slam_v = pick('slam', scen.get('slam'), _LEGACY['slam'], sentinel='auto')
    slam_on = str(slam_v).lower() in ('true', '1', 'yes')

    map_v = pick('map_yaml', scen.get('map_yaml') or None, _LEGACY['map_yaml'])
    if map_v and not os.path.isabs(map_v):
        map_v = os.path.join(BRINGUP_DIR, 'maps', map_v)

    person = scen.get('person') or {}
    relay  = scen.get('relay') or {}
    obst   = scen.get('obstacle') or {}
    obst_v = pick('obstacle', obst.get('enabled'), _LEGACY['obstacle'], sentinel='auto')
    obst_on = str(obst_v).lower() in ('true', '1', 'yes')
    overrides = scen.get('planning_overrides') or {}

    planning_yaml = os.path.join(BRINGUP_DIR, 'config', 'planning_params.yaml')
    nav2_params_sim  = os.path.join(BRINGUP_DIR, 'config', 'nav2_params_sim.yaml')
    nav2_params_real = os.path.join(BRINGUP_DIR, 'config', 'nav2_params.yaml')
    slam_params_sim  = os.path.join(BRINGUP_DIR, 'config', 'slam_params_sim.yaml')
    slam_params_real = os.path.join(BRINGUP_DIR, 'config', 'slam_params.yaml')

    actions = []
    if scen_name:
        actions.append(LogInfo(msg=[
            f"[scenario:{scen_name}] {scen.get('description', '')} "
            f"(推奨モード {scen.get('recommended_mode', '-')}: "
            f"ros2 service call /mode_manager/set_mode th_system_msgs/srv/SetMode "
            f"\"{{requested_mode: {scen.get('recommended_mode', 2)}, requester: 'cli'}}\")"]))

    # ── 追従プランナ (実機・シミュ共通。シナリオ上書きは dict 後勝ち) ──
    def planner_params(node_key):
        ov = overrides.get(node_key)
        return [planning_yaml] + ([dict(ov)] if ov else [])

    actions += [
        Node(
            package='th_planning',
            executable='follow_planner.py',
            name='follow_planner',
            parameters=planner_params('follow_planner'),
            output='screen',
        ),
        Node(
            package='th_planning',
            executable='follow_planner_mapless.py',
            name='follow_planner_mapless',
            parameters=planner_params('follow_planner_mapless'),
            output='screen',
        ),
    ]

    # ── SLAM / Localization (有効 slam/map 値で分岐) ─────────
    slam_params = slam_params_sim if is_sim else slam_params_real
    nav2_params = nav2_params_sim if is_sim else nav2_params_real
    sim_time_s  = 'True' if is_sim else 'False'
    if slam_on:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(SLAM_DIR, 'launch', 'online_async_launch.py')),
            launch_arguments={
                'slam_params_file': slam_params,
                'use_sim_time':     sim_time_s,
            }.items(),
        ))
    elif map_v:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(NAV2_DIR, 'launch', 'localization_launch.py')),
            launch_arguments={
                'map':          map_v,
                'use_sim_time': sim_time_s,
                'params_file':  nav2_params,
                'autostart':    'true',
            }.items(),
        ))
    else:
        actions.append(LogInfo(msg=[
            '[th_bringup] slam=false かつ map_yaml が空のため '
            'map_server/AMCL を起動しません']))

    if not is_sim:
        return actions

    # ── 以下シミュレーション専用 ─────────────────────────────
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world':   world_v,
            'verbose': 'false',
            'pause':   'false',
        }.items(),
    ))

    actions.append(TimerAction(
        period=4.5,   # Gazebo 完全起動 + LiDAR/person_relay 安定化に余裕を持たせる
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_robot',
            arguments=[
                '-topic', '/robot_description',
                '-entity', 'th_robot',
                '-x', str(rx), '-y', str(ry), '-Y', str(ryaw),
            ],
            output='screen',
        )],
    ))

    # Gazebo 中継 → /person/status
    relay_params = {
        'actor_name':       'inspector',
        'robot_name':       'th_robot',
        'base_frame':       'base_link',
        # 部屋対角をカバー (デフォルト8mだと対角の試験員が圏外になる)
        'max_detect_range': float(relay.get('max_detect_range', 12.0)),
    }
    if relay.get('occlusion_check'):
        segs = [float(v) for v in (relay.get('occlusion_segments') or [])]
        relay_params['occlusion_check'] = True
        relay_params['occlusion_segments'] = segs
    actions.append(Node(
        package='th_perception',
        executable='gazebo_person_relay.py',
        name='gazebo_person_relay',
        parameters=[relay_params],
        output='screen',
    ))

    # 試験員シリンダー移動
    mover_params = {
        'pattern':    person.get('pattern', 'patrol'),
        'model_name': 'inspector',
        'move_speed': float(person.get('move_speed', 0.5)),
        'pause_sec':  float(person.get('pause_sec', 4.0)),
    }
    wps = [float(v) for v in (person.get('waypoints') or [])]
    if wps:
        mover_params['waypoints'] = wps
    actions.append(Node(
        package='th_perception',
        executable='person_mover.py',
        name='person_mover',
        parameters=[mover_params],
        output='screen',
    ))

    # ランダム移動障害物 wanderer
    if obst_on:
        bounds = obst.get('bounds') or {}
        actions.append(Node(
            package='th_perception',
            executable='obstacle_mover.py',
            name='obstacle_mover',
            parameters=[{
                'model_name': 'wanderer',
                'move_speed': float(obst.get('move_speed', 0.5)),
                'x_min': float(bounds.get('x_min', -4.0)),
                'x_max': float(bounds.get('x_max',  4.0)),
                'y_min': float(bounds.get('y_min', -3.0)),
                'y_max': float(bounds.get('y_max',  3.0)),
            }],
            output='screen',
        ))

    return actions

def generate_launch_description():

    # ── 引数定義 ─────────────────────────────────────────────
    declared_args = [
        DeclareLaunchArgument('sim',        default_value='true',
            description='true=Gazebo シミュレーション, false=実機'),
        DeclareLaunchArgument('scenario',   default_value='',
            description='シナリオプリセット名 (config/scenarios/<name>.yaml)。'
                        '未指定=従来挙動'),
        DeclareLaunchArgument('slam',       default_value='auto',
            description='true=SLAM マッピング, false=既存地図でナビ '
                        '(auto=シナリオ設定に従う。シナリオ未指定なら true)'),
        DeclareLaunchArgument('map_yaml',   default_value='',
            description='既存地図 YAML パス (slam:=false 時に使用。'
                        '未指定ならシナリオ設定に従う)'),
        DeclareLaunchArgument('use_stub',   default_value='false',
            description='true=試験員トラッカースタブを使用'),
        DeclareLaunchArgument('obstacle',   default_value='auto',
            description='true=ランダム移動する障害物(wanderer)を動かす(回避検証用) '
                        '(auto=シナリオ設定に従う。シナリオ未指定なら true)'),
        DeclareLaunchArgument('imu_enabled',default_value='false',
            description='true=IMU を EKF に追加'),
        DeclareLaunchArgument('rviz',       default_value='true',
            description='true=RViz2 を起動'),
        DeclareLaunchArgument('log_level',  default_value='info'),
        DeclareLaunchArgument('world',      default_value='',
            description='Gazebo ワールドファイル '
                        '(未指定ならシナリオ設定、それもなければ panel_room_no_actor.world)'),
        DeclareLaunchArgument('robot_x',    default_value=''),
        DeclareLaunchArgument('robot_y',    default_value=''),
        DeclareLaunchArgument('robot_yaw',  default_value=''),
        DeclareLaunchArgument('stage', default_value='1',
            description='params_generation.py が registry.yaml を解決するステージ'
                        '番号 (WP-PARAM-02)'),
    ]

    sim        = LaunchConfiguration('sim')
    slam       = LaunchConfiguration('slam')
    use_stub   = LaunchConfiguration('use_stub')
    rviz       = LaunchConfiguration('rviz')

    # ── URDF / robot_description ─────────────────────────────
    urdf_file = os.path.join(DESC_DIR, 'urdf', 'th_robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    # ── 設定ファイルパス ─────────────────────────────────────
    nav2_params_sim  = os.path.join(BRINGUP_DIR, 'config', 'nav2_params_sim.yaml')
    nav2_params_real = os.path.join(BRINGUP_DIR, 'config', 'nav2_params.yaml')
    safety_sim       = os.path.join(BRINGUP_DIR, 'config', 'safety_monitor_sim.yaml')
    safety_real      = os.path.join(SAFETY_DIR,  'config', 'safety_monitor.yaml')
    ekf_yaml         = os.path.join(BRINGUP_DIR, 'config', 'ekf_params.yaml')
    calib_yaml       = os.path.join(BRINGUP_DIR, 'config', 'calib.yaml')
    panels_yaml      = os.path.join(BRINGUP_DIR, 'config', 'panels.yaml')
    planning_yaml    = os.path.join(BRINGUP_DIR, 'config', 'planning_params.yaml')
    perc_yaml        = os.path.join(BRINGUP_DIR, 'config', 'perception_params.yaml')
    rviz_cfg         = os.path.join(BRINGUP_DIR, 'config', 'rviz', 'th_sim.rviz')

    # ── use_sim_time を全ノードに伝播 ────────────────────────
    sim_time_action = OpaqueFunction(function=_set_sim_time)

    # ── パラメータ生成 (WP-PARAM-02) ─────────────────────────
    # registry.yaml → /root/th_data/generated/*.yaml を、ノードを1つも起動する前に
    # 同期生成する（G-1）。アサーション違反なら例外で launch ごと止まる（G-2）。
    # sim launch 引数で --sim の可否を切り替える（sim:=true なら stage=1 の
    # blocking placeholder A8 を回避できる。実機は blocking のため stage:=1 では
    # 起動しない — 意図された挙動。詳細はコミットメッセージ参照）。
    params_generation_action = OpaqueFunction(
        function=make_opaque_function(sim_arg_name='sim'))

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
        # 静的 th_safety/config/twist_mux.yaml は読まない。generated/twist_mux.yaml
        # が階層構造(locks/topics)を完全に持つ唯一の情報源 (G-3, 二重管理の防止)。
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            parameters=[os.path.join(GENERATED_DIR, 'twist_mux.yaml')],
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
        # 静的ファイルを土台にし、registry.yaml 由来の生成ファイルを後段に重ねる (G-4)。
        Node(
            package='th_perception',
            executable='lidar_filter.py',
            name='lidar_filter',
            parameters=[perc_yaml, os.path.join(GENERATED_DIR, 'lidar_filter.yaml')],
            output='screen',
        ),
        # person_predictor
        Node(
            package='th_perception',
            executable='person_predictor.py',
            name='person_predictor',
            output='screen',
        ),
        # follow_planner / follow_planner_mapless は _scenario_setup 内で起動
        # (シナリオの planning_overrides を dict 後勝ちで適用するため)
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
        # summon_navigator（呼び寄せ）
        Node(
            package='th_planning',
            executable='summon_navigator.py',
            name='summon_navigator',
            parameters=[planning_yaml],
            output='screen',
        ),
        # config_manager (WebUI 設定パネル: パラメータ調整の仲介)
        Node(
            package='th_config_manager',
            executable='config_manager.py',
            name='config_manager',
            output='screen',
        ),
        # slam_control (WebUI: 地図作成 開始/停止の仲介)
        Node(
            package='th_config_manager',
            executable='slam_control.py',
            name='slam_control',
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
    # シナリオ依存アクション
    # (Gazebo 本体 / スポーン / person_relay / person_mover /
    #  obstacle_mover / SLAM・Localization / 追従プランナ)
    # は _scenario_setup 内で構築する
    # ════════════════════════════════════════════════════════
    scenario_action = OpaqueFunction(function=_scenario_setup)

    # enabled_targets (O-7): registry.yaml の既定値は空リストであり、かつ
    # export.py は空リストを生成物からサニタイズして落とす（D3）ため、実際の値は
    # launch から明示的に渡す（DetailedDesign-names.md §7.3 の note の実体）。
    # 対象は「その publisher が実際にこの launch で起動するか」で決める。
    #
    # sim: esp32_bridge / state_manager とも condition=UnlessCondition(sim) で
    # 起動しない（このファイル内で確認済み）ため、esp32・state は publisher が
    # 無い。runaway は /esp32/wheel_feedback（同じく無い）を要るため除外。
    # firmware も esp32_bridge が publisher のため除外。Gazebo の LiDAR センサ
    # プラグインは sim/実機を問わず /scan を出すため lidar だけ有効にする。
    SAFETY_ENABLED_TARGETS_SIM = ['lidar']
    # 実機: bringup.launch.py と同じ判断（WP-SAFE-01 完了報告に詳細）。
    SAFETY_ENABLED_TARGETS_REAL = ['lidar', 'esp32', 'runaway', 'state', 'firmware']

    # safety_monitor: シミュレーション設定
    # 静的ファイルを土台にし、registry.yaml 由来の生成ファイルを後段に重ねる (G-4)。
    safety_sim_node = Node(
        package='th_safety',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[safety_sim, os.path.join(GENERATED_DIR, 'safety_monitor.yaml'),
                    {'enabled_targets': SAFETY_ENABLED_TARGETS_SIM}],
        output='screen',
        condition=IfCondition(sim),
    )

    # safety_monitor: 実機設定
    safety_real_node = Node(
        package='th_safety',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[safety_real, os.path.join(GENERATED_DIR, 'safety_monitor.yaml'),
                    {'enabled_targets': SAFETY_ENABLED_TARGETS_REAL}],
        output='screen',
        condition=UnlessCondition(sim),
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
        # 静的ファイル（+ calib.yaml があれば）を土台にし、registry.yaml 由来の
        # 生成ファイルを最後段に重ねる (G-4)。
        parameters=[
            os.path.join(get_package_share_directory('th_esp32_bridge'),
                         'config', 'params.yaml'),
        ] + ([calib_yaml] if os.path.exists(calib_yaml) else [])
          + [os.path.join(GENERATED_DIR, 'esp32_bridge.yaml')],
        output='screen',
        condition=UnlessCondition(sim),
    )

    # person_tracker 本番実装 (human_kenchi: DR-SPAAM + PersonTracker leg モード)
    # 実機かつ use_stub:=false のときのみ有効。シミュレーションは常に gazebo_person_relay を使う。
    real_tracker_expr = PythonExpression(
        ["'", sim, "' == 'false' and '", use_stub, "' == 'false'"])

    leg_detection = IncludeLaunchDescription(
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
        condition=IfCondition(real_tracker_expr),
    )

    person_tracker_bridge = Node(
        package='th_perception',
        executable='person_tracker_bridge.py',
        name='person_tracker_bridge',
        output='screen',
        condition=IfCondition(real_tracker_expr),
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

    # Localization / SLAM は有効 slam・map_yaml の解決が必要なため
    # _scenario_setup 内で構築する

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

    return LaunchDescription(
        declared_args + [
            sim_time_action,
            LogInfo(msg=['[th_bringup] sim=', sim, ' slam=', slam,
                         ' scenario=', LaunchConfiguration('scenario')]),

            # パラメータ生成 (WP-PARAM-02)。ノードを1つも起動する前に
            # 同期生成する (G-1)。scenario_action / common_nodes より前に置く。
            params_generation_action,

            # シナリオ依存アクション
            # (Gazebo/スポーン/中継/人物移動/障害物/SLAM/Localization/追従プランナ)
            scenario_action,

            # URDF / robot_state_publisher
            *common_nodes,

            # safety_monitor（設定が異なる）
            safety_sim_node,
            safety_real_node,

            # スタブ試験員ソース
            person_stub,

            # 実機ノード
            lidar_node,
            esp32_bridge,
            ekf_node,
            leg_detection,
            person_tracker_bridge,

            # Nav2
            nav2_sim,
            nav2_real,

            # RViz2
            rviz_node,
        ]
    )