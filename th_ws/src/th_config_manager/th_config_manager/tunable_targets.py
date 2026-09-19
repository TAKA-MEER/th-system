"""
tunable_targets.py
===================
WebUI から調整可能なパラメータのレジストリ（ROS2 非依存の純粋 Python）。

config_manager ノードはこの辞書を見て、どのノードのどのパラメータを
中継し、どの YAML ファイルのどのブロックに書き戻すかを決める。

新しいチューニング可能パラメータを追加する手順は
docs/architecture.md の「WebUI 設定パネル」セクションを参照。
"""

TUNABLE_TARGETS = {
    "follow_planner_mapless": {
        "yaml_package": "th_bringup",
        "yaml_relpath": "config/planning_params.yaml",
        "block_key": "follow_planner_mapless",
        # odom_frame / base_frame は文字列（配線設定であり調整値ではない）ため対象外。
        # update_rate_hz も対象外: 制御ループのタイマー周期は起動時に固定される
        # ため、ライブ変更しても実際の周期は変わらない上、mapless_follow_core.py の
        # max_up/max_down 計算で除数として使われておりゼロ除算でノードが
        # クラッシュしうる（実機検証で確認）。
        "params": [
            "lookback_distance",
            "trail_sample_interval_m",
            "trail_max_points",
            "stop_distance",
            "resume_distance",
            "obstacle_check_distance_m",
            "obstacle_check_half_width_deg",
            "v_max",
            "k_ang",
            "stop_radius_m",
            "w_max_rad_s",
            "max_linear_accel_mps2",
            "max_linear_decel_mps2",
            "max_angular_accel_rad_s2",
        ],
    },
    "lidar_filter": {
        "yaml_package": "th_bringup",
        "yaml_relpath": "config/perception_params.yaml",
        "block_key": "lidar_filter",
        "params": [
            "blind_angle_ranges",
        ],
    },
    # WS-9W: 再生（教示再生）の自己位置推定の当たり方は現場（廊下の長さ・特徴量）で
    # 最適値が変わる。slam_toolbox のスキャンマッチ関連をチューニング対象にする。
    #
    # 重要な癖: slam_toolbox はランタイムのパラメータコールバックを持たず、Karto の
    # マッパーは起動時に確定する。config_manager の set_parameters は**値を保持する
    # だけで、その場では効かない**。ただし WS-9S で「この経路で進む」のたびに
    # slam_toolbox を respawn して slam_params.yaml を読み直すので、
    # 「変更 →『YAML に保存』→ 経路を選び直す」で新しい値が効く。
    #
    # 対象外にしたもの: resolution / max_laser_range / min_laser_range（センサ・地図の
    # 基本仕様であり調整値ではない）、do_loop_closing（WS-9K で false 固定。
    # test_ekf_config.test_slam_loop_closing_is_disabled が縛る）。
    "slam_toolbox": {
        "yaml_package": "th_bringup",
        "yaml_relpath": "config/slam_params.yaml",
        "block_key": "slam_toolbox",
        "params": [
            "minimum_travel_distance",             # 補正する間隔（距離 m）
            "minimum_travel_heading",              # 補正する間隔（角度 rad）
            "correlation_search_space_dimension",  # スキャンマッチ探索窓（全幅 m。半分が片側）
            "correlation_search_space_resolution", # 探索の刻み（m）
            "link_match_minimum_response_fine",    # マッチ受理の最小相関
        ],
    },
}
