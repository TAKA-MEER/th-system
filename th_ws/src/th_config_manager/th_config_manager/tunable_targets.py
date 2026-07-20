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
}
