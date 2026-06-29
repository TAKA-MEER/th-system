#!/usr/bin/env python3
"""
teleop.launch.py — キーボード手動操作
========================================
シミュレーション・実機共通で使用できる。

使い方:
  # 通常操作 (MANUAL モード、twist_mux 経由)
  ros2 launch th_bringup teleop.launch.py

  # SLAM 地図作成時 (twist_mux バイパス)
  ros2 launch th_bringup teleop.launch.py direct:=true

引数:
  direct   true  → /cmd_vel へ直接送る (SLAM 地図作成時)
           false → /cmd_vel_nav 経由で twist_mux を通す (デフォルト)

注意:
  このコマンドを実行したターミナル上でキー入力を受け付ける。
  別ウィンドウは開かない。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    direct = LaunchConfiguration('direct')

    return LaunchDescription([
        DeclareLaunchArgument(
            'direct', default_value='false',
            description='true=/cmd_vel に直接, false=/cmd_vel_nav 経由 (twist_mux 通過)'),

        # ── クローラー専用テレオペ: 通常モード (/cmd_vel_nav) ──────
        Node(
            package='th_planning',
            executable='crawler_teleop.py',
            name='crawler_teleop',
            parameters=[{
                'publish_topic':   '/cmd_vel_nav',
                'linear_speed':    0.20,   # m/s
                'spin_speed':      0.80,   # rad/s (超信地旋回)
                'publish_rate_hz': 10.0,
            }],
            output='screen',
            condition=UnlessCondition(direct),
        ),

        # ── クローラー専用テレオペ: direct モード (/cmd_vel) ────────
        Node(
            package='th_planning',
            executable='crawler_teleop.py',
            name='crawler_teleop_direct',
            parameters=[{
                'publish_topic':   '/cmd_vel',
                'linear_speed':    0.20,
                'spin_speed':      0.80,
                'publish_rate_hz': 10.0,
            }],
            output='screen',
            condition=IfCondition(direct),
        ),
    ])
