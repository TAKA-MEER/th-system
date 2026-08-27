#!/usr/bin/env python3
"""
teleop.launch.py — キーボード手動操作
========================================
シミュレーション・実機共通で使用できる。

使い方:
  # 通常操作 (MANUAL モード、twist_mux 経由)
  ros2 launch th_bringup teleop.launch.py

  # バックアップ手段 (twist_mux バイパス。下の警告を必ず読むこと)
  ros2 launch th_bringup teleop.launch.py direct:=true

引数:
  direct   true  → /cmd_vel へ直接送る (バックアップ手段)
           false → /cmd_vel_nav 経由で twist_mux を通す (デフォルト)

★★★ direct:=true は安全チェーンを丸ごと迂回する ★★★

`/cmd_vel` へ直接送るため、次のいずれも効かない:
  - `/safety/estop`     (twist_mux の lock priority 255)
  - `/safety/fault_lock` (同 254)
  - `obstacle_limiter`  (WP-SAFE-03 で最終段に入った障害物リミッタ)

**非常停止を押しても止まらない。**止める手段は物理非常停止と
ESP32 のウォッチドッグ (600ms) だけになる。

地図作成は WebUI の手動操作で行うのが通常手順で、この direct モードは
**それが使えないときのバックアップ**として残してある (2026-08-27 判断。
DetailedDesign-open.md の N-18)。使うときは人が張り付くこと。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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
                # 超信地旋回 [rad/s]。0.80 から下げた (2026-08-07)。
                # 地図作成はこのテレオペで走るため、旋回時の yaw 誤差
                # (クローラーのスリップ・スキャンの時刻ズレ・スキャン内の
                #  モーション歪み。いずれも角速度に比例) が地図品質へ直結する。
                # Nav2 側 rotate_to_heading_angular_vel を 0.4 に下げたのと同じ理由。
                # 走行中は teleop の [ / ] キーで 0.1 刻みに変えられる。
                'spin_speed':      0.40,
                'publish_rate_hz': 10.0,
            }],
            prefix='xterm -fa "Monospace" -fs 11 -e',
            output='screen',
            condition=UnlessCondition(direct),
        ),

        # ── クローラー専用テレオペ: direct モード (/cmd_vel) ────────
        # ★ 安全チェーン (estop / fault_lock / obstacle_limiter) を迂回する。
        #   冒頭の警告を参照。バックアップ手段として意図的に残している (N-18)。
        Node(
            package='th_planning',
            executable='crawler_teleop.py',
            name='crawler_teleop_direct',
            parameters=[{
                'publish_topic':   '/cmd_vel',
                'linear_speed':    0.20,
                'spin_speed':      0.40,   # 上記 /cmd_vel_nav 側と同じ理由
                'publish_rate_hz': 10.0,
            }],
            prefix='xterm -fa "Monospace" -fs 11 -e',
            output='screen',
            condition=IfCondition(direct),
        ),
    ])
