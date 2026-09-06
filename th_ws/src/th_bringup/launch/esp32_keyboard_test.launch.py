#!/usr/bin/env python3
"""
esp32_keyboard_test.launch.py — ESP32以降のみのキーボード操作テスト
====================================================================
LiDAR・安全監視・twist_mux を含まない最小構成。
keyboard → /cmd_vel → esp32_bridge(WebSocketサーバー) → pi_serial_relay(WebSocket
クライアント、ラズパイ) → ESP32(シリアル) → モーター
(2026-09-05: ESP32↔PC間の無線WebSocketを廃止し、ラズパイ経由のシリアル接続に
変更。esp32_bridge 自体・このノードの起動方法は無変更。VISION.md 参照)

使い方:
  ros2 launch th_bringup esp32_keyboard_test.launch.py

esp32_bridge のアドレス(config/params.yaml の ws_host/ws_port)へは
ラズパイの pi_serial_relay が接続しに来る(ESP32 自身はもう WiFi を使わない)。
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    esp32_params = os.path.join(
        get_package_share_directory('th_esp32_bridge'),
        'config', 'params.yaml')

    return LaunchDescription([
        # 1. ESP32 ブリッジ (/cmd_vel → 差動駆動 → WebSocket → ESP32)
        Node(
            package='th_esp32_bridge',
            executable='esp32_bridge.py',
            name='esp32_bridge',
            parameters=[esp32_params],
            output='screen',
        ),

        # 2. キーボードテレオペ (direct モード: /cmd_vel へ直接送信)
        #    teleop.launch.py の IfCondition(direct) ブランチと同一設定
        Node(
            package='th_planning',
            executable='crawler_teleop.py',
            name='crawler_teleop',
            parameters=[{
                'publish_topic':   '/cmd_vel',
                'linear_speed':    0.20,
                'spin_speed':      0.80,
                'publish_rate_hz': 10.0,
            }],
            prefix='xterm -fa "Monospace" -fs 11 -e',
            output='screen',
        ),
    ])
