#!/usr/bin/env python3
"""
esp32_keyboard_test.launch.py — ESP32以降のみのキーボード操作テスト
====================================================================
LiDAR・安全監視・twist_mux を含まない最小構成。
keyboard → /cmd_vel → esp32_bridge(WebSocketサーバー) → ESP32(WebSocketクライアント) → モーター

使い方:
  ros2 launch th_bringup esp32_keyboard_test.launch.py

ESP32側は th_ws/esp32/src/wifi_credentials.h の WS_SERVER_HOST/WS_SERVER_PORT
で esp32_bridge のアドレスを指定する(config/params.yaml の ws_host/ws_port と
一致させること)。
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
