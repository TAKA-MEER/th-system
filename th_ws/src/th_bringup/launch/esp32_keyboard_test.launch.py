#!/usr/bin/env python3
"""
esp32_keyboard_test.launch.py — ESP32以降のみのキーボード操作テスト
====================================================================
LiDAR・安全監視・twist_mux を含まない最小構成。
keyboard → /cmd_vel → esp32_bridge → wheel_cmd → micro_ros_agent → ESP32 → モーター

使い方:
  ros2 launch th_bringup esp32_keyboard_test.launch.py
  ros2 launch th_bringup esp32_keyboard_test.launch.py device:=/dev/ttyUSB0

引数:
  device   ESP32 シリアルポート (デフォルト: /dev/esp32)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device = LaunchConfiguration('device')

    esp32_params = os.path.join(
        get_package_share_directory('th_esp32_bridge'),
        'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'device', default_value='/dev/esp32',
            description='ESP32 シリアルポート (例: /dev/ttyUSB0)'),

        # 1. micro-ROS Agent — bringup.launch.py と同一引数、device のみ可変
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['serial', '--dev', device, '-b', '115200', '-v6',
                       '--ros-args', '--log-level', 'warn'],
            output='screen',
        ),

        # 2. ESP32 ブリッジ (/cmd_vel → 差動駆動 → wheel_cmd)
        Node(
            package='th_esp32_bridge',
            executable='esp32_bridge',
            name='esp32_bridge',
            parameters=[esp32_params],
            output='screen',
        ),

        # 3. キーボードテレオペ (direct モード: /cmd_vel へ直接送信)
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
