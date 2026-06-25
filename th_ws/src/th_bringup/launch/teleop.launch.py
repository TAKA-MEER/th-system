#!/usr/bin/env python3
"""
teleop.launch.py — キーボード手動操作
========================================
シミュレーション・実機共通で使用できる。
MANUAL モード時に /cmd_vel_nav へ直接速度指令を送る。

使い方:
  ros2 launch th_bringup teleop.launch.py

  # 地図作成時（Nav2 なしで直接 /cmd_vel へ）
  ros2 launch th_bringup teleop.launch.py direct:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('direct', default_value='false',
            description='true=/cmd_vel に直接送る, false=/cmd_vel_nav 経由(twist_mux通過)'),

        # /cmd_vel_nav 経由（通常の MANUAL 操作）
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            remappings=[('cmd_vel', '/cmd_vel_nav')],
            prefix='xterm -e',
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('direct')),
        ),

        # /cmd_vel に直接送る（SLAM 地図作成時など）
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard_direct',
            remappings=[('cmd_vel', '/cmd_vel')],
            prefix='xterm -e',
            output='screen',
            condition=IfCondition(LaunchConfiguration('direct')),
        ),
    ])
