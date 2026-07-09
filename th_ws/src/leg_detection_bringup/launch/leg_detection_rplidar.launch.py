"""All-in-one launch for the RPLIDAR (A3/S1) leg-detection setup.

One command brings up everything:

    RPLIDAR driver (sllidar_ros2)  -->  /scan
        --> leg_detection.launch.py (DR-SPAAM + PersonTracker, leg mode)
        --> RViz (preconfigured: gray raw scan, white/green cloud, target coord text)

Usage:
    ros2 launch leg_detection_bringup leg_detection_rplidar.launch.py

Override the serial port if needed:
    ros2 launch leg_detection_bringup leg_detection_rplidar.launch.py serial_port:=/dev/ttyUSB1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("leg_detection_bringup")

    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    scan_frame = LaunchConfiguration("scan_frame")
    # NOTE: named "rviz" (not "use_rviz") on purpose. IncludeLaunchDescription does
    # not scope launch configurations, so the use_rviz:="false" we pass to the inner
    # leg_detection.launch.py would otherwise overwrite this value and skip our RViz.
    rviz = LaunchConfiguration("rviz")

    args = [
        DeclareLaunchArgument(
            "serial_port", default_value="/dev/ttyUSB0",
            description="RPLIDAR serial device."),
        DeclareLaunchArgument(
            "serial_baudrate", default_value="256000",
            description="RPLIDAR baudrate (A3/S1 = 256000, A1/A2 = 115200, S2 = 1000000)."),
        DeclareLaunchArgument(
            "scan_frame", default_value="laser_frame",
            description="frame_id stamped on the LaserScan and used as the tracker target frame."),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="Launch the preconfigured RViz view (rviz:=false to disable)."),
    ]

    # --- RPLIDAR driver -> /scan ---
    sllidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        output="screen",
        parameters=[{
            "channel_type": "serial",
            "serial_port": serial_port,
            "serial_baudrate": serial_baudrate,
            "frame_id": scan_frame,
            "angle_compensate": True,
        }],
    )

    # --- DR-SPAAM + PersonTracker (leg mode) ---
    # Single-LiDAR: target/odom frame = laser frame -> TF is identity, no extra TF needed.
    pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "leg_detection.launch.py"])),
        launch_arguments={
            "scan_topic": "/scan",
            "target_frame": scan_frame,
            "scan_frame": scan_frame,
            "odom_frame": scan_frame,
            "use_rviz": "false",   # RViz handled below with our config
            "autostart": "true",
        }.items(),
    )

    # --- RViz with the preconfigured view ---
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution([bringup_share, "rviz", "leg_detection.rviz"]),
        ],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(args + [sllidar_node, pipeline, rviz_node])
