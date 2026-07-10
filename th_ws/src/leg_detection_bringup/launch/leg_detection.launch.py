"""LiDAR-only leg detection.

Pipeline (no camera, no driving control):

    LaserScan --> dr_spaam_ros (DR-SPAAM leg detector) --> PoseArray of legs
              --> PersonTracker (leg mode, Kalman filter)
              --> tracked leg coordinate (FollowingPosition + PointStamped)

Everything robot-specific is a launch argument, so this works on any robot
that publishes a 2D LaserScan and a TF from the laser frame to `target_frame`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _setup(context):
    bringup_share = FindPackageShare("leg_detection_bringup")

    scan_topic = LaunchConfiguration("scan_topic").perform(context)
    dr_spaam_topic = LaunchConfiguration("dr_spaam_topic").perform(context)
    target_frame = LaunchConfiguration("target_frame").perform(context)
    scan_frame = LaunchConfiguration("scan_frame").perform(context)
    odom_frame = LaunchConfiguration("odom_frame").perform(context)
    autostart = str(LaunchConfiguration("autostart").perform(context)).strip().lower() \
        in ("true", "1", "yes", "on")

    tracker_params = PathJoinSubstitution([bringup_share, "param", "leg_tracker_param.yaml"])
    dr_spaam_params = PathJoinSubstitution([bringup_share, "param", "dr_spaam_param.yaml"])

    # DR-SPAAM leg detector (external package: 2d_lidar_person_detection)
    dr_spaam_node = Node(
        package="dr_spaam_ros",
        executable="dr_spaam_ros",
        name="dr_spaam_ros",
        namespace="dr_spaam",
        output="screen",
        # CPU 推論中に稀にセグフォで落ちることが実機で確認されたため自動再起動する
        # (落ちたままだと /person/status が途絶し PERSON_TRACKER_LOST が出続ける)
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            dr_spaam_params,
            {
                "scan_topic_name": scan_topic,
                "auto_configure": autostart,
                "auto_activate": autostart,
            },
        ],
    )

    # PersonTracker in leg mode -> outputs the tracked leg coordinate
    tracker_container = ComposableNodeContainer(
        name="leg_detection_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="multiple_sensor_person_tracking",
                plugin="multiple_sensor_person_tracking::PersonTracker",
                name="person_tracker",
                namespace="",
                parameters=[
                    tracker_params,
                    {
                        "scan_topic_name": scan_topic,
                        "dr_spaam_topic_name": dr_spaam_topic,
                        "target_frame": target_frame,
                        "scan_frame_name": scan_frame,
                        "odom_frame_name": odom_frame,
                    },
                ],
            ),
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="leg_detection_lifecycle_manager",
        namespace="",
        output="screen",
        parameters=[{
            "autostart": autostart,
            "bond_timeout": 0.0,
            "node_names": ["person_tracker"],
        }],
    )

    rviz_config = PathJoinSubstitution([bringup_share, "rviz", "leg_detection.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return [dr_spaam_node, tracker_container, lifecycle_manager, rviz_node]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "scan_topic", default_value="/scan",
            description="Input 2D LaserScan topic."),
        DeclareLaunchArgument(
            "dr_spaam_topic", default_value="/dr_spaam/dr_spaam_detections",
            description="PoseArray of detected legs published by dr_spaam_ros."),
        DeclareLaunchArgument(
            "target_frame", default_value="base_link",
            description="Frame the leg coordinate is expressed in. "
                        "Set equal to the laser frame for a single-LiDAR setup."),
        DeclareLaunchArgument(
            "scan_frame", default_value="",
            description="Override the LaserScan frame_id (empty = use the message header)."),
        DeclareLaunchArgument(
            "odom_frame", default_value="odom",
            description="Odom frame for the auxiliary target_position_odom output "
                        "(unused if no odom TF is available)."),
        DeclareLaunchArgument(
            "use_rviz", default_value="false",
            description="Launch RViz."),
        DeclareLaunchArgument(
            "autostart", default_value="true",
            description="Auto configure+activate the lifecycle nodes."),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
