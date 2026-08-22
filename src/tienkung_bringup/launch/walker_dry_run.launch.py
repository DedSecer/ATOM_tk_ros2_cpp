from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("tienkung_bringup"))
    robot_config = str(share / "config" / "walker_config.yaml")
    node_config = str(share / "config" / "policy_runner.yaml")
    motion_config = str(share / "config" / "motion_source.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("policy_path", default_value=""),
        DeclareLaunchArgument("manifest_path", default_value=""),
        DeclareLaunchArgument("robot_config_path", default_value=robot_config),
        DeclareLaunchArgument("device", default_value="cpu"),
        DeclareLaunchArgument("zero_duration", default_value="2.0"),
        DeclareLaunchArgument("report_interval", default_value="5.0"),
        DeclareLaunchArgument("motion_file", default_value=""),
        DeclareLaunchArgument("use_motion_source", default_value="false"),
        DeclareLaunchArgument("joy_device", default_value="/dev/input/js0"),
        DeclareLaunchArgument("enable_runtime_log", default_value="false"),
        DeclareLaunchArgument("log_dir", default_value="/tmp"),
        Node(
            package="joy",
            executable="joy_node",
            name="walker_joy_node",
            output="screen",
            parameters=[{"device": LaunchConfiguration("joy_device")}],
            remappings=[("joy", "/sbus_data")],
        ),
        Node(
            package="tienkung_motion_source",
            executable="motion_source_node",
            name="walker_motion_source",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_motion_source")),
            parameters=[
                motion_config,
                {
                    "motion_file": LaunchConfiguration("motion_file"),
                    "robot_config_path": LaunchConfiguration("robot_config_path"),
                    "control_mode_topic": "/walker/control_mode",
                    "motion_reference_topic": "/walker/motion_reference",
                },
            ],
        ),
        Node(
            package="tienkung_policy_runner",
            executable="dry_run_node",
            name="walker_dry_run",
            output="screen",
            parameters=[
                node_config,
                {
                    "policy_path": LaunchConfiguration("policy_path"),
                    "manifest_path": LaunchConfiguration("manifest_path"),
                    "robot_config_path": LaunchConfiguration("robot_config_path"),
                    "device": LaunchConfiguration("device"),
                    "motion_reference_topic": "/walker/motion_reference",
                    "control_mode_topic": "/walker/control_mode",
                    "zero_duration_sec": ParameterValue(
                        LaunchConfiguration("zero_duration"), value_type=float
                    ),
                    "report_interval_sec": ParameterValue(
                        LaunchConfiguration("report_interval"), value_type=float
                    ),
                    "require_motion_source": ParameterValue(
                        LaunchConfiguration("use_motion_source"), value_type=bool
                    ),
                    "enable_runtime_log": ParameterValue(
                        LaunchConfiguration("enable_runtime_log"), value_type=bool
                    ),
                    "log_dir": LaunchConfiguration("log_dir"),
                },
            ],
        ),
    ])