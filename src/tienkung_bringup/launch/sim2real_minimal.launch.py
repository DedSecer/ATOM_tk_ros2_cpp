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
    robot_config = str(share / "config" / "tg22_config.yaml")
    runner_config = str(share / "config" / "policy_runner.yaml")
    motion_config = str(share / "config" / "motion_source.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("policy_path", default_value=""),
        DeclareLaunchArgument("manifest_path", default_value=""),
        DeclareLaunchArgument("motion_file", default_value=""),
        DeclareLaunchArgument("robot_config_path", default_value=robot_config),
        DeclareLaunchArgument("device", default_value="cpu"),
        DeclareLaunchArgument("joy_device", default_value="/dev/input/js0"),
        DeclareLaunchArgument("use_motion_source", default_value="false"),
        DeclareLaunchArgument("enable_runtime_log", default_value="false"),
        DeclareLaunchArgument("log_dir", default_value="/tmp"),
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[{"device": LaunchConfiguration("joy_device")}],
            remappings=[("joy", "/sbus_data")],
        ),
        Node(
            package="tienkung_motion_source",
            executable="motion_source_node",
            name="tienkung_motion_source",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_motion_source")),
            parameters=[
                motion_config,
                {
                    "motion_file": LaunchConfiguration("motion_file"),
                    "robot_config_path": LaunchConfiguration("robot_config_path"),
                },
            ],
        ),
        Node(
            package="tienkung_policy_runner",
            executable="policy_runner_node",
            name="tienkung_policy_runner",
            output="screen",
            parameters=[
                runner_config,
                {
                    "policy_path": LaunchConfiguration("policy_path"),
                    "manifest_path": LaunchConfiguration("manifest_path"),
                    "robot_config_path": LaunchConfiguration("robot_config_path"),
                    "device": LaunchConfiguration("device"),
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