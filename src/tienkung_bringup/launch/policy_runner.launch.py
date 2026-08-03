from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("tienkung_bringup"))
    robot_config = str(share / "config" / "tg22_config.yaml")
    node_config = str(share / "config" / "policy_runner.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("policy_path", default_value=""),
        DeclareLaunchArgument("manifest_path", default_value=""),
        DeclareLaunchArgument("robot_config_path", default_value=robot_config),
        DeclareLaunchArgument("device", default_value="cpu"),
        DeclareLaunchArgument("enable_runtime_log", default_value="false"),
        DeclareLaunchArgument("log_dir", default_value="/tmp"),
        Node(
            package="tienkung_policy_runner",
            executable="policy_runner_node",
            name="tienkung_policy_runner",
            output="screen",
            parameters=[
                node_config,
                {
                    "policy_path": LaunchConfiguration("policy_path"),
                    "manifest_path": LaunchConfiguration("manifest_path"),
                    "robot_config_path": LaunchConfiguration("robot_config_path"),
                    "device": LaunchConfiguration("device"),
                    "enable_runtime_log": ParameterValue(
                        LaunchConfiguration("enable_runtime_log"), value_type=bool
                    ),
                    "log_dir": LaunchConfiguration("log_dir"),
                },
            ],
        ),
    ])