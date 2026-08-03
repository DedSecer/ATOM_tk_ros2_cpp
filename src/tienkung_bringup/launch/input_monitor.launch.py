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
    runner_config = str(share / "config" / "policy_runner.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("robot_config_path", default_value=robot_config),
        DeclareLaunchArgument("print_hz", default_value="1.0"),
        DeclareLaunchArgument("verbose", default_value="false"),
        Node(
            package="tienkung_policy_runner",
            executable="input_monitor_node",
            name="tienkung_input_monitor",
            output="screen",
            parameters=[
                runner_config,
                {
                    "robot_config_path": LaunchConfiguration("robot_config_path"),
                    "print_hz": ParameterValue(
                        LaunchConfiguration("print_hz"), value_type=float
                    ),
                    "verbose": ParameterValue(
                        LaunchConfiguration("verbose"), value_type=bool
                    ),
                },
            ],
        ),
    ])