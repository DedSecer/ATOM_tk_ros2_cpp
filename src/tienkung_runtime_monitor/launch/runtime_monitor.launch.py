"""Launch only the read-only web monitor, never a policy/motor node."""
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _start_monitor(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    executable = Path(get_package_prefix("tienkung_runtime_monitor")) / "lib" / "tienkung_runtime_monitor" / "runtime_monitor"
    command = [str(executable), "--bind", value("bind"), "--port", value("port")]
    if value("log_file"):
        command += ["--log-file", value("log_file")]
    else:
        command += ["--log-dir", value("log_dir"), "--pattern", value("pattern")]
    if value("urdf_path"):
        command += ["--urdf", value("urdf_path")]
    if value("asset_root"):
        command += ["--asset-root", value("asset_root")]
    return [ExecuteProcess(cmd=command, output="screen", shell=False)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("log_dir", default_value="/tmp", description="Prefer a directory dedicated to one policy node"),
        DeclareLaunchArgument("log_file", default_value="", description="Pin one JSONL file instead of following the newest run"),
        DeclareLaunchArgument("pattern", default_value="policy_log_*.jsonl"),
        DeclareLaunchArgument("bind", default_value="0.0.0.0"),
        DeclareLaunchArgument("port", default_value="8080"),
        DeclareLaunchArgument("urdf_path", default_value="", description="Absolute deployed URDF path; optional"),
        DeclareLaunchArgument("asset_root", default_value="", description="Allowed local mesh root; defaults to URDF directory"),
        OpaqueFunction(function=_start_monitor),
    ])
