from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share_dir = Path(get_package_share_directory("hybrid_fleet_manager"))
    default_params = str(share_dir / "config" / "fleet_manager_params.yaml")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to fleet manager parameter YAML file",
    )

    fleet_manager_node = Node(
        package="hybrid_fleet_manager",
        executable="fleet_manager",
        name="fleet_manager_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    grid_monitor_node = Node(
        package="hybride_fleet_manager",
        executable="grid_monitor",
        name="grid_monitor_node",
        output="screen",
    )

    return LaunchDescription([
        params_file_arg,
        fleet_manager_node,
        grid_monitor_node
    ])
