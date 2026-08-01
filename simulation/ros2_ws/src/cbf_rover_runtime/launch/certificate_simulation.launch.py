"""One auditable Gazebo + ScenarioSpec + q-SBC monitor-only run."""

from __future__ import annotations

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from cbf_rover.application.scenario_spec import load_scenario_spec


def _resolved_runtime_nodes(context: LaunchContext) -> list[Node]:
    scenario_path = LaunchConfiguration("scenario_path").perform(context)
    controller_path = LaunchConfiguration("controller_path").perform(context)
    certificate_path = LaunchConfiguration("certificate_path").perform(context)
    resolved = load_scenario_spec(scenario_path)
    return [
        Node(
            package="cbf_rover_sim",
            executable="obstacle_spawner",
            name="obstacle_spawner",
            output="screen",
            parameters=[
                {
                    "obstacles": list(
                        resolved.gazebo_obstacle_parameters
                    ),
                    "use_sim_time": True,
                }
            ],
        ),
        Node(
            package="cbf_rover_runtime",
            executable="scenario_visualizer",
            name="scenario_visualizer",
            output="screen",
            parameters=[
                {
                    "scenario_path": scenario_path,
                    "use_sim_time": True,
                }
            ],
        ),
        Node(
            package="cbf_rover_runtime",
            executable="polynomial_tracker",
            name="polynomial_tracker",
            output="screen",
            parameters=[
                {
                    "controller_path": controller_path,
                    "certificate_path": certificate_path,
                    "use_sim_time": True,
                }
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("cbf_rover_sim"),
                    "launch",
                    "simulation.launch.py",
                ]
            )
        ),
        launch_arguments={
            "gui": LaunchConfiguration("gazebo_gui"),
            "use_sim_time": "true",
            "x": "0.0",
            "y": "0.0",
            "z": "0.0",
            "yaw": "0.0",
        }.items(),
    )
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("cbf_rover_runtime"),
            "config",
            "certificate_monitor.rviz",
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario_path"),
            DeclareLaunchArgument("controller_path"),
            DeclareLaunchArgument("certificate_path"),
            DeclareLaunchArgument("gazebo_gui", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            simulation_launch,
            OpaqueFunction(function=_resolved_runtime_nodes),
            Node(
                package="rviz2",
                executable="rviz2",
                name="certificate_rviz",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
