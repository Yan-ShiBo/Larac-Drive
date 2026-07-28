from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [
            FindPackageShare("cbf_rover_runtime"),
            "config",
            "external_command_relay.yaml",
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_external_command_relay",
                default_value="false",
                description=(
                    "Start the integration-only command relay; this is not a "
                    "certificate runtime or safety guard."
                ),
            ),
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="cbf_rover_runtime",
                executable="external_command_relay",
                name="external_command_relay",
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("start_external_command_relay")
                ),
                parameters=[
                    LaunchConfiguration("config"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )
