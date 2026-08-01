from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("controller_path"),
            DeclareLaunchArgument("certificate_path", default_value=""),
            Node(
                package="cbf_rover_runtime",
                executable="polynomial_tracker",
                name="polynomial_tracker",
                output="screen",
                parameters=[
                    {
                        "controller_path": LaunchConfiguration(
                            "controller_path"
                        ),
                        "certificate_path": LaunchConfiguration(
                            "certificate_path"
                        ),
                    }
                ],
            ),
        ]
    )
