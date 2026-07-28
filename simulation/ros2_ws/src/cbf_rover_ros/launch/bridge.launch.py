from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_bridge",
                default_value="false",
                description="Create the optional lifecycle bridge node.",
            ),
            DeclareLaunchArgument("artifact_root", default_value=""),
            DeclareLaunchArgument("artifact_manifest", default_value=""),
            DeclareLaunchArgument("method_config", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            LifecycleNode(
                package="cbf_rover_ros",
                executable="lifecycle_bridge",
                name="cbf_rover_bridge",
                namespace="",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_bridge")),
                parameters=[
                    {
                        "artifact_root": LaunchConfiguration("artifact_root"),
                        "artifact_manifest": LaunchConfiguration(
                            "artifact_manifest"
                        ),
                        "method_config": LaunchConfiguration("method_config"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
            ),
        ]
    )
