from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    spawn_demo_obstacles = LaunchConfiguration("spawn_demo_obstacles")

    simulation_xacro = PathJoinSubstitution(
        [FindPackageShare("cbf_rover_gazebo"), "urdf", "cbf_rover_sim.urdf.xacro"]
    )
    robot_description = {
        "robot_description": Command(["xacro ", simulation_xacro])
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
            )
        ),
        launch_arguments={"world": world, "gui": gui}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("cbf_rover_gazebo"),
                        "worlds",
                        "static_obstacles.world",
                    ]
                ),
            ),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("spawn_demo_obstacles", default_value="true"),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.12"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            gazebo,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[robot_description, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                output="screen",
                arguments=[
                    "-entity",
                    "cbf_rover",
                    "-topic",
                    "robot_description",
                    "-x",
                    LaunchConfiguration("x"),
                    "-y",
                    LaunchConfiguration("y"),
                    "-z",
                    LaunchConfiguration("z"),
                    "-Y",
                    LaunchConfiguration("yaw"),
                ],
            ),
            Node(
                package="cbf_rover_gazebo",
                executable="obstacle_spawner",
                output="screen",
                condition=IfCondition(spawn_demo_obstacles),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "obstacles": [
                            "box_1,1.5,0.0,0.35,0.35,0.80",
                            "box_2,2.5,0.8,0.45,0.45,1.00",
                            "box_3,2.5,-0.8,0.45,0.45,1.00",
                        ],
                    }
                ],
            ),
        ]
    )
