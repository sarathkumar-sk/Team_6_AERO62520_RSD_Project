from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    map_file = "/home/student41/robot_ws_clean/maps/fixed_map_v1.yaml"

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("leo_gazebo"),
                "launch",
                "sim.launch.py"
            ])
        ])
    )

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("leo_gazebo"),
                "launch",
                "bridge.launch.py"
            ])
        ])
    )

    lidar_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="rplidar_static_tf",
        arguments=[
            "--x", "0.14",
            "--y", "0.0",
            "--z", "0.02",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "leo/base_link/rplidar_a2_m12"
        ],
        output="screen"
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "yaml_filename": map_file
            }
        ]
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("leo_navigation"),
                "config",
                "amcl_params_sim.yaml"
            ])
        ]
    )

    lifecycle_manager_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["map_server", "amcl"]
            }
        ]
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("leo_navigation"),
                "launch",
                "nav2_slam.launch.py"
            ])
        ])
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        prefix="env LIBGL_ALWAYS_SOFTWARE=1 QT_QPA_PLATFORM=xcb",
        parameters=[
            {
                "use_sim_time": True
            }
        ],
        output="screen"
    )

    return LaunchDescription([
        gazebo_sim,

        TimerAction(
            period=3.0,
            actions=[bridge]
        ),

        TimerAction(
            period=4.0,
            actions=[lidar_static_tf]
        ),

        TimerAction(
            period=5.0,
            actions=[map_server, amcl, lifecycle_manager_localization]
        ),

        TimerAction(
            period=8.0,
            actions=[nav2]
        ),

        TimerAction(
            period=10.0,
            actions=[rviz2]
        ),
    ])
