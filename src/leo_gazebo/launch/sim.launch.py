from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    world_file = PathJoinSubstitution([
        FindPackageShare('leo_gazebo'),
        'worlds',
        'my_world.world'
    ])

    xacro_file = PathJoinSubstitution([
        FindPackageShare('leo_description'),
        'urdf',
        'leo.urdf.xacro'
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_file]
        }.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file])
        }],
        output='screen'
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_spawn_model.launch.py'
            ])
        ]),
        launch_arguments={
            'world': 'game_style_map',
            'topic': 'robot_description',
            'entity_name': 'leo',
            'x': '0.0',
            'y': '0.0',
            'z': '0.2'
        }.items()
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_robot
    ])