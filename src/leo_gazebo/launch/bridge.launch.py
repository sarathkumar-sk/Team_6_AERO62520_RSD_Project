from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('leo_gazebo'),
        'config',
        'bridge.yaml'
    ])

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_bridge'),
                'launch',
                'ros_gz_bridge.launch.py'
            ])
        ]),
        launch_arguments={
            'bridge_name': 'leo_bridge',
            'config_file': config_file
        }.items()
    )

    return LaunchDescription([
        bridge
    ])
