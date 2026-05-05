from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('leo_gazebo'),
                'launch',
                'sim.launch.py'
            ])
        ])
    )

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('leo_gazebo'),
                'launch',
                'bridge.launch.py'
            ])
        ])
    )

    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rplidar_static_tf',
        arguments=[
            '--x', '0.14',
            '--y', '0.0',
            '--z', '0.02',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'leo/base_link/rplidar_a2_m12'
        ],
        output='screen'
    )

    # Gazebo joint states -> ROS /joint_states
    # This is needed for wheel_FL_link / wheel_FR_link / wheel_RL_link / wheel_RR_link in RViz.
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_state_bridge',
        arguments=[
            '/world/game_style_map/model/leo/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '--ros-args',
            '-r',
            '/world/game_style_map/model/leo/joint_state:=/joint_states'
        ],
        output='screen'
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'scan_topic': '/scan'
        }.items()
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        prefix='env LIBGL_ALWAYS_SOFTWARE=1 QT_QPA_PLATFORM=xcb',
        parameters=[{
            'use_sim_time': True
        }],
        output='screen'
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
            period=4.5,
            actions=[joint_state_bridge]
        ),

        TimerAction(
            period=6.0,
            actions=[slam_toolbox]
        ),

        TimerAction(
            period=8.0,
            actions=[rviz2]
        ),
    ])