import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    mir_description_dir = get_package_share_directory('mir_description')
    rviz_config_file = os.path.join(
        mir_description_dir, 'rviz', 'mir_description.rviz')

    return LaunchDescription([

        DeclareLaunchArgument(
            'joint_state_publisher_enabled',
            default_value='true',
            description='Enable to publish joint states using joint state publisher'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mir_description_dir, 'launch', 'mir_launch.py')),
            launch_arguments={
                'joint_state_publisher_enabled':
                LaunchConfiguration('joint_state_publisher_enabled'),
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )

    ])
