from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace to push all topics into.'),

        # TODO: check add namespace remapping
        Node(
            package='ira_laser_tools',
            name='mir_laser_scan_merger',
            namespace=LaunchConfiguration('namespace'),
            executable='laserscan_multi_merger',
            parameters=[
                {'laserscan_topics': "b_scan f_scan",
                'destination_frame': "virtual_laser_link",
                'scan_destination_topic': 'scan',
                'cloud_destination_topic': 'scan_cloud',
                'min_height': -0.25,
                'max_completion_time': 0.05,
                'max_merge_time_diff': 0.005,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'best_effort': False}],
            output='screen')
    ])
