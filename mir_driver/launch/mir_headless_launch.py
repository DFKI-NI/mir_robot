import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    mir_description_dir = get_package_share_directory('mir_description')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace to push all topics into.'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description=''),

        DeclareLaunchArgument(
            'mir_hostname',
            default_value='192.168.12.20',
            description=''),

        # DeclareLaunchArgument(
        #     'disable_map',
        #     default_value='false',
        #     description='Disable the map topic and map -> odom_comb TF transform from the MiR'),

        DeclareLaunchArgument(
            'robot_state_publisher_enabled',
            default_value='true',
            description='Set to true to publish tf using mir_description'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mir_description_dir, 'launch', 'mir_launch.py')),
            launch_arguments={
                'joint_state_publisher_enabled': 'false',
                'namespace': LaunchConfiguration('namespace')
            }.items(),
            condition=IfCondition(LaunchConfiguration(
                'robot_state_publisher_enabled'))
        ),

        Node(
            package='mir_driver',
            executable='mir_bridge',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                         'tf_prefix': LaunchConfiguration('namespace')}],
            namespace=LaunchConfiguration('namespace'),
            output='screen'),

        Node(
            package='mir_driver',
            executable='fake_mir_joint_publisher',
            remappings=[('use_sim_time', LaunchConfiguration('use_sim_time'))],
            parameters=[{'tf_prefix': LaunchConfiguration('namespace')}],
            namespace=LaunchConfiguration('namespace'),
            output='screen'),

        Node(
            package='twist_stamper',
            executable='twist_stamper',
            name='twist_stamper_cmd_vel_mir',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                ('cmd_vel_in', 'cmd_vel'),
                ('cmd_vel_out', 'cmd_vel_stamped'),
            ],
            namespace=LaunchConfiguration('namespace'),
        ),

        Node(
            package='ira_laser_tools',
            name='mir_laser_scan_merger',
            executable='laserscan_multi_merger',
            parameters=[{'laserscan_topics': "b_scan f_scan",
                         'destination_frame': "virtual_laser_link",
                         'scan_destination_topic': 'scan',
                         'cloud_destination_topic': 'scan_cloud',
                         'min_height': -0.25,
                         'max_merge_time_diff': 0.05,
                         # driver (msg converter) delay
                         'max_delay_scan_time': 2.5,
                         'max_completion_time': 0.1,
                         'alow_scan_delay': True,
                         'use_sim_time': use_sim_time,
                         'best_effort': False}],
            namespace=LaunchConfiguration('namespace'),
            output='screen')

    ])
