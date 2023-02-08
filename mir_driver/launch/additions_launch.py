import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    mir_description_dir = get_package_share_directory('mir_description')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    return LaunchDescription([

        DeclareLaunchArgument(
            name='namespace',
            default_value=''
        ),

        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(mir_description_dir, 'rviz', 'mir_visu_full.rviz'),
            description='Define rviz config to be used'
        ),

        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            namespace=namespace,
            prefix='xterm -e'),

        Node(
            package='rviz2',
            executable='rviz2',
            output={'both': 'log'},
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', rviz_config_file]
        ),

    ])
