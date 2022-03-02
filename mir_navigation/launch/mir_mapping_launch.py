import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, \
    SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    mir_driver_dir = get_package_share_directory('mir_driver')
    mir_nav_dir = get_package_share_directory('mir_navigation')

    def declare_rviz_config(context):
        nav_enabled = context.launch_configurations['navigation_enabled']
        if (nav_enabled == 'true'):
            config_file = os.path.join(
                mir_nav_dir, 'rviz', 'mir_mapping_nav.rviz')
        else:
            config_file = os.path.join(mir_nav_dir, 'rviz', 'mir_mapping.rviz')
        return [SetLaunchConfiguration('rviz_config_file', config_file)]

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("mir_navigation"),
                                   'config', 'mir_mapping_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    declare_nav_argument = DeclareLaunchArgument(
        'navigation_enabled',
        default_value='false',
        description='Use navigation for mapping')

    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace to push all topics into.')

    start_driver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_driver_dir, 'launch', 'mir_launch.py')),
        launch_arguments={'rviz_config_file': LaunchConfiguration(
            'rviz_config_file')}.items()
    )

    launch_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_nav_dir, 'launch', 'include', 'mapping.py')),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('slam_params_file', LaunchConfiguration('slam_params_file'))]
    )

    launch_navigation_if_enabled = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_nav_dir, 'launch', 'include', 'navigation.py')),
        condition=IfCondition(LaunchConfiguration('navigation_enabled')),
        launch_arguments={'map_subscribe_transient_local': 'true'}.items()
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_arg)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_nav_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(OpaqueFunction(function=declare_rviz_config))

    ld.add_action(start_driver_cmd)
    ld.add_action(launch_mapping)
    ld.add_action(launch_navigation_if_enabled)

    return ld
