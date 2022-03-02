import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, \
                           SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    mir_description_dir = get_package_share_directory('mir_description')
    mir_gazebo_dir = get_package_share_directory('mir_gazebo')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    rviz_config_file = LaunchConfiguration('rviz_config_file')

    ld = LaunchDescription()

    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace to push all topics into.')

    declare_robot_x_arg = DeclareLaunchArgument(
        'robot_x',
        default_value='0.0',
        description='Spawning position of robot (x)')

    declare_robot_y_arg = DeclareLaunchArgument(
        'robot_y',
        default_value='0.0',
        description='Spawning position of robot (y)')

    declare_robot_yaw_arg = DeclareLaunchArgument(
        'robot_yaw',
        default_value='0.0',
        description='Spawning position of robot (yaw)')

    declare_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='Choose simulation world. Available worlds: empty, maze')

    declare_verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Set to true to enable verbose mode for Gazebo.')

    declare_teleop_arg = DeclareLaunchArgument(
        'teleop_enabled',
        default_value='true',
        description='Set to true to enable teleop to manually move MiR around.')

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz_enabled',
        default_value='true',
        description='Set to true to launch rviz.')

    declare_rviz_config_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            mir_description_dir, 'rviz', 'mir_visu_full.rviz'),
        description='Define rviz config file to be used.')

    declare_gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run headless.')

    launch_gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'verbose': LaunchConfiguration('verbose'),
            'gui': LaunchConfiguration('gui'),
            'world': [mir_gazebo_dir, '/worlds/', LaunchConfiguration('world'), '.world']
        }.items()
    )

    launch_mir_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_description_dir, 'launch', 'mir_launch.py')
        )
    )

    launch_mir_gazebo_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_gazebo_dir, 'launch',
                         'include', 'mir_gazebo_common.py')
        )
    )

    def process_namespace(context):
        robot_name = "mir_robot"
        try:
            namespace = context.launch_configurations['namespace']
            robot_name = namespace + '/' + robot_name
        except KeyError:
            pass
        return [SetLaunchConfiguration('robot_name', robot_name)]

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', LaunchConfiguration('robot_name'),
                   '-topic', 'robot_description',
                   '-b'],  # bond node to gazebo model,
        namespace=LaunchConfiguration('namespace'),
        output='screen')

    launch_rviz = Node(
        condition=IfCondition(LaunchConfiguration('rviz_enabled')),
        package='rviz2',
        executable='rviz2',
        output={'both': 'log'},
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    launch_teleop = Node(
        condition=IfCondition(LaunchConfiguration("teleop_enabled")),
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        prefix='xterm -e')

    ld.add_action(OpaqueFunction(function=process_namespace))
    ld.add_action(declare_namespace_arg)
    ld.add_action(declare_robot_x_arg)
    ld.add_action(declare_robot_y_arg)
    ld.add_action(declare_robot_yaw_arg)
    ld.add_action(declare_sim_time_arg)
    ld.add_action(declare_world_arg)
    ld.add_action(declare_verbose_arg)
    ld.add_action(declare_teleop_arg)
    ld.add_action(declare_rviz_arg)
    ld.add_action(declare_rviz_config_arg)
    ld.add_action(declare_gui_arg)

    ld.add_action(launch_gazebo_world)
    ld.add_action(launch_mir_description)
    ld.add_action(launch_mir_gazebo_common)
    ld.add_action(spawn_robot)
    ld.add_action(launch_rviz)
    ld.add_action(launch_teleop)

    return ld
