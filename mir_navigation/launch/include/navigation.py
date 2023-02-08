# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction, \
                           SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    mir_nav_dir = get_package_share_directory('mir_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    command_topic = LaunchConfiguration('cmd_vel_w_prefix')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_nav_to_pose_bt_xml = LaunchConfiguration(
        'default_nav_to_pose_bt_xml')
    map_subscribe_transient_local = LaunchConfiguration(
        'map_subscribe_transient_local')

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'recoveries_server',
                       'bt_navigator',
                       'waypoint_follower']

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('cmd_vel', command_topic)]

    # Create our own temporary YAML files that include substitutions
    # Watch out for parameters that don't exist in yaml - will not be substituted of course
    # (default_nav_to_pose_bt_xml)
    # TODO: Needs to be addressed in nav2
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local}

    configured_params = RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True)

    def add_prefix_to_cmd_vel(context):
        topic = context.launch_configurations['cmd_vel_topic']
        try:
            namespace = context.launch_configurations['namespace']
            topic = namespace + '/' + topic
        except KeyError:
            pass
        return [SetLaunchConfiguration('cmd_vel_w_prefix', topic)]

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='cmd_vel',
            description='Define cmd_vel topic'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                mir_nav_dir, 'config', 'mir_nav_params.yaml'),
            description='Full path to the ROS2 parameters file to use'),

        DeclareLaunchArgument(
            'default_nav_to_pose_bt_xml',
            default_value=os.path.join(
                mir_nav_dir, 'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml'),
            description='Full path to the behavior tree xml file to use'),

        DeclareLaunchArgument(
            'map_subscribe_transient_local', default_value='true',
            description='Whether to set the map subscriber QoS to transient local'),

        OpaqueFunction(function=add_prefix_to_cmd_vel),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings
        ),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                configured_params,
                {'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml}],
            remappings=remappings
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params],
            remappings=remappings
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])
    ])
