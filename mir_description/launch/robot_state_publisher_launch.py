import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    mir_description_dir = get_package_share_directory('mir_description')

    def create_robot_description(context):
      tf_prefix = context.launch_configurations['tf_prefix']
      urdf_dir = os.path.join(mir_description_dir, 'urdf')
      xacro_file = os.path.join(urdf_dir, 'mir.urdf.xacro')
      doc = xacro.process_file(xacro_file, mappings={'tf_prefix' : tf_prefix})
      robot_desc = doc.toprettyxml(indent='  ')
      return [SetLaunchConfiguration('robot_description', robot_desc)]

    return LaunchDescription([

      DeclareLaunchArgument(
        'tf_prefix',
        default_value='',
        description='Robot tf prefix'),

      DeclareLaunchArgument(
        'joint_state_publisher_enabled',
        default_value='false',
        description='Enable to publish joint states using joint state publisher'),

      OpaqueFunction(function=create_robot_description),

      Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': use_sim_time, 
                      'robot_description': LaunchConfiguration('robot_description')}]),

      Node(
        condition=IfCondition(LaunchConfiguration('joint_state_publisher_enabled')),
        package='joint_state_publisher',
        executable='joint_state_publisher')

    ])
