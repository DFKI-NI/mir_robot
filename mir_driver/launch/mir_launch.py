import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    mir_driver = get_package_share_directory('mir_driver')

    return LaunchDescription([

      IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
          os.path.join(mir_driver, 'launch', 'mir_headless_launch.py')),
      ),

      IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
          os.path.join(mir_driver, 'launch', 'additions_launch.py')),
      )

    ])
