import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('system_launcher'),
      'config',
      'bmnrim.yaml'
      )

   return LaunchDescription([
      Node(
         package='b_rim',
         executable='brim_node',
         namespace='b_rim',
         name='sim',
         parameters=[config]
      )
   ])