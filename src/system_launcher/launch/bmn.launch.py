import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('system_launcher'),
      'config',
      'bmn.yaml'
      )

   return LaunchDescription([
      Node(
         package='bmn',
         executable='bmn_node',
         namespace='BMN',
         name='sim',
         parameters=[config]
      )
   ])