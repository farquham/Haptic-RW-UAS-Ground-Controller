import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('system_launcher'),
      'config',
      'visuals.yaml'
      )

   return LaunchDescription([
      Node(
         package='visualization_inputs',
         executable='vis',
         namespace='visuals',
         name='sim',
         parameters=[]
      )
   ])