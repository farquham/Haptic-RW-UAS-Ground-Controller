import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('system_launcher'),
      'config',
      'rbsim.yaml'
      )

   return LaunchDescription([
      Node(
         package='rb_quad_sim',
         executable='rb_quad_sim_node',
         namespace='rbsim',
         name='sim',
         parameters=[config]
      )
   ])