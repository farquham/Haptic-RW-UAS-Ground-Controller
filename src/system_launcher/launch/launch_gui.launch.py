import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
   visualization_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('system_launcher'), 'launch/'),
         '/visuals.launch.py'])
      )
   logging_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('system_launcher'), 'launch/'),
         '/logging.launch.py'])
      )
   bubble_method_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('system_launcher'), 'launch/'),
         'bmn.launch.py'])
      )
   bmn_rim_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('system_launcher'), 'launch/'),
         '/bmnrim.launch.py'])
      )
   rb_sim_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('system_launcher'), 'launch/'),
         '/rbsim.launch.py'])
      )
   rpi_rim_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('system_launcher'), 'launch/'),
         '/rpirim.launch.py'])
      )
   rpi_comms_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('system_launcher'), 'launch/'),
         '/rpicomms.launch.py'])
      )

   return LaunchDescription([
      visualization_node,
      logging_node,
      bubble_method_node,
      bmn_rim_node,
      rb_sim_node,
      rpi_rim_node,
      rpi_comms_node
   ])
