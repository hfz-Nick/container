from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import launch.actions

def generate_launch_description():
  return LaunchDescription(  [launch_ros.actions.Node(package='navatics_server',
                                                     node_executable='endpoints_node',
                                                     output='screen', emulate_tty='true',
                                                     parameters=
                                                     ['/home/ros/params/server_params.yaml']
                                                    ),
                            ] )
# def generate_launch_description

