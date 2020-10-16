from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import launch.actions

def generate_launch_description():

  can_params_arg = launch.actions.DeclareLaunchArgument( 'can_interface_params', 
                                                          default_value='can_interface_params.yaml',
                                                          description='can interface parameters')
 

  can = launch_ros.actions.Node(package='navatics_can_interface',
                                node_executable='can_interface_node',
                                output='screen', emulate_tty='true',
                                parameters=
                                [LaunchConfiguration('can_interface_params')],
                                on_exit=launch.actions.Shutdown())
  
  return LaunchDescription( [can_params_arg, can] )
# def generate_launch_description

