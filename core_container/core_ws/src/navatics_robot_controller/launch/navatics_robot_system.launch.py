from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import launch.actions

def generate_launch_description():
  return LaunchDescription( [launch.actions.DeclareLaunchArgument(
                               'controller_params', default_value='control_params.yaml',
                               description='controller parameters'
                             ),
                             launch.actions.DeclareLaunchArgument(
                               'thrusters_params', default_value='thrusters_params.yaml',
                               description='thrusters parameters'
                             ),
                             launch_ros.actions.Node(package='navatics_robot_controller',
                                                     node_executable='controller_node',
                                                     output='screen', emulate_tty='true',
                                                     parameters=
                                                     [LaunchConfiguration('controller_params'),
                                                      LaunchConfiguration('thrusters_params')]
                                                    ),
                            ] )
# def generate_launch_description

