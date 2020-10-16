from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import launch.actions

def generate_launch_description():
  return LaunchDescription( [launch.actions.DeclareLaunchArgument(
                               'position_estimator_params',
                               default_value='position_estimator_params.yaml',
                               description='position estimator params'
                             ),
                             launch_ros.actions.Node(package='navatics_position_estimator',
                                                     node_executable='position_estimator_node',
                                                     output='screen', emulate_tty='true',
                                                     parameters=
                                                     [LaunchConfiguration('position_estimator_params')]
                                                    ),
                            ] )
# def generate_launch_description

