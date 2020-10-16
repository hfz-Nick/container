from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import launch.actions

def generate_launch_description():
  return LaunchDescription( [launch.actions.DeclareLaunchArgument(
                               'attitude_estimator_params',
                               default_value='attitude_estimator_params.yaml',
                               description='attitude estimator parameters'
                             ),
                             launch_ros.actions.Node(package='navatics_attitude_estimator',
                                                     node_executable='attitude_estimator_node',
                                                     output='screen', emulate_tty='true',
                                                     parameters=
                                                     [LaunchConfiguration('attitude_estimator_params')]
                                                    ),
                            ] )
# def generate_launch_description

