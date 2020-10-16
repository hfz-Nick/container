from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import launch.actions

def generate_launch_description():
  # include attitude estimator
  attitude_est_dir = get_package_share_directory('navatics_attitude_estimator')
  attitude_est = launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                [attitude_est_dir, '/launch/navatics_robot_system.launch.py']),
                launch_arguments=
                {'attitude_estimator_params': '/home/ros/params/attitude_estimator_params.yaml'}.items()
              )

  # include position estimator
  position_est_dir = get_package_share_directory('navatics_position_estimator')
  position_est = launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                [position_est_dir, '/launch/navatics_robot_system.launch.py']),
                launch_arguments=
                {'position_estimator_params': '/home/ros/params/position_estimator_params.yaml'}.items()
              )

  # include controller
  controller_dir = get_package_share_directory('navatics_robot_controller')
  controller = launch.actions.IncludeLaunchDescription(
                  launch.launch_description_sources.PythonLaunchDescriptionSource(
                  [controller_dir, '/launch/navatics_robot_system.launch.py']),
                  launch_arguments={'controller_params': '/home/ros/params/controller_params.yaml',
                    'thrusters_params': '/home/ros/params/thrusters_params.yaml'}.items()
                )

  # include can_interface
  can_interface_dir = get_package_share_directory('navatics_can_interface')
  can_interface = launch.actions.IncludeLaunchDescription(
                    launch.launch_description_sources.PythonLaunchDescriptionSource(
                    [can_interface_dir, '/launch/navatics_robot_system.launch.py']),
                    launch_arguments=
                     {'can_interface_params': '/home/ros/params/can_interface_params.yaml'}.items()
                  )
  
  return LaunchDescription( [controller, can_interface, attitude_est, position_est] )

# def generate_launch_description
