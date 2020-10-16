import subprocess
import sys
import os
import yaml
from envbash import load_envbash

def build_test(package, ws_dir = '/home/ros/core_ws'):
  os.chdir(ws_dir)
  ret_val = 0
  build_cmd = 'colcon build --packages-select %s'%package
  ret_val = 0
  try:
    subprocess.check_output(build_cmd, shell=True)
  except subprocess.CalledProcessError as exc:
    ret_val = exc.returncode
  return ret_val
# def build_test

def predictor_unit_test(pkg, exe, distro='eloquent', install_dir = '/home/ros/core_ws/install'):
  ret_val = 0
  # set colcon prefix path
  load_envbash(install_dir + '/setup.bash', override=True)
  # set ros_run_cmd
  ros_run_cmd = '/opt/ros/%s/bin/ros2 run %s %s'
  distro = distro
  # run
  try:
    subprocess.check_output(ros_run_cmd%(distro, pkg, exe) , shell=True)
  except subprocess.CalledProcessError as exc:
    ret_val = exc.returncode
  return ret_val

if __name__ == "__main__":
  # get arguments
  arglen = len(sys.argv)
  assert arglen == 2, 'Missing workspace directory, aborting'
  ws_dir = sys.argv[1]
  
  # set pkg
  pkg = 'navatics_attitude_estimator'
  unit_test_node = 'predictor_unit_test'  

  # build test
  print("Testing for %s package in workspace %s"%(pkg, ws_dir))
  build_exit_code = build_test(pkg, ws_dir=ws_dir)
  assert (build_exit_code == 0), "Build failed with exit code %d"%build_exit_code
  print("Build completed")
  
  # unit test
  print("Running unit test")
  install_dir = ws_dir + '/install'
  unit_test_code = predictor_unit_test(pkg, unit_test_node, install_dir = install_dir)
  assert unit_test_code == 0, "Unit test failed with exit code %d"%unit_test_code
  print("Unit test completed")
  
# main
