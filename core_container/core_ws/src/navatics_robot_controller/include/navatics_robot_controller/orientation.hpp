#ifndef NAVATICSROBOTCONTROLLER_ORIENTATION_H
#define NAVATICSROBOTCONTROLLER_ORIENTATION_H

#include <Eigen/Eigen>

#include <iostream>

namespace navatics_robot_controller{

#define FLAT 0
#define ROLL_RIGHT 1
#define ROLL_LEFT 2
#define TILT_UP 3
#define TILT_DOWN 4
#define UPSIDE_DOWN 5

class Orientation{
  /*
  This class is designed to:
  1. Handle calculation for orientation control input
  */

  public:
  Orientation(Eigen::Vector3f p_gain, Eigen::Vector3f d_gain, bool compensate_bias,
              Eigen::Vector3f center_of_gravity, Eigen::Vector3f center_of_buoyancy, 
              float buoyancy = 0.0);

  // calculate control input given position (and velocity), reference, comanded input, and orientation
  Eigen::Vector3f calculate_control_input(Eigen::Quaternionf orientation, 
                                          Eigen::Quaternionf ref_orientation,
                                          Eigen::Vector3f angular_velocity,
                                          Eigen::Vector3f ref_angular_velocity,
                                          int orientation_flag = FLAT);
  
  // get ref orientation
  Eigen::Quaternionf get_adjusted_orientation(Eigen::Quaternionf quaternion, int orientation_flag=FLAT);
  Eigen::Quaternionf get_orientation_bias(int orientation_flag = FLAT);

  private:
  // helper functions
  Eigen::Vector3f get_bias(Eigen::Matrix3f rotation_matrix);
  Eigen::Vector3f get_orientation_error(Eigen::Quaternionf orientation, 
                                        Eigen::Quaternionf ref_orientation,
                                        int orientation_flag = FLAT);
  Eigen::Vector3f get_angular_velocity_error(Eigen::Vector3f angular_velocity, 
                                             Eigen::Vector3f ref_angular_velocity,
                                             int orientation_flag);
  
  // calculation parameters
  Eigen::Vector3f p_gain;
  Eigen::Vector3f d_gain;
  bool compensate_bias = true;
  float buoyancy = 0;
  Eigen::Vector3f buoyancy_vector;

}; // class Orientation

} // namespace navatics_robot_controller

#endif // NAVATICSROBOTCONTROLLER_ORIENTATION_H
