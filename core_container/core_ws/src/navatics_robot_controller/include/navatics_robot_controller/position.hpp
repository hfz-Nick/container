#ifndef NAVATICSROBOTCONTROLLER_POSITION_H
#define NAVATICSROBOTCONTROLLER_POSITION_H

#include <Eigen/Eigen>
#include <iostream>

namespace navatics_robot_controller{

class Position{
  /*
  This class is designed to:
  1. Handle the calculation for position control input
  */

  public:
  Position(Eigen::Vector3f p_gain, Eigen::Vector3f d_gain, Eigen::Vector3f max_cmd_force, 
           bool compensate_bias, float weight = 0.0, float buoyancy = 0.0);

  // calculate control input given position (and velocity), reference, comanded input, and orientation
  Eigen::Vector3f calculate_control_input(Eigen::Vector3f position, Eigen::Vector3f ref_position,
                                          Eigen::Vector3f velocity, Eigen::Vector3f ref_velocity,
                                          Eigen::Vector3f commanded_input,
                                          Eigen::Quaternionf orientation, 
                                          Eigen::Quaternionf ref_orientation,
                                          Eigen::Quaternionf bias); 

  private:
  // helper functions
  Eigen::Vector3f get_bias(Eigen::Matrix3f rotation_matrix);
  Eigen::Vector3f get_position_error(Eigen::Vector3f position, Eigen::Vector3f ref_position, 
                                     Eigen::Matrix3f rotation_matrix);
  Eigen::Vector3f get_velocity_error(Eigen::Vector3f velocity, Eigen::Vector3f ref_velocity, 
                                     Eigen::Matrix3f rotation_matrix);
  
  // calculation parameters
  Eigen::Vector3f p_gain;
  Eigen::Vector3f d_gain;
  Eigen::Vector3f max_cmd_force;
  bool compensate_bias = true;
  float bias = 0;

}; // class Position

} // namespace navatics_robot_controller

#endif // NAVATICSROBOTCONTROLLER_POSITION_H
