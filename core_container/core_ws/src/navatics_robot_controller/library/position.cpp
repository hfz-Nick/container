#include "navatics_robot_controller/position.hpp"

namespace navatics_robot_controller{

Position::Position(Eigen::Vector3f p_gain, Eigen::Vector3f d_gain, Eigen::Vector3f max_cmd_force, 
                   bool compensate_bias, float weight, float buoyancy){
  this->p_gain = p_gain;
  this->d_gain = d_gain;
  this->compensate_bias = compensate_bias;
  this->bias = weight - buoyancy;
  this->max_cmd_force = max_cmd_force;
} // constructor

Eigen::Vector3f Position::get_bias(Eigen::Matrix3f rotation_matrix){
  if (this->compensate_bias)
    return (rotation_matrix.col(2)*(this->bias));
  return Eigen::Vector3f(0,0,0);
} // get_bias

Eigen::Vector3f Position::get_position_error(Eigen::Vector3f position, Eigen::Vector3f ref_position,
                                             Eigen::Matrix3f rotation_matrix)
{
  return (rotation_matrix*(ref_position - position));
} // get_position_error


Eigen::Vector3f Position::get_velocity_error(Eigen::Vector3f velocity, Eigen::Vector3f ref_velocity,
                                             Eigen::Matrix3f rotation_matrix)
{
  return (rotation_matrix*(ref_velocity - velocity));
} // get_velocity_error

Eigen::Vector3f Position::calculate_control_input(Eigen::Vector3f position,
                                                  Eigen::Vector3f ref_position,
                                                  Eigen::Vector3f velocity,
                                                  Eigen::Vector3f ref_velocity,
                                                  Eigen::Vector3f commanded_input,
                                                  Eigen::Quaternionf orientation,
                                                  Eigen::Quaternionf ref_orientation,
                                                  Eigen::Quaternionf bias)
{
  Eigen::Matrix3f rotation_matrix = orientation.toRotationMatrix().transpose();
  Eigen::Matrix3f ref_matrix = ref_orientation.toRotationMatrix().transpose();
  Eigen::Matrix3f rotation_bias = bias.toRotationMatrix().transpose();
  Eigen::Vector3f Tp = this->p_gain.array() * 
                       (this->get_position_error(position, ref_position, rotation_matrix)).array();
  Eigen::Vector3f Td = this->d_gain.array() *
                       (this->get_velocity_error(velocity, ref_velocity, rotation_matrix)).array();
  Eigen::Vector3f cmdf = commanded_input.array() * this->max_cmd_force.array();
  cmdf = (rotation_matrix*ref_matrix.transpose()*cmdf);
  return (cmdf - this->get_bias(rotation_matrix) + Tp + Td);
} // calculate_control_input


} // namespace navatics_robot_controller
