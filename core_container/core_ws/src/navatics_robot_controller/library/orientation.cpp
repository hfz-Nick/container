#include "navatics_robot_controller/orientation.hpp"

namespace navatics_robot_controller{

Orientation::Orientation(Eigen::Vector3f p_gain, Eigen::Vector3f d_gain, bool compensate_bias,
                         Eigen::Vector3f center_of_gravity, Eigen::Vector3f center_of_buoyancy, 
                         float buoyancy)
{
  this->p_gain = p_gain;
  this->d_gain = d_gain;
  this->compensate_bias = compensate_bias;
  this->buoyancy = buoyancy;
  this->buoyancy_vector = center_of_gravity - center_of_buoyancy;
} // constructor

Eigen::Vector3f Orientation::get_bias(Eigen::Matrix3f rotation_matrix){
  if (this->compensate_bias){
    Eigen::Vector3f G = rotation_matrix.col(2)*this->buoyancy;
    Eigen::Vector3f bias = G.cross(this->buoyancy_vector);
    return bias;
  }
  return Eigen::Vector3f (0,0,0);
} // get_bias

Eigen::Quaternionf Orientation::get_adjusted_orientation(Eigen::Quaternionf quaternion, 
                                                         int orientation_flag)
{
  Eigen::Quaternionf r = quaternion*this->get_orientation_bias(orientation_flag);
  r.normalize();
  return r;
}

Eigen::Vector3f Orientation::get_orientation_error(Eigen::Quaternionf orientation,
                                                   Eigen::Quaternionf ref_orientation,
                                                   int orientation_flag)
{
  // get difference in rotation
  Eigen::Quaternionf ref = ref_orientation*this->get_orientation_bias(orientation_flag);
  ref.normalize();
  Eigen::Quaternionf qe = orientation*ref.inverse();
  qe.normalize();
  // calculate orientation error
  Eigen::AngleAxisf aa(qe.inverse());
  float angle = atan2(sin(aa.angle()), cos(aa.angle())); // normlize angle to -180 to 180
  Eigen::Vector3f error;
  error = aa.axis().normalized();
  error = error*angle;
  // transform error to correct frame (jacobian)
  Eigen::Matrix3f V, R;
  V << 0, -error(2), error(1),
       error(2), 0 , -error(0),
       -error(1), error(0), 0;
  R = ref.toRotationMatrix().transpose(); // transposed for consistency
  V = R * V * R.transpose();
  Eigen::Vector3f transformed_error(V(2,1), V(0,2), V(1,0));
  // std::cout << "Error: " << transformed_error.transpose() << "\n";
  return transformed_error;
} // get_orientation_error

Eigen::Quaternionf Orientation::get_orientation_bias(int orientation_flag){
  Eigen::Quaternionf q(1,0,0,0);
  switch(orientation_flag){
    case ROLL_RIGHT : q = Eigen::Quaternionf(0.707, 0.707, 0, 0); break;
    case ROLL_LEFT : q = Eigen::Quaternionf(0.707, -0.707, 0, 0); break;
    case TILT_UP : q = Eigen::Quaternionf(0.707, 0, 0.707, 0); break;
    case TILT_DOWN: q = Eigen::Quaternionf(0.707, 0, -0.707, 0); break;
    case UPSIDE_DOWN: q = Eigen::Quaternionf(0, 1, 0, 0); break;
  }
  q.normalize();
  return q;
}

Eigen::Vector3f Orientation::get_angular_velocity_error(Eigen::Vector3f angular_velocity,
                                                        Eigen::Vector3f ref_angular_velocity,
                                                        int orientation_flag)
{
  Eigen::Matrix3f R = this->get_orientation_bias(orientation_flag).toRotationMatrix().transpose();
  Eigen::Matrix3f V;
  V << 0, -ref_angular_velocity(2), ref_angular_velocity(1),
       ref_angular_velocity(2), 0 , -ref_angular_velocity(0),
       -ref_angular_velocity(1), ref_angular_velocity(0), 0;
  V = R*V*R.transpose();
  Eigen::Vector3f wref(V(2,1), V(0,2), V(1,0));
  return (wref - angular_velocity);
} // get_angular_velocity_error

Eigen::Vector3f Orientation::calculate_control_input(Eigen::Quaternionf orientation, 
                                                     Eigen::Quaternionf ref_orientation,
                                                     Eigen::Vector3f angular_velocity,
                                                     Eigen::Vector3f ref_angular_velocity,
                                                     int orientation_flag)
{
  Eigen::Matrix3f rotation_matrix = orientation.toRotationMatrix().transpose();
  Eigen::Vector3f Tp = this->p_gain.array() *
                       ( this->get_orientation_error(orientation, ref_orientation,
                                                     orientation_flag) ).array();
  Eigen::Vector3f Td = this->d_gain.array() *
                       ( this->get_angular_velocity_error(angular_velocity, 
                                                          ref_angular_velocity, 
                                                          orientation_flag) ).array();
  Eigen::Vector3f T = Tp + Td - this->get_bias(rotation_matrix);
  return T;
} // calculate_control_input



} // namespace navatics_robot_controller
