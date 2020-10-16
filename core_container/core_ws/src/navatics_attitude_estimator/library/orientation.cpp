#include "navatics_attitude_estimator/orientation.hpp"

namespace navatics_attitude_estimator{

Orientation::Orientation(rclcpp::Node * node){
  // initialize quaternion storage variable
  this->orientation = Eigen::Quaternionf(1, 0, 0, 0);
  this->angular_velocity << 0, 0, 0;
  this->orientation_publisher = node->create_publisher<geometry_msgs::msg::Quaternion>
                                  ("state/orientation", 100);
  this->angular_velocity_publisher = node->create_publisher<geometry_msgs::msg::Vector3>
                                      ("state/angular_velocity", 100);
} // constructor

int Orientation::update_equation_unit_test(float weight, float q_threshold, float w_threshold)
{
  // generate parameters for test
  float dt = 0.01; // 0.01 s
  std::default_random_engine gen;
  std::uniform_int_distribution<int> dist(-100, 100);
  Eigen::Quaternionf q(1.0f, (float)dist(gen)/100, (float)dist(gen)/100, (float)dist(gen)/100);
  Eigen::Vector3f w((float)dist(gen)/100, (float)dist(gen)/100, 0.4);
  // initialize state
  this->initialize(q, w);
  // calculate q used for update
  Eigen::Vector3f r_axis = w.normalized();
  float r_angle = w.norm()*dt;
  Eigen::AngleAxisf _aa(r_angle, r_axis);
  Eigen::Quaternionf _qe(_aa);
  _qe.normalize();
  Eigen::Quaternionf new_rotation = q*_qe;
  // update orientation and angular velocity
  this->update_orientation(new_rotation, weight);
  this->update_angular_velocity(w, weight);
  Eigen::Quaternionf q_test_result = this->get_orientation();
  Eigen::Vector3f w_test_result = this->get_angular_velocity();
  // calculate target q
  r_angle = w.norm()*dt*weight;
  Eigen::AngleAxisf _target_aa(r_angle, r_axis);
  Eigen::Quaternionf _target_qe(_target_aa);
  _qe.normalize();
  Eigen::Quaternionf q_test = q*_target_qe;
  q_test.normalize();
  this->print_quaternion(q_test);
  this->print_quaternion(q_test_result);
  // calculate target w
  Eigen::Vector3f w_test = w*weight;
  // check test result
  int test_result = 0, q_test_result_flag = 0, w_test_result_flag = 0;
  if ( (fabs( q_test.w() - q_test_result.w() ) > q_threshold) or
       (fabs( q_test.x() - q_test_result.x() ) > q_threshold) or
       (fabs( q_test.y() - q_test_result.y() ) > q_threshold) or
       (fabs( q_test.z() - q_test_result.z() ) > q_threshold) ) { 
    std::cout << "Update rotation calculation test failed\n";
    q_test_result_flag = 1;
  } else {
    std::cout << "Update rotation calculation test succeeded\n";
    q_test_result_flag= 0;
  } // update equation check
  if ( fabs(w_test.norm() - w_test_result.norm()) > w_threshold ){
    std::cout << "Update angular velocity calculation test failed\n";
    w_test_result_flag = 1;
  } else {
    std::cout << "Update angular velocity calculation test succeeded\n";
    w_test_result_flag = 0;
  } // update equation check
  
  return q_test_result_flag + 2*w_test_result_flag;
} // update_unit_test

void Orientation::initialize(Eigen::Quaternionf q, Eigen::Vector3f w){
  this->orientation = q;
  this->orientation.normalize();
  this->angular_velocity = w;
  this->is_initialized = true;
} // initialize

void Orientation::update_orientation(Eigen::Quaternionf q, float weight){
  if (this->initialized()){
    // the update equation here
    Eigen::Quaternionf _qe = (q.inverse())*this->get_orientation();
    // convert quaternion error to angle-axis representation
    Eigen::AngleAxisf _aa(_qe);
    // multiply weight with the angle of angle-axis error representation
    _aa.angle() *= weight;
    // convert back to quaternion
    Eigen::Quaternionf _qupdate(_aa);
    this->orientation = this->get_orientation()*_qupdate.inverse();
    this->orientation.normalize();
  } // if initialized
} // update_orientation

void Orientation::update_angular_velocity(Eigen::Vector3f w, float weight){
  if (this->initialized()){
    this->angular_velocity = this->angular_velocity*(1-weight) + w*weight;
  } // if initialized
} // update_angular_velocity

Eigen::Quaternionf Orientation::get_orientation(){
  return this->orientation;
} // get_orientation

Eigen::Vector3f Orientation::get_angular_velocity(){
  return this->angular_velocity;
} // get_angular_velocity

bool Orientation::initialized(){
  return this->is_initialized;
} // initialized()

void Orientation::print_quaternion(Eigen::Quaternionf q){
  std::cout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << "\n";
} // print_quaternion

void Orientation::publish_state(){
  if (this->is_initialized){
    auto angular_velocity_msg = geometry_msgs::msg::Vector3();
    angular_velocity_msg.x = this->angular_velocity[0];
    angular_velocity_msg.y = this->angular_velocity[1];
    angular_velocity_msg.z = this->angular_velocity[2];
    this->angular_velocity_publisher->publish(angular_velocity_msg);
    auto quaternion_msg = geometry_msgs::msg::Quaternion();
    quaternion_msg.w = this->orientation.w();
    quaternion_msg.x = this->orientation.x();
    quaternion_msg.y = this->orientation.y();
    quaternion_msg.z = this->orientation.z();
    this->orientation_publisher->publish(quaternion_msg);
  }
}

} // namespace navatics_attitude_estimator
