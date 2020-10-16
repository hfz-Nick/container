#include "navatics_robot_controller/controller.hpp"

namespace navatics_robot_controller{

Controller::Controller(rclcpp::Node * node){
  // initialize parameter storage
  this->position_params.initialize("position controller");
  this->orientation_params.initialize("orientation controller");
  this->position << 0,0,0;
  this->orientation = Eigen::Quaternionf(1,0,0,0);
  // declare and get params
  this->declare_params(node);
  this->get_params(node);
  // print params
  this->bias_params.print();
  this->orientation_params.print();
  this->position_params.print();

  // initialize states
  this->initialize_states();

  // initialize controllers
  this->position_controller = new Position(this->position_params.p_gain, this->position_params.d_gain,
                                           this->max_cmd_force, this->position_params.use_compensation,
                                           this->bias_params.weight, this->bias_params.buoyancy);

  this->orientation_controller = new Orientation(this->orientation_params.p_gain, 
                                                 this->orientation_params.d_gain,
                                                 this->orientation_params.use_compensation,
                                                 this->bias_params.center_of_gravity,
                                                 this->bias_params.center_of_buoyancy,
                                                 this->bias_params.buoyancy);
  this->thrusters_controller = new Thrusters(node);
  // initialize refstate
  this->ref = new RefState(node, this->max_velocity, this->refstate_update_hz, true);

  // create subscribers
  using namespace std::placeholders;
  this->orientation_sub = node->create_subscription<geometry_msgs::msg::Quaternion>
                            ("state/orientation", 200,
                              std::bind(&Controller::orientation_sub_callback, this, _1));
  this->position_sub = node->create_subscription<geometry_msgs::msg::Vector3>
                      ("state/position", 200, std::bind(&Controller::position_sub_callback, this, _1));
  this->velocity_sub = node->create_subscription<geometry_msgs::msg::Vector3>
                      ("state/velocity", 200, std::bind(&Controller::velocity_sub_callback, this, _1));
  this->angular_velocity_sub = node->create_subscription<geometry_msgs::msg::Vector3>
                        ("state/angular_velocity", 200, 
                         std::bind(&Controller::angular_velocity_sub_callback, this, _1));
  this->orientation_flag_sub = node->create_subscription<std_msgs::msg::UInt8>
                               ("state/orientation/flag", 200, 
                                std::bind(&Controller::orientation_flag_sub_callback, this, _1));

  // initialize controller timer
  std::chrono::milliseconds controller_update_duration_ms((int)(1000/this->controller_update_hz));
  this->controller_timer = node->create_wall_timer( controller_update_duration_ms,
                            std::bind(&Controller::controller_timer_callback, this) );
  std::chrono::milliseconds rpm_update_duration_ms((int)(1000/this->rpm_update_hz));
  this->rpm_timer = node->create_wall_timer( rpm_update_duration_ms,
                            std::bind(&Controller::rpm_timer_callback, this) );

  // for debug, remove immediately when deploy
  // this->ref->initialize(this->position, this->velocity, this->orientation, this->angular_velocity);
  // this->is_controller_initialized = true;
} // constructor

void Controller::controller_timer_callback(){
  if (this->is_controller_initialized){
    Eigen::Quaternionf tilt_bias = this->orientation_controller->get_orientation_bias(
                                    this->orientation_flag);
    Eigen::Vector3f T = this->position_controller->calculate_control_input
                         ( this->position, this->ref->get_ref_position(), this->velocity,
                           this->ref->get_ref_velocity(), this->ref->get_user_command(), 
                           this->orientation, this->ref->get_ref_orientation(), tilt_bias );
    // std::cout << "T: " << T.transpose() << "\n";
    Eigen::Vector3f M = this->orientation_controller->calculate_control_input
                         ( this->orientation, this->ref->get_ref_orientation(), 
                           this->angular_velocity, this->ref->get_ref_angular_velocity(),
                           this->orientation_flag );
    // std::cout << "M: " << M.transpose() << "\n";
    Eigen::VectorXi _rpm = this->thrusters_controller->calculate_rpm(T, M);
    // std::cout << "RPM: " << _rpm.transpose() << "\n";
    this->rpm = _rpm;
  } // if initialized
} // controller_timer_callback

void Controller::rpm_timer_callback(){
  if (this->is_controller_initialized)
    this->thrusters_controller->publish_rpm(this->rpm);
} // rpm_timer_callback

void Controller::orientation_flag_sub_callback(const std_msgs::msg::UInt8::SharedPtr msg){
  if (msg->data > 5)
    this->orientation_flag = 0;
  else
    this->orientation_flag = msg->data;
}

void Controller::orientation_sub_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg){
  this->orientation.w() = msg->w;
  this->orientation.x() = msg->x;
  this->orientation.y() = msg->y;
  this->orientation.z() = msg->z;
  if (!this->is_orientation_initialized)
    this->is_orientation_initialized = true;
  // initialize refstate and controller
  if (this->is_orientation_initialized and this->is_position_initialized and
      !this->is_controller_initialized)
  {
    std::cout << "refstate initialized at " << this->position.transpose() << "\n";
    this->ref->initialize(this->position, this->velocity, this->orientation, this->angular_velocity);
    this->is_controller_initialized = true;
  } // initialize refstate and the entire controller
} // orientation_sub_callback

void Controller::position_sub_callback(const geometry_msgs::msg::Vector3::SharedPtr msg){
  this->position[0] = msg->x;
  this->position[1] = msg->y;
  this->position[2] = msg->z;
  if (!this->is_position_initialized)
    this->is_position_initialized = true;
} // position_sub_callback

void Controller::velocity_sub_callback(const geometry_msgs::msg::Vector3::SharedPtr msg){
  this->velocity[0] = msg->x;
  this->velocity[1] = msg->y;
  this->velocity[2] = msg->z;
} // velocity_sub_callback

void Controller::angular_velocity_sub_callback(const geometry_msgs::msg::Vector3::SharedPtr msg){
  this->angular_velocity[0] = msg->x;
  this->angular_velocity[1] = msg->y;
  this->angular_velocity[2] = msg->z;
} // angular_velocity_sub_callback

void Controller::initialize_states(){
  // initialize to default values
  this->orientation = Eigen::Quaternionf(1,0,0,0);
  this->orientation.normalize();
  this->angular_velocity << 0,0,0;
  this->position << 0,0,0;
  this->velocity << 0,0,0;
  this->user_cmd << 0,0,0;
} // initialize_states

void Controller::declare_params(rclcpp::Node * node){
  // declare update rate
  node->declare_parameter("controller_update_hz");
  node->declare_parameter("refstate_update_hz");
  node->declare_parameter("rpm_update_hz");
  // declare parameter for controller
  node->declare_parameter("controller.cmd_force");
  node->declare_parameter("controller.max_velocity");
  node->declare_parameter("controller.position.p_gain");
  node->declare_parameter("controller.position.d_gain");
  node->declare_parameter("controller.position.compensate_bias");
  node->declare_parameter("controller.orientation.p_gain");
  node->declare_parameter("controller.orientation.d_gain");
  node->declare_parameter("controller.orientation.flag");
  node->declare_parameter("controller.orientation.compensate_bias");
  node->declare_parameter("controller.bias.weight");
  node->declare_parameter("controller.bias.buoyancy");
  node->declare_parameter("controller.bias.center_of_gravity");
  node->declare_parameter("controller.bias.center_of_buoyancy");
} // declare_params

void Controller::get_params(rclcpp::Node * node){
  // get optional controller and rpm update rate
  node->get_parameter_or("controller_update_hz", this->controller_update_hz, 100.0f);
  node->get_parameter_or("refstate_update_hz", this->refstate_update_hz, 50.0f);
  node->get_parameter_or("rpm_update_hz", this->rpm_update_hz, 100.0f);
  // get controller params
  // position
  rclcpp::Parameter p("", std::vector<double>({0.0, 0.0, 0.0}));
  node->get_parameter("controller.position.p_gain", p);
  this->position_params.p_gain = this->get_float_vector(p);
  node->get_parameter("controller.position.d_gain", p);
  this->position_params.d_gain = this->get_float_vector(p);
  node->get_parameter_or("controller.position.compensate_bias", this->position_params.use_compensation,
                         true);
  // max controller
  rclcpp::Parameter cf_p("", std::vector<double>({1.0, 1.0, 0.0})); // commanded force parameter
  node->get_parameter("controller.cmd_force", cf_p);
  this->max_cmd_force = this->get_float_vector(cf_p);
  std::cout << "Max Commanded Force: " << this->max_cmd_force.transpose() << "\n";
  node->get_parameter("controller.max_velocity", cf_p);
  this->max_velocity = this->get_float_vector(cf_p);
  std::cout << "Max Commanded Velocity: " << this->max_velocity.transpose() << "\n";
  // orientation
  node->get_parameter("controller.orientation.p_gain", p);
  this->orientation_params.p_gain = this->get_float_vector(p);
  node->get_parameter("controller.orientation.d_gain", p);
  this->orientation_params.d_gain = this->get_float_vector(p);
  node->get_parameter_or("controller.orientation.compensate_bias",
                         this->orientation_params.use_compensation, true);
  node->get_parameter_or("controller.orientation.flag", this->orientation_flag, 0);
  std::cout << "Orientation flag: " << this->orientation_flag << "\n";
  // bias
  node->get_parameter_or("controller.bias.weight", this->bias_params.weight, 0.0f);
  node->get_parameter_or("controller.bias.buoyancy", this->bias_params.buoyancy, 0.0f);
  node->get_parameter("controller.bias.center_of_gravity", p);
  this->bias_params.center_of_gravity = this->get_float_vector(p);
  node->get_parameter("controller.bias.center_of_buoyancy", p);
  this->bias_params.center_of_buoyancy = this->get_float_vector(p);
} // get_params

Eigen::Vector3f Controller::get_float_vector(rclcpp::Parameter p){
  return (Eigen::Vector3d(p.as_double_array().data()).cast <float> ());
} // extract_float_vector

} // namespace navatics_robot_controller
