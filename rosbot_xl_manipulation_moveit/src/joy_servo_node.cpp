#include <rosbot_xl_manipulation_moveit/joy_servo_node.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <moveit_msgs/srv/change_drift_dimensions.hpp>

namespace rosbot_xl_manipulation
{

JoyServoNode::JoyServoNode(const rclcpp::NodeOptions & options) : Node("joy_servo_node", options)
{
  dead_man_switch_ = JoyControlFactory(
    this->get_node_parameters_interface(), this->get_node_logging_interface(), "dead_man_switch");

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&JoyServoNode::JoyCb, this, std::placeholders::_1));

  StartServo();
  ChangeCartesianDriftDimensions();
}

void JoyServoNode::StartServo()
{
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client =
    this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
  while (!servo_start_client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO_STREAM(
      this->get_logger(), servo_start_client->get_service_name()
                            << " service not available, waiting again...");
  }
  servo_start_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
}

void JoyServoNode::ChangeCartesianDriftDimensions()
{
  rclcpp::Client<moveit_msgs::srv::ChangeDriftDimensions>::SharedPtr
    change_drift_dimensions_client = this->create_client<moveit_msgs::srv::ChangeDriftDimensions>(
      "/servo_node/change_drift_dimensions");
  while (!change_drift_dimensions_client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO_STREAM(
      this->get_logger(), change_drift_dimensions_client->get_service_name()
                            << " service not available, waiting again...");
  }

  this->declare_parameter("cartesian_drift_dimensions", rclcpp::PARAMETER_BOOL_ARRAY);
  std::vector<bool> cartesian_drift_dimensions =
    this->get_parameter("cartesian_drift_dimensions").as_bool_array();
  moveit_msgs::srv::ChangeDriftDimensions::Request::SharedPtr req =
    std::make_shared<moveit_msgs::srv::ChangeDriftDimensions::Request>();

  if (cartesian_drift_dimensions.size() != 6) {
    RCLCPP_ERROR_STREAM(
      this->get_logger(), "Invalid cartesian drift dimensions size (it should have 6 values)");
    throw std::runtime_error("Invalid cartesian drift dimensions size (it should have 6 values)");
  }

  req->drift_x_translation = cartesian_drift_dimensions[0];
  req->drift_y_translation = cartesian_drift_dimensions[1];
  req->drift_z_translation = cartesian_drift_dimensions[2];
  req->drift_x_rotation = cartesian_drift_dimensions[3];
  req->drift_y_rotation = cartesian_drift_dimensions[4];
  req->drift_z_rotation = cartesian_drift_dimensions[5];

  change_drift_dimensions_client->async_send_request(req);
}

void JoyServoNode::JoyCb(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // Lazy intialization of controlles - they require having fully constructed node
  // and shared_from_this can be called only after constructor
  if (!controllers_initialized_) {
    controllers_initialized_ = true;
    InitializeControllers();
  }

  if (!dead_man_switch_->IsPressed(msg)) {
    StopControllers(manipulator_controllers_);
    StopControllers(gripper_controllers_);
    return;
  }

  ProcessControllers(msg, manipulator_controllers_);
  ProcessControllers(msg, gripper_controllers_);
}

void JoyServoNode::ProcessControllers(
  const sensor_msgs::msg::Joy::SharedPtr msg,
  const std::vector<std::unique_ptr<ManipulationController>> & controllers)
{
  bool no_controller_activated = true;
  for (auto & c : controllers) {
    if (c->Process(msg)) {
      no_controller_activated = false;
      break;
    }
  }

  if (no_controller_activated) {
    StopControllers(controllers);
  }
}

void JoyServoNode::StopControllers(
  const std::vector<std::unique_ptr<ManipulationController>> & controllers)
{
  for (auto & c : controllers) {
    c->Stop();
  }
}

void JoyServoNode::InitializeControllers()
{
  manipulator_controllers_.push_back(
    std::make_unique<ManipulatorMoveGroupController>(this->shared_from_this()));
  manipulator_controllers_.push_back(
    std::make_unique<CartesianController>(this->shared_from_this()));
  manipulator_controllers_.push_back(std::make_unique<JointController>(this->shared_from_this()));

  gripper_controllers_.push_back(
    std::make_unique<GripperMoveGroupController>(this->shared_from_this()));
}

}  // namespace rosbot_xl_manipulation

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rosbot_xl_manipulation::JoyServoNode)