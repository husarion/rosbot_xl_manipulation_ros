#include <rosbot_xl_manipulation_moveit/manipulation_controller.hpp>

#include <rclcpp/parameter.hpp>

namespace rosbot_xl_manipulation
{

bool ManipulationController::CheckIfPressed(
  const sensor_msgs::msg::Joy::SharedPtr msg,
  const std::map<std::string, std::unique_ptr<JoyControl>> & controls)
{
  for (const auto & c : controls) {
    if (c.second->IsPressed(msg)) {
      return true;
    }
  }
  return false;
}

std::vector<double> ManipulationController::CalculateCommand(
  const sensor_msgs::msg::Joy::SharedPtr msg, const std::vector<std::string> & cmd_names,
  const std::map<std::string, std::unique_ptr<JoyControl>> & controls)
{
  std::vector<double> cmds(cmd_names.size(), 0.0);
  for (auto const & c : controls) {
    if (c.second->IsPressed(msg)) {
      auto name_it = std::find(cmd_names.begin(), cmd_names.end(), c.first);
      int name_idx = std::distance(cmd_names.begin(), name_it);
      cmds[name_idx] = c.second->GetControlValue(msg);
    }
  }
  return cmds;
}

JointController::JointController(const rclcpp::Node::SharedPtr & node)
{
  clock_itf_ = node->get_node_clock_interface();
  ParseParameters(node);

  joint_cmds_pub_ =
    node->create_publisher<control_msgs::msg::JointJog>("servo_node/delta_joint_cmds", 10);
};

bool JointController::Process(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (CheckIfPressed(msg, manipulator_joint_controls_)) {
    std::vector<double> cmds = CalculateCommand(msg, joint_names_, manipulator_joint_controls_);
    SendJointCommand(cmds, msg->header.stamp);
    return true;
  }
  return false;
}

void JointController::Stop()
{
  SendJointCommand(std::vector<double>(joint_names_.size(), 0.0), clock_itf_->get_clock()->now());
}

void JointController::ParseParameters(const rclcpp::Node::SharedPtr & node)
{
  node->declare_parameter("joint_control_velocity", 0.5);
  joint_control_velocity_ = node->get_parameter("joint_control_velocity").as_double();

  node->declare_parameter("joint_names", rclcpp::PARAMETER_STRING_ARRAY);
  // TODO check ParameterUninitializedException exception
  try {
    joint_names_ = node->get_parameter("joint_names").as_string_array();
  } catch (const rclcpp::exceptions::ParameterUninitializedException & e) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Required parameter not defined: " << e.what());
    throw e;
  }
  for (const auto & joint_name : joint_names_) {
    std::string param_namespace = "joints_control." + joint_name;

    manipulator_joint_controls_[joint_name] = JoyControlFactory(
      node->get_node_parameters_interface(), node->get_node_logging_interface(), param_namespace,
      joint_control_velocity_);
  }
}

void JointController::SendJointCommand(
  const std::vector<double> & cmds, const builtin_interfaces::msg::Time & timestamp)
{
  control_msgs::msg::JointJog joint_cmd_msg;
  joint_cmd_msg.header.stamp = timestamp;
  joint_cmd_msg.duration = 0.0;
  joint_cmd_msg.joint_names = joint_names_;
  joint_cmd_msg.velocities = cmds;
  joint_cmds_pub_->publish(joint_cmd_msg);
}

CartesianController::CartesianController(const rclcpp::Node::SharedPtr & node)
: cartesian_cmd_names_{
    "linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z",
  }
{
  clock_itf_ = node->get_node_clock_interface();
  ParseParameters(node);

  twist_cmds_pub_ =
    node->create_publisher<geometry_msgs::msg::TwistStamped>("servo_node/delta_twist_cmds", 10);
};

bool CartesianController::Process(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (CheckIfPressed(msg, manipulator_cartesian_controls_)) {
    std::vector<double> cmds =
      CalculateCommand(msg, cartesian_cmd_names_, manipulator_cartesian_controls_);
    SendCartesianCommand(cmds, msg->header.stamp);
    return true;
  }
  return false;
}

void CartesianController::Stop()
{
  SendCartesianCommand(
    std::vector<double>(cartesian_cmd_names_.size(), 0.0), clock_itf_->get_clock()->now());
}

void CartesianController::ParseParameters(const rclcpp::Node::SharedPtr & node)
{
  node->declare_parameter<std::string>("cartesian_control_reference_frame", "link2");
  cartesian_control_reference_frame_ =
    node->get_parameter("cartesian_control_reference_frame").as_string();

  node->declare_parameters<double>(
    "", {
          {"cartesian_control_velocity_linear", 0.1},
          {"cartesian_control_velocity_angular", 0.4},
        });
  cartesian_control_velocity_linear_ =
    node->get_parameter("cartesian_control_velocity_linear").as_double();
  cartesian_control_velocity_angular_ =
    node->get_parameter("cartesian_control_velocity_angular").as_double();

  node->declare_parameter("cartesian_control_names", rclcpp::PARAMETER_STRING_ARRAY);
  try {
    cartesian_control_names_ = node->get_parameter("cartesian_control_names").as_string_array();
  } catch (const rclcpp::exceptions::ParameterUninitializedException & e) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Required parameter not defined: " << e.what());
    throw e;
  }
  for (const auto & cartesian_control_name : cartesian_control_names_) {
    if (
      std::find(cartesian_cmd_names_.begin(), cartesian_cmd_names_.end(), cartesian_control_name) ==
      cartesian_cmd_names_.end()) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Unknown cartesian control type {cartesian_control_name},"
        " currently supported names: {self.cartesian_control_names}");
      continue;
    }
    std::string param_namespace = "cartesian_control." + cartesian_control_name;

    double scaling = 1.0;
    if (cartesian_control_name.find("linear") != std::string::npos) {
      scaling = cartesian_control_velocity_linear_;
    } else if (cartesian_control_name.find("angular") != std::string::npos) {
      scaling = cartesian_control_velocity_angular_;
    }

    manipulator_cartesian_controls_[cartesian_control_name] = JoyControlFactory(
      node->get_node_parameters_interface(), node->get_node_logging_interface(), param_namespace,
      scaling);
  }
}

void CartesianController::SendCartesianCommand(
  const std::vector<double> & cmds, const builtin_interfaces::msg::Time & timestamp)
{
  geometry_msgs::msg::TwistStamped cartesian_cmd_msg;
  cartesian_cmd_msg.header.stamp = timestamp;
  cartesian_cmd_msg.header.frame_id = cartesian_control_reference_frame_;
  cartesian_cmd_msg.twist.linear.x = cmds[0];
  cartesian_cmd_msg.twist.linear.y = cmds[1];
  cartesian_cmd_msg.twist.linear.z = cmds[2];
  cartesian_cmd_msg.twist.angular.x = cmds[3];
  cartesian_cmd_msg.twist.angular.y = cmds[4];
  cartesian_cmd_msg.twist.angular.z = cmds[5];

  twist_cmds_pub_->publish(cartesian_cmd_msg);
}

ManipulatorMoveGroupController::ManipulatorMoveGroupController(const rclcpp::Node::SharedPtr & node)
{
  move_group_manipulator_ =
    std::make_unique<moveit::planning_interface::MoveGroupInterface>(node, "manipulator");
  ParseParameters(node);
}

bool ManipulatorMoveGroupController::Process(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (home_manipulator_->IsPressed(msg)) {
    if (!action_already_executed_) {
      action_already_executed_ = true;
      MoveToHome();
    }
    return true;
  }
  action_already_executed_ = false;
  return false;
}

void ManipulatorMoveGroupController::ParseParameters(const rclcpp::Node::SharedPtr & node)
{
  home_manipulator_ = JoyControlFactory(
    node->get_node_parameters_interface(), node->get_node_logging_interface(), "home_manipulator");
}

void ManipulatorMoveGroupController::MoveToHome()
{
  move_group_manipulator_->setNamedTarget("Home");
  move_group_manipulator_->move();
}

GripperMoveGroupController::GripperMoveGroupController(const rclcpp::Node::SharedPtr & node)
{
  move_group_gripper_ =
    std::make_unique<moveit::planning_interface::MoveGroupInterface>(node, "gripper");
  ParseParameters(node);
}

bool GripperMoveGroupController::Process(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (toggle_gripper_position_->IsPressed(msg)) {
    if (!action_already_executed_) {
      action_already_executed_ = true;
      if (gripper_position_ == GripperPosition::CLOSED) {
        OpenGripper();
      } else if (gripper_position_ == GripperPosition::OPENED) {
        CloseGripper();
      }
    }
    return true;
  }
  action_already_executed_ = false;
  return false;
}

void GripperMoveGroupController::ParseParameters(const rclcpp::Node::SharedPtr & node)
{
  toggle_gripper_position_ = JoyControlFactory(
    node->get_node_parameters_interface(), node->get_node_logging_interface(),
    "gripper_control.toggle");
}

void GripperMoveGroupController::CloseGripper()
{
  move_group_gripper_->setNamedTarget("Closed");
  move_group_gripper_->move();
  gripper_position_ = GripperPosition::CLOSED;
}

void GripperMoveGroupController::OpenGripper()
{
  move_group_gripper_->setNamedTarget("Opened");
  move_group_gripper_->move();
  gripper_position_ = GripperPosition::OPENED;
}

}  // namespace rosbot_xl_manipulation
