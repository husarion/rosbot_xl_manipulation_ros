#include <rosbot_xl_manipulation_moveit/joy_servo_node.hpp>

namespace rosbot_xl_manipulation
{
JoyServoNode::JoyServoNode(const rclcpp::NodeOptions & options) : Node("joy_servo_node", options)
{
  dead_man_switch_ = JoyControlFactory(
    this->get_node_parameters_interface(), this->get_node_logging_interface(), "dead_man_switch");

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&JoyServoNode::JoyCb, this, std::placeholders::_1));
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