#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace rosbot_xl_manipulation
{
class JoyControl
{
public:
  virtual bool IsPressed(const sensor_msgs::msg::Joy::SharedPtr msg) const = 0;
  virtual double GetValue(const sensor_msgs::msg::Joy::SharedPtr msg) const = 0;
};

class AxisControl : public JoyControl
{
public:
  AxisControl(
    int axis_id, double axis_deadzone, double scaling = 1.0, bool inverted_control = false)
  {
    axis_id_ = axis_id;
    axis_deadzone_ = axis_deadzone;
    scaling_ = inverted_control ? -scaling : scaling;
  }

  bool IsPressed(const sensor_msgs::msg::Joy::SharedPtr msg) const override
  {
    return std::fabs(msg->axes[axis_id_]) > axis_deadzone_;
  }
  double GetValue(const sensor_msgs::msg::Joy::SharedPtr msg) const override
  {
    return msg->axes[axis_id_] * scaling_;
  }

private:
  int axis_id_;
  double axis_deadzone_;
  double scaling_;
};

class DoubleButtonControl : public JoyControl
{
public:
  DoubleButtonControl(int positive_button_id, int negative_button_id, double scaling = 1.0)
  {
    positive_button_id_ = positive_button_id;
    negative_button_id_ = negative_button_id;
    scaling_ = scaling;
  }

  bool IsPressed(const sensor_msgs::msg::Joy::SharedPtr msg) const override
  {
    return msg->buttons[positive_button_id_] || msg->buttons[negative_button_id_];
  }
  double GetValue(const sensor_msgs::msg::Joy::SharedPtr msg) const override
  {
    return (msg->buttons[positive_button_id_] - msg->buttons[negative_button_id_]) * scaling_;
  }

private:
  int positive_button_id_;
  int negative_button_id_;
  double scaling_;
};

class SingleButtonControl : public JoyControl
{
public:
  SingleButtonControl(int button_id, double scaling = 1.0)
  {
    button_id_ = button_id;
    scaling_ = scaling;
  }

  bool IsPressed(const sensor_msgs::msg::Joy::SharedPtr msg) const override
  {
    return msg->buttons[button_id_];
  }
  double GetValue(const sensor_msgs::msg::Joy::SharedPtr msg) const override
  {
    return msg->buttons[button_id_] * scaling_;
  }

private:
  int button_id_;
  double scaling_;
};

std::unique_ptr<JoyControl> ParseJoyControl(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & param_itf,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & logging_itf,
  std::string param_namespace, double scaling = 1.0)
{
  param_itf->declare_parameter(param_namespace + ".control_type", rclcpp::ParameterValue(""));
  std::string control_type =
    param_itf->get_parameter(param_namespace + ".control_type").as_string();

  std::unique_ptr<JoyControl> controller;

  if (control_type == "double_button") {
    param_itf->declare_parameter(
      param_namespace + ".positive_button_id", rclcpp::PARAMETER_INTEGER);
    param_itf->declare_parameter(
      param_namespace + ".negative_button_id", rclcpp::PARAMETER_INTEGER);
    int positive_button_id =
      param_itf->get_parameter(param_namespace + ".positive_button_id").as_int();
    int negative_button_id =
      param_itf->get_parameter(param_namespace + ".negative_button_id").as_int();
    controller =
      std::make_unique<DoubleButtonControl>(positive_button_id, negative_button_id, scaling);
  } else if (control_type == "single_button") {
    param_itf->declare_parameter(param_namespace + ".button_id", rclcpp::PARAMETER_INTEGER);
    int button_id = param_itf->get_parameter(param_namespace + ".button_id").as_int();
    controller = std::make_unique<SingleButtonControl>(button_id, scaling);
  } else if (control_type == "axis") {
    if (!param_itf->has_parameter("axis_deadzone")) {
      param_itf->declare_parameter("axis_deadzone", rclcpp::ParameterValue(0.05));
    }
    double axis_deadzone = param_itf->get_parameter("axis_deadzone").as_double();
    param_itf->declare_parameter(param_namespace + ".axis_id", rclcpp::PARAMETER_INTEGER);
    param_itf->declare_parameter(param_namespace + ".inverted", rclcpp::ParameterValue(false));
    int axis_id = param_itf->get_parameter(param_namespace + ".axis_id").as_int();
    bool inverted = param_itf->get_parameter(param_namespace + ".inverted").as_bool();
    controller = std::make_unique<AxisControl>(axis_id, axis_deadzone, scaling, inverted);
  } else {
    RCLCPP_ERROR_STREAM(
      logging_itf->get_logger(),
      "Unknown control type " << control_type << " in " << param_namespace
                              << ", currently supported types: single_button, double_button, axis. "
                                 "Please make sure that it is defined.");
    throw std::runtime_error("Unknown control type");
  }

  return controller;
}

class Controller
{
public:
  virtual void Stop() = 0;
  virtual bool Process(const sensor_msgs::msg::Joy::SharedPtr msg) = 0;

protected:
  bool CheckIfPressed(
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

  std::vector<double> CalculateCommand(
    const sensor_msgs::msg::Joy::SharedPtr msg, const std::vector<std::string> & cmd_names,
    const std::map<std::string, std::unique_ptr<JoyControl>> & controls)
  {
    std::vector<double> cmds(cmd_names.size(), 0.0);
    for (auto const & c : controls) {
      if (c.second->IsPressed(msg)) {
        auto name_it = std::find(cmd_names.begin(), cmd_names.end(), c.first);
        int name_idx = std::distance(cmd_names.begin(), name_it);
        cmds[name_idx] = c.second->GetValue(msg);
      }
    }
    return cmds;
  }
};

class JointController : public Controller
{
public:
  JointController(const rclcpp::Node::SharedPtr & node)
  {
    clock_itf_ = node->get_node_clock_interface();
    ParseParameters(node);

    joint_cmds_pub_ =
      node->create_publisher<control_msgs::msg::JointJog>("servo_node/delta_joint_cmds", 10);
  };

  bool Process(const sensor_msgs::msg::Joy::SharedPtr msg) override
  {
    if (CheckIfPressed(msg, manipulator_joint_controls_)) {
      std::vector<double> cmds = CalculateCommand(msg, joint_names_, manipulator_joint_controls_);
      SendJointCommand(cmds, msg->header.stamp);
      return true;
    }
    return false;
  }

  void Stop() override
  {
    SendJointCommand(std::vector<double>(joint_names_.size(), 0.0), clock_itf_->get_clock()->now());
  }

private:
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_cmds_pub_;

  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_itf_;

  std::map<std::string, std::unique_ptr<JoyControl>> manipulator_joint_controls_;

  std::vector<std::string> joint_names_;
  double joint_control_velocity_;

  void ParseParameters(const rclcpp::Node::SharedPtr & node)
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

      manipulator_joint_controls_[joint_name] = ParseJoyControl(
        node->get_node_parameters_interface(), node->get_node_logging_interface(), param_namespace,
        joint_control_velocity_);
    }
  }

  void SendJointCommand(
    const std::vector<double> & cmds, const builtin_interfaces::msg::Time & timestamp)
  {
    control_msgs::msg::JointJog joint_cmd_msg;
    joint_cmd_msg.header.stamp = timestamp;
    joint_cmd_msg.duration = 0.0;
    joint_cmd_msg.joint_names = joint_names_;
    joint_cmd_msg.velocities = cmds;
    joint_cmds_pub_->publish(joint_cmd_msg);
  }
};

class CartesianController : public Controller
{
public:
  CartesianController(const rclcpp::Node::SharedPtr & node)
  : cartesian_cmd_names_{
      "linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z",
    }
  {
    clock_itf_ = node->get_node_clock_interface();
    ParseParameters(node);

    twist_cmds_pub_ =
      node->create_publisher<geometry_msgs::msg::TwistStamped>("servo_node/delta_twist_cmds", 10);
  };

  bool Process(const sensor_msgs::msg::Joy::SharedPtr msg) override
  {
    if (CheckIfPressed(msg, manipulator_cartesian_controls_)) {
      std::vector<double> cmds =
        CalculateCommand(msg, cartesian_cmd_names_, manipulator_cartesian_controls_);
      SendCartesianCommand(cmds, msg->header.stamp);
      return true;
    }
    return false;
  }

  void Stop() override
  {
    SendCartesianCommand(
      std::vector<double>(cartesian_cmd_names_.size(), 0.0), clock_itf_->get_clock()->now());
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmds_pub_;

  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_itf_;

  std::map<std::string, std::unique_ptr<JoyControl>> manipulator_cartesian_controls_;

  std::vector<std::string> cartesian_control_names_;
  std::vector<std::string> cartesian_cmd_names_;
  std::string cartesian_control_reference_frame_;
  double cartesian_control_velocity_linear_;
  double cartesian_control_velocity_angular_;

  void ParseParameters(const rclcpp::Node::SharedPtr & node)
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
        std::find(
          cartesian_cmd_names_.begin(), cartesian_cmd_names_.end(), cartesian_control_name) ==
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

      manipulator_cartesian_controls_[cartesian_control_name] = ParseJoyControl(
        node->get_node_parameters_interface(), node->get_node_logging_interface(), param_namespace,
        scaling);
    }
  }

  void SendCartesianCommand(
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
};

class MoveGroupController : public Controller
{
public:
  MoveGroupController(const rclcpp::Node::SharedPtr & node)
  {
    move_group_manipulator_ =
      std::make_unique<moveit::planning_interface::MoveGroupInterface>(node, "manipulator");
    ParseParameters(node);
  }

  bool Process(const sensor_msgs::msg::Joy::SharedPtr msg) override
  {
    // Gripper commands shouldn't affect stopping manipulator, so they are not considered for
    // output of method. It would be better to move gripper to seperate controller, but it
    // also uses move group, which will make this code much more complicated
    if (gripper_close_->IsPressed(msg)) {
      CloseGripper();
    } else if (gripper_open_->IsPressed(msg)) {
      OpenGripper();
    }

    if (home_manipulator_->IsPressed(msg)) {
      MoveToHome();
      return true;
    }

    return false;
  }

  void Stop() override {}

private:
  moveit::planning_interface::MoveGroupInterfacePtr move_group_manipulator_;

  std::unique_ptr<JoyControl> home_manipulator_;
  std::unique_ptr<JoyControl> gripper_close_;
  std::unique_ptr<JoyControl> gripper_open_;

  std::string gripper_joint_name_;
  std::vector<double> home_joint_configuration_;
  double opened_gripper_position_;
  double closed_gripper_position_;

  void ParseParameters(const rclcpp::Node::SharedPtr & node)
  {
    node->declare_parameter<std::string>("gripper_joint_names", "gripper_left_joint");
    gripper_joint_name_ = node->get_parameter("gripper_joint_names").as_string();

    node->declare_parameter("home_joint_configuration", rclcpp::PARAMETER_DOUBLE_ARRAY);
    // TODO check ParameterUninitializedException exception
    try {
      home_joint_configuration_ = node->get_parameter("home_joint_configuration").as_double_array();
    } catch (const rclcpp::exceptions::ParameterUninitializedException & e) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Required parameter not defined: " << e.what());
      throw e;
    }

    node->declare_parameters<double>(
      "", {{"gripper_control.open.position", 0.009}, {"gripper_control.close.position", -0.009}});
    opened_gripper_position_ = node->get_parameter("gripper_control.open.position").as_double();
    closed_gripper_position_ = node->get_parameter("gripper_control.close.position").as_double();

    gripper_open_ = ParseJoyControl(
      node->get_node_parameters_interface(), node->get_node_logging_interface(),
      "gripper_control.open");
    gripper_close_ = ParseJoyControl(
      node->get_node_parameters_interface(), node->get_node_logging_interface(),
      "gripper_control.close");

    home_manipulator_ = ParseJoyControl(
      node->get_node_parameters_interface(), node->get_node_logging_interface(),
      "home_manipulator");
  }

  void CloseGripper()
  {
    move_group_manipulator_->getNamedTargetValues("gripper").at(gripper_joint_name_) =
      closed_gripper_position_;
    move_group_manipulator_->setJointValueTarget(
      move_group_manipulator_->getNamedTargetValues("gripper"));
    move_group_manipulator_->move();
  }

  void OpenGripper()
  {
    move_group_manipulator_->getNamedTargetValues("gripper").at(gripper_joint_name_) =
      opened_gripper_position_;
    move_group_manipulator_->setJointValueTarget(
      move_group_manipulator_->getNamedTargetValues("gripper"));
    move_group_manipulator_->move();
  }

  void MoveToHome()
  {
    move_group_manipulator_->setJointValueTarget(home_joint_configuration_);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_manipulator_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
      move_group_manipulator_->execute(plan);
    }
  }
};

class JoyServoNode : public rclcpp::Node
{
public:
  JoyServoNode(const rclcpp::NodeOptions & options) : Node("joy_servo_node", options)
  {
    dead_man_switch_ = ParseJoyControl(
      this->get_node_parameters_interface(), this->get_node_logging_interface(), "dead_man_switch");

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyServoNode::JoyCb, this, std::placeholders::_1));
  }

  void InitializeControllers()
  {
    controllers_.push_back(std::make_unique<MoveGroupController>(this->shared_from_this()));
    controllers_.push_back(std::make_unique<CartesianController>(this->shared_from_this()));
    controllers_.push_back(std::make_unique<JointController>(this->shared_from_this()));
  }

private:
  std::vector<std::unique_ptr<Controller>> controllers_;
  // Lazy intialization of controlles - they require having fully constructed node
  // and shared_from_this can be called only after constructor
  bool controllers_initialized_ = false;

  std::unique_ptr<JoyControl> dead_man_switch_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  void JoyCb(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (!controllers_initialized_) {
      controllers_initialized_ = true;
      InitializeControllers();
    }

    if (!dead_man_switch_->IsPressed(msg)) {
      StopManipulator();
      return;
    }

    bool no_controller_activated = true;
    for (auto & c : controllers_) {
      if (c->Process(msg)) {
        no_controller_activated = false;
        break;
      }
    }

    if (no_controller_activated) {
      StopManipulator();
    }
  }

  void StopManipulator()
  {
    for (auto & c : controllers_) {
      c->Stop();
    }
  }
};
}  // namespace rosbot_xl_manipulation

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rosbot_xl_manipulation::JoyServoNode)