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
  virtual float GetValue(const sensor_msgs::msg::Joy::SharedPtr msg) const = 0;
};

class AxisControl : public JoyControl
{
public:
  AxisControl(int axis_id, float axis_deadzone, float scaling = 1.0, bool inverted_control = false)
  {
    axis_id_ = axis_id;
    axis_deadzone_ = axis_deadzone;
    scaling_ = inverted_control ? -scaling : scaling;
  }

  bool IsPressed(const sensor_msgs::msg::Joy::SharedPtr msg) const override
  {
    return std::fabs(msg->axes[axis_id_]) > axis_deadzone_;
  }
  float GetValue(const sensor_msgs::msg::Joy::SharedPtr msg) const override
  {
    return msg->axes[axis_id_] * scaling_;
  }

private:
  int axis_id_;
  float axis_deadzone_;
  float scaling_;
};

class DoubleButtonControl : public JoyControl
{
public:
  DoubleButtonControl(int positive_button_id, int negative_button_id, float scaling = 1.0)
  {
    positive_button_id_ = positive_button_id;
    negative_button_id_ = negative_button_id;
    scaling_ = scaling;
  }

  bool IsPressed(const sensor_msgs::msg::Joy::SharedPtr msg) const override
  {
    return msg->buttons[positive_button_id_] || msg->buttons[negative_button_id_];
  }
  float GetValue(const sensor_msgs::msg::Joy::SharedPtr msg) const override
  {
    return (msg->buttons[positive_button_id_] - msg->buttons[negative_button_id_]) * scaling_;
  }

private:
  int positive_button_id_;
  int negative_button_id_;
  float scaling_;
};

class SingleButtonControl : public JoyControl
{
public:
  SingleButtonControl(int button_id, float scaling = 1.0)
  {
    button_id_ = button_id;
    scaling_ = scaling;
  }

  bool IsPressed(const sensor_msgs::msg::Joy::SharedPtr msg) const override
  {
    return msg->buttons[button_id_];
  }
  float GetValue(const sensor_msgs::msg::Joy::SharedPtr msg) const override
  {
    return msg->buttons[button_id_] * scaling_;
  }

private:
  int button_id_;
  float scaling_;
};

class JoyServoNode : public rclcpp::Node
{
public:
  JoyServoNode(const rclcpp::NodeOptions & options)
  : Node("joy_servo_node", options),
    cartesian_cmd_names_{
      "linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z",
    }
  {
    ParseParameters();

    joint_cmds_pub_ =
      this->create_publisher<control_msgs::msg::JointJog>("servo_node/delta_joint_cmds", 10);
    twist_cmds_pub_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>("servo_node/delta_twist_cmds", 10);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyServoNode::JoyCb, this, std::placeholders::_1));
  }

  void Initialize()
  {
    // https://answers.ros.org/question/353828/getting-a-nodesharedptr-from-this/
    // can't be in constructor
    move_group_manipulator_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      this->shared_from_this(), "manipulator");
  }

private:
  moveit::planning_interface::MoveGroupInterfacePtr move_group_manipulator_;
  void JoyCb(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (!dead_man_switch_->IsPressed(msg)) {
      StopManipulator();
      return;
    }

    if (home_manipulator_->IsPressed(msg)) {
      MoveToHome();
    } else if (CheckIfPressed(msg, manipulator_cartesian_controls_)) {
      std::vector<double> cmds =
        CalculateCommand(msg, cartesian_cmd_names_, manipulator_cartesian_controls_);
      SendCartesianCommand(cmds);
    } else if (CheckIfPressed(msg, manipulator_joint_controls_)) {
      std::vector<double> cmds = CalculateCommand(msg, joint_names_, manipulator_joint_controls_);
      SendJointCommand(cmds);
    } else {
      StopManipulator();
    }

    ControlGripper(msg);
  }

  void StopManipulator()
  {
    SendCartesianCommand(std::vector<double>(6, 0.0));
    SendJointCommand(std::vector<double>(6, 0.0));
  }

  bool CheckIfPressed(
    const sensor_msgs::msg::Joy::SharedPtr msg,
    const std::map<std::string, std::unique_ptr<JoyControl>> & controls)
  {
    for (auto const & c : controls) {
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

  void SendCartesianCommand(const std::vector<double> & cmds)
  {
    geometry_msgs::msg::TwistStamped cartesian_cmd_msg;
    cartesian_cmd_msg.header.stamp = now();
    cartesian_cmd_msg.header.frame_id = cartesian_control_reference_frame_;
    cartesian_cmd_msg.twist.linear.x = cmds[0];
    cartesian_cmd_msg.twist.linear.y = cmds[1];
    cartesian_cmd_msg.twist.linear.z = cmds[2];
    cartesian_cmd_msg.twist.angular.x = cmds[3];
    cartesian_cmd_msg.twist.angular.y = cmds[4];
    cartesian_cmd_msg.twist.angular.z = cmds[5];

    twist_cmds_pub_->publish(cartesian_cmd_msg);
  }

  void SendJointCommand(const std::vector<double> & cmds)
  {
    control_msgs::msg::JointJog joint_cmd_msg;
    joint_cmd_msg.header.stamp = now();
    joint_cmd_msg.duration = 0.0;
    joint_cmd_msg.joint_names = joint_names_;
    joint_cmd_msg.velocities = cmds;
    joint_cmds_pub_->publish(joint_cmd_msg);
  }

  void ParseParameters()
  {
    this->declare_parameters<double>(
      "", {{"axis_deadzone", 0.05},
           {"joint_control_velocity", 0.5},
           {"cartesian_control_velocity_linear", 0.1},
           {"cartesian_control_velocity_angular", 0.4},
           {"gripper_control.open.position", 0.009},
           {"gripper_control.close.position", -0.009}});

    this->declare_parameter("cartesian_control_names", rclcpp::PARAMETER_STRING_ARRAY);
    this->declare_parameter("joint_names", rclcpp::PARAMETER_STRING_ARRAY);
    this->declare_parameter("home_joint_configuration", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter<std::vector<std::string>>(
      "gripper_joint_names", {"gripper_left_joint"});
    this->declare_parameters<std::string>(
      "",
      {{"end_effector_name", "end_effector_link"}, {"cartesian_control_reference_frame", "link2"}});

    axis_deadzone_ = this->get_parameter("axis_deadzone").as_double();
    joint_control_velocity_ = this->get_parameter("joint_control_velocity").as_double();
    cartesian_control_velocity_linear_ =
      this->get_parameter("cartesian_control_velocity_linear").as_double();
    cartesian_control_velocity_angular_ =
      this->get_parameter("cartesian_control_velocity_angular").as_double();
    opened_gripper_position_ = this->get_parameter("gripper_control.open.position").as_double();
    closed_gripper_position_ = this->get_parameter("gripper_control.close.position").as_double();
    end_effector_name_ = this->get_parameter("end_effector_name").as_string();
    cartesian_control_reference_frame_ =
      this->get_parameter("cartesian_control_reference_frame").as_string();
    gripper_joint_names_ = this->get_parameter("gripper_joint_names").as_string_array();

    // TODO check ParameterUninitializedException exception
    try {
      joint_names_ = this->get_parameter("joint_names").as_string_array();
      cartesian_control_names_ = this->get_parameter("cartesian_control_names").as_string_array();
      home_joint_configuration_ = this->get_parameter("home_joint_configuration").as_double_array();
    } catch (const rclcpp::exceptions::ParameterUninitializedException & e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Required parameter not defined: " << e.what());
      throw e;
    }

    for (const auto & joint_name : joint_names_) {
      std::string param_namespace = "joints_control." + joint_name;

      manipulator_joint_controls_[joint_name] =
        ParseJoyControl(param_namespace, joint_control_velocity_);
    }

    for (const auto & cartesian_control_name : cartesian_control_names_) {
      if (
        std::find(
          cartesian_cmd_names_.begin(), cartesian_cmd_names_.end(), cartesian_control_name) ==
        cartesian_cmd_names_.end()) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Unknown cartesian control type {cartesian_control_name},"
          " currently supported names: {self.cartesian_control_names}");
        continue;
      }
      std::string param_namespace = "cartesian_control." + cartesian_control_name;

      float scaling = 1.0;
      if (
        std::find(cartesian_cmd_names_.begin(), cartesian_cmd_names_.end(), "linear") !=
        cartesian_cmd_names_.end()) {
        scaling = cartesian_control_velocity_linear_;
      } else if (
        std::find(cartesian_cmd_names_.begin(), cartesian_cmd_names_.end(), "angular") !=
        cartesian_cmd_names_.end()) {
        scaling = cartesian_control_velocity_angular_;
      }

      manipulator_cartesian_controls_[cartesian_control_name] =
        ParseJoyControl(param_namespace, scaling);
    }
    gripper_open_ = ParseJoyControl("gripper_control.open");
    gripper_close_ = ParseJoyControl("gripper_control.close");
  }

  std::unique_ptr<JoyControl> ParseJoyControl(std::string param_namespace, float scaling = 1.0)
  {
    this->declare_parameter(param_namespace + ".control_type", "");
    std::string control_type = this->get_parameter(param_namespace + ".control_type").as_string();

    std::unique_ptr<JoyControl> controller;

    if (control_type == "double_button") {
      this->declare_parameter(param_namespace + ".positive_button_id", rclcpp::PARAMETER_INTEGER);
      this->declare_parameter(param_namespace + ".negative_button_id", rclcpp::PARAMETER_INTEGER);
      int positive_button_id =
        this->get_parameter(param_namespace + ".positive_button_id").as_int();
      int negative_button_id =
        this->get_parameter(param_namespace + ".negative_button_id").as_int();
      controller =
        std::make_unique<DoubleButtonControl>(positive_button_id, negative_button_id, scaling);
    } else if (control_type == "single_button") {
      this->declare_parameter(param_namespace + ".button_id", rclcpp::PARAMETER_INTEGER);
      int button_id = this->get_parameter(param_namespace + ".button_id").as_int();
      controller = std::make_unique<SingleButtonControl>(button_id, scaling);
    } else if (control_type == "axis") {
      this->declare_parameter(param_namespace + ".axis_id", rclcpp::PARAMETER_INTEGER);
      this->declare_parameter(param_namespace + ".inverted", false);
      int axis_id = this->get_parameter(param_namespace + ".axis_id").as_int();
      bool inverted = this->get_parameter(param_namespace + ".inverted").as_bool();
      controller = std::make_unique<AxisControl>(axis_id, axis_deadzone_, scaling, inverted);
    } else {
      RCLCPP_ERROR_STREAM(
        this->get_logger(), "Unknown control type "
                              << control_type << " in " << param_namespace
                              << ", currently supported types: single_button, double_button, axis. "
                                 "Please make sure that it is defined.");
      throw std::runtime_error("Unknown control type");
    }

    return controller;
  }

  double axis_deadzone_;
  double joint_control_velocity_;
  double cartesian_control_velocity_linear_;
  double cartesian_control_velocity_angular_;
  double opened_gripper_position_;
  double closed_gripper_position_;
  std::string end_effector_name_;
  std::string cartesian_control_reference_frame_;
  std::vector<std::string> gripper_joint_names_;
  std::vector<std::string> cartesian_control_names_;

  std::unique_ptr<JoyControl> dead_man_switch_;
  std::unique_ptr<JoyControl> home_manipulator_;
  std::unique_ptr<JoyControl> gripper_close_;
  std::unique_ptr<JoyControl> gripper_open_;
  std::map<std::string, std::unique_ptr<JoyControl>> manipulator_cartesian_controls_;
  std::map<std::string, std::unique_ptr<JoyControl>> manipulator_joint_controls_;
  std::vector<std::string> cartesian_cmd_names_;
  std::vector<std::string> joint_names_;
  std::vector<double> home_joint_configuration_;

  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_cmds_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmds_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  void ControlGripper(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (gripper_close_->IsPressed(msg)) {
      move_group_manipulator_->getNamedTargetValues("gripper").at(gripper_joint_names_[0]) =
        closed_gripper_position_;
      move_group_manipulator_->setJointValueTarget(
        move_group_manipulator_->getNamedTargetValues("gripper"));
      move_group_manipulator_->move();
    } else if (gripper_open_->IsPressed(msg)) {
      move_group_manipulator_->getNamedTargetValues("gripper").at(gripper_joint_names_[0]) =
        opened_gripper_position_;
      move_group_manipulator_->setJointValueTarget(
        move_group_manipulator_->getNamedTargetValues("gripper"));
      move_group_manipulator_->move();
    }
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
}  // namespace rosbot_xl_manipulation

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rosbot_xl_manipulation::JoyServoNode)