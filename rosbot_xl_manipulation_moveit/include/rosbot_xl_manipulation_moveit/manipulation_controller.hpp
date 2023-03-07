#ifndef ROSBOT_XL_MANIPULATION_MOVEIT_MOVEIT_CONTROLLERS_H_
#define ROSBOT_XL_MANIPULATION_MOVEIT_MOVEIT_CONTROLLERS_H_

#include <rclcpp/rclcpp.hpp>

#include <control_msgs/msg/joint_jog.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <rosbot_xl_manipulation_moveit/joy_control.hpp>

namespace rosbot_xl_manipulation
{

class ManipulationController
{
public:
  /**
   * @brief Checks if button/axis was activated, if so send a command
   * 
   * @returns true if button/axis was activated
   */
  virtual bool Process(const sensor_msgs::msg::Joy::SharedPtr msg) = 0;
  virtual void Stop() = 0;

protected:
  bool CheckIfPressed(
    const sensor_msgs::msg::Joy::SharedPtr msg,
    const std::map<std::string, std::unique_ptr<JoyControl>> & controls);

  std::vector<double> CalculateCommand(
    const sensor_msgs::msg::Joy::SharedPtr msg, const std::vector<std::string> & cmd_names,
    const std::map<std::string, std::unique_ptr<JoyControl>> & controls);
};

class JointController : public ManipulationController
{
public:
  JointController(const rclcpp::Node::SharedPtr & node);
  bool Process(const sensor_msgs::msg::Joy::SharedPtr msg) override;
  void Stop() override;

private:
  void ParseParameters(const rclcpp::Node::SharedPtr & node);

  void SendJointCommand(
    const std::vector<double> & cmds, const builtin_interfaces::msg::Time & timestamp);

  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_cmds_pub_;

  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_itf_;

  std::map<std::string, std::unique_ptr<JoyControl>> manipulator_joint_controls_;

  std::vector<std::string> joint_names_;
  double joint_control_velocity_;
};

class CartesianController : public ManipulationController
{
public:
  CartesianController(const rclcpp::Node::SharedPtr & node);
  bool Process(const sensor_msgs::msg::Joy::SharedPtr msg) override;
  void Stop() override;

private:
  void ParseParameters(const rclcpp::Node::SharedPtr & node);

  void SendCartesianCommand(
    const std::vector<double> & cmds, const builtin_interfaces::msg::Time & timestamp);

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmds_pub_;

  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_itf_;

  std::map<std::string, std::unique_ptr<JoyControl>> manipulator_cartesian_controls_;

  std::vector<std::string> cartesian_control_names_;
  std::vector<std::string> cartesian_cmd_names_;
  std::string cartesian_control_reference_frame_;
  double cartesian_control_velocity_linear_;
  double cartesian_control_velocity_angular_;
};

class ManipulatorMoveGroupController : public ManipulationController
{
public:
  ManipulatorMoveGroupController(const rclcpp::Node::SharedPtr & node);
  bool Process(const sensor_msgs::msg::Joy::SharedPtr msg) override;
  void Stop() override {}

private:
  void ParseParameters(const rclcpp::Node::SharedPtr & node);

  void MoveToHome();

  moveit::planning_interface::MoveGroupInterfacePtr move_group_manipulator_;

  std::unique_ptr<JoyControl> home_manipulator_;

  // action of this controller should be triggered only once per button press
  //  and require releasing button before executing again
  bool action_already_executed_ = false;
};

class GripperMoveGroupController : public ManipulationController
{
public:
  GripperMoveGroupController(const rclcpp::Node::SharedPtr & node);
  bool Process(const sensor_msgs::msg::Joy::SharedPtr msg) override;
  void Stop() override {}

private:
  void ParseParameters(const rclcpp::Node::SharedPtr & node);

  void CloseGripper();
  void OpenGripper();

  moveit::planning_interface::MoveGroupInterfacePtr move_group_gripper_;

  std::unique_ptr<JoyControl> toggle_gripper_position_;

  enum GripperPosition {
    OPENED,
    CLOSED
  };
  GripperPosition gripper_position_ = GripperPosition::CLOSED;

  // action of this controller should be triggered only once per button press
  //  and require releasing button before executing again
  bool action_already_executed_ = false;
};

}  // namespace rosbot_xl_manipulation

#endif