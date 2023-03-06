#ifndef MOVEIT_CONTROLLERS_H
#define MOVEIT_CONTROLLERS_H

#include <rclcpp/rclcpp.hpp>

#include <control_msgs/msg/joint_jog.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <rosbot_xl_manipulation_moveit/JoyControl.h>

namespace rosbot_xl_manipulation
{

class Controller
{
public:
  virtual void Stop() = 0;
  virtual bool Process(const sensor_msgs::msg::Joy::SharedPtr msg) = 0;

protected:
  bool CheckIfPressed(
    const sensor_msgs::msg::Joy::SharedPtr msg,
    const std::map<std::string, std::unique_ptr<JoyControl>> & controls);

  std::vector<double> CalculateCommand(
    const sensor_msgs::msg::Joy::SharedPtr msg, const std::vector<std::string> & cmd_names,
    const std::map<std::string, std::unique_ptr<JoyControl>> & controls);
};

class JointController : public Controller
{
public:
  JointController(const rclcpp::Node::SharedPtr & node);
  bool Process(const sensor_msgs::msg::Joy::SharedPtr msg) override;
  void Stop() override;

private:
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_cmds_pub_;

  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_itf_;

  std::map<std::string, std::unique_ptr<JoyControl>> manipulator_joint_controls_;

  std::vector<std::string> joint_names_;
  double joint_control_velocity_;

  void ParseParameters(const rclcpp::Node::SharedPtr & node);

  void SendJointCommand(
    const std::vector<double> & cmds, const builtin_interfaces::msg::Time & timestamp);
};

class CartesianController : public Controller
{
public:
  CartesianController(const rclcpp::Node::SharedPtr & node);

  bool Process(const sensor_msgs::msg::Joy::SharedPtr msg) override;

  void Stop() override;

private:
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmds_pub_;

  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_itf_;

  std::map<std::string, std::unique_ptr<JoyControl>> manipulator_cartesian_controls_;

  std::vector<std::string> cartesian_control_names_;
  std::vector<std::string> cartesian_cmd_names_;
  std::string cartesian_control_reference_frame_;
  double cartesian_control_velocity_linear_;
  double cartesian_control_velocity_angular_;

  void ParseParameters(const rclcpp::Node::SharedPtr & node);

  void SendCartesianCommand(
    const std::vector<double> & cmds, const builtin_interfaces::msg::Time & timestamp);
};

class MoveGroupManipulatorController : public Controller
{
public:
  MoveGroupManipulatorController(const rclcpp::Node::SharedPtr & node);

  bool Process(const sensor_msgs::msg::Joy::SharedPtr msg) override;

  void Stop() override {}

private:
  moveit::planning_interface::MoveGroupInterfacePtr move_group_manipulator_;

  std::unique_ptr<JoyControl> home_manipulator_;

  void ParseParameters(const rclcpp::Node::SharedPtr & node);

  void MoveToHome();
};

class MoveGroupGripperController : public Controller
{
public:
  MoveGroupGripperController(const rclcpp::Node::SharedPtr & node);

  bool Process(const sensor_msgs::msg::Joy::SharedPtr msg) override;

  void Stop() override {}

private:
  moveit::planning_interface::MoveGroupInterfacePtr move_group_gripper_;

  std::unique_ptr<JoyControl> gripper_close_;
  std::unique_ptr<JoyControl> gripper_open_;

  void ParseParameters(const rclcpp::Node::SharedPtr & node);

  void CloseGripper();

  void OpenGripper();
};

}  // namespace rosbot_xl_manipulation

#endif