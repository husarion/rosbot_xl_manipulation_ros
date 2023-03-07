#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>

#include <sensor_msgs/msg/joy.hpp>

#include <rosbot_xl_manipulation_moveit/joy_control.hpp>
#include <rosbot_xl_manipulation_moveit/manipulation_controller.hpp>

namespace rosbot_xl_manipulation
{
class JoyServoNode : public rclcpp::Node
{
public:
  JoyServoNode(const rclcpp::NodeOptions & options);

private:
  void JoyCb(const sensor_msgs::msg::Joy::SharedPtr msg);

  void InitializeControllers();
  void ProcessControllers(
    const sensor_msgs::msg::Joy::SharedPtr msg,
    const std::vector<std::unique_ptr<ManipulationController>> & controllers);
  void StopControllers(const std::vector<std::unique_ptr<ManipulationController>> & controllers);

  void StartServo();
  void ChangeCartesianDriftDimensions();

  std::vector<std::unique_ptr<ManipulationController>> manipulator_controllers_;
  std::vector<std::unique_ptr<ManipulationController>> gripper_controllers_;

  bool controllers_initialized_ = false;

  std::unique_ptr<JoyControl> dead_man_switch_;
  
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};
}  // namespace rosbot_xl_manipulation