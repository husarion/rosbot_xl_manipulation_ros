cmake_minimum_required(VERSION 3.10.2)
project(rosbot_xl_manipulation_moveit)

set(PACKAGE_INCLUDE_DEPENDS
  control_msgs
  geometry_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_servo
)

find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
  include
)

add_library(joy_servo SHARED 
  src/joy_servo_node.cpp
  src/joy_control.cpp
  src/manipulation_controller.cpp)
ament_target_dependencies(joy_servo ${PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(joy_servo "rosbot_xl_manipulation::JoyServoNode")
rclcpp_components_register_node(joy_servo
  PLUGIN "rosbot_xl_manipulation::JoyServoNode"
  EXECUTABLE joy_servo_node
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

install(
  TARGETS joy_servo
  LIBRARY DESTINATION lib
)

ament_package()