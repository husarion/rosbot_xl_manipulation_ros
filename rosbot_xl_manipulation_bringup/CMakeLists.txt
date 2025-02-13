cmake_minimum_required(VERSION 3.11)
project(rosbot_xl_manipulation_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()