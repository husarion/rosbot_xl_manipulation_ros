cmake_minimum_required(VERSION 3.10.2)
project(rosbot_xl_manipulation_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
