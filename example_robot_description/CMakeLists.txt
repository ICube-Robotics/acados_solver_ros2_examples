cmake_minimum_required(VERSION 3.5)
project(example_robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY rviz ros2_control urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
