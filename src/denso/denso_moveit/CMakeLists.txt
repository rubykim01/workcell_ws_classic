cmake_minimum_required(VERSION 3.8)
project(denso_moveit)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config rviz srdf robot
  DESTINATION share/${PROJECT_NAME})

ament_package()
