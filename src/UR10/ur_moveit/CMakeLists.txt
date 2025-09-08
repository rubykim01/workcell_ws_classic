cmake_minimum_required(VERSION 3.5)
project(ur_moveit)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY config launch rviz srdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
