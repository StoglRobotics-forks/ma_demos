cmake_minimum_required(VERSION 3.5)
project(kuka_kr16_2_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY ros2_control rviz_config urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
