cmake_minimum_required(VERSION 3.16)
project(ma_demos_launch_helpers LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_core REQUIRED)
find_package(backward_ros REQUIRED)


ament_python_install_package(ma_demos_launch_helpers
  SCRIPTS_DESTINATION lib/ma_demos_launch_helpers
)

ament_package()
