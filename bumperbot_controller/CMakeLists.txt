cmake_minimum_required(VERSION 3.8)
project(bumperbot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

ament_python_install_package(${PROJECT_NAME})
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

set(dependencies
rclcpp
geometry_msgs
std_msgs
${Eigen_LIBRARIES}
)

add_executable(simple_controller src/simple_controller.cpp)
ament_target_dependencies(simple_controller ${dependencies})

install(
DIRECTORY include
DESTINATION include
)

install(TARGETS
simple_controller

DESTINATION lib/${PROJECT_NAME}/
)


install(PROGRAMS
${PROJECT_NAME}/simple_controller.py
DESTINATION lib/${PROJECT_NAME}/
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_export_dependencies(${dependencies})
ament_package()