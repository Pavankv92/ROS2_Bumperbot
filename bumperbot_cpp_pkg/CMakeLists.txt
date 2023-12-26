cmake_minimum_required(VERSION 3.8)
project(bumperbot_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(turtlesim REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(bumperbot_msgs REQUIRED)


include_directories(include)

set(dependencies
rclcpp
rcl_interfaces
turtlesim
tf2_ros
geometry_msgs
bumperbot_msgs
tf2
)

add_executable(simple_parameter src/simple_parameter.cpp)
ament_target_dependencies(simple_parameter ${dependencies})

add_executable(turtlesim_kinematics src/turtlesim_kinematics.cpp)
ament_target_dependencies(turtlesim_kinematics ${dependencies})

add_executable(simple_tf_kinematics src/simple_tf_kinematics.cpp)
ament_target_dependencies(simple_tf_kinematics ${dependencies})

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS 
simple_parameter
turtlesim_kinematics
simple_tf_kinematics

DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})
ament_package()
