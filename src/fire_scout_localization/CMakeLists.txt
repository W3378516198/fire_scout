cmake_minimum_required(VERSION 3.8)
project(fire_scout_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(px4_odom_adapter src/px4_odom_adapter.cpp)
ament_target_dependencies(
  px4_odom_adapter
  rclcpp
  px4_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

install(TARGETS px4_odom_adapter DESTINATION lib/${PROJECT_NAME})

ament_package()
