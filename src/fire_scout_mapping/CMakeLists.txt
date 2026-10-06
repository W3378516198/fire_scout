cmake_minimum_required(VERSION 3.8)

project(fire_scout_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)


add_executable(
  radar_map_builder
  src/radar_map_builder.cpp
)

ament_target_dependencies(
  radar_map_builder

  rclcpp
  sensor_msgs

  tf2
  tf2_ros
  tf2_sensor_msgs
)


install(
  TARGETS
  radar_map_builder

  DESTINATION
  lib/${PROJECT_NAME}
)


install(
  DIRECTORY
  launch

  DESTINATION
  share/${PROJECT_NAME}
)


ament_package()