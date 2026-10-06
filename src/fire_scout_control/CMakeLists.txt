cmake_minimum_required(VERSION 3.8)

project(fire_scout_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(
  scout_takeoff
  src/scout_takeoff.cpp
)

ament_target_dependencies(
  scout_takeoff
  rclcpp
  px4_msgs
)


add_executable(
  scout_enter_window
  src/scout_enter_window.cpp
)

ament_target_dependencies(
  scout_enter_window
  rclcpp
  px4_msgs
)

add_executable(
 entrance_wall_search
 src/entrance_wall_search.cpp
)


ament_target_dependencies(
 entrance_wall_search
 rclcpp
 sensor_msgs
 px4_msgs
)

install(
  TARGETS
    scout_takeoff
    scout_enter_window
    entrance_wall_search
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()