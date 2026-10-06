cmake_minimum_required(VERSION 3.8)
project(fire_scout_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(mmwave_enricher src/mmwave_enricher.cpp)
ament_target_dependencies(
  mmwave_enricher
  rclcpp
  sensor_msgs
  nav_msgs
)

add_executable(ultrasonic_to_px4
    src/ultrasonic_to_px4.cpp
)


ament_target_dependencies(
    ultrasonic_to_px4
    rclcpp
    sensor_msgs
    px4_msgs
)



install(TARGETS mmwave_enricher ultrasonic_to_px4 DESTINATION lib/${PROJECT_NAME})

ament_package()
