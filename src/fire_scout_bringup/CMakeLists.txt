cmake_minimum_required(VERSION 3.8)
project(fire_scout_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch docs DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS
    scripts/start_radar_bridge.sh
    scripts/diagnose_multi_tf.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
