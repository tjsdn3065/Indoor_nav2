cmake_minimum_required(VERSION 3.8)
project(indoor_nav2_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config maps rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/build_voronoi_roadmap.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
