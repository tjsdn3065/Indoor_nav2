cmake_minimum_required(VERSION 3.8)
project(indoor_nav2_rtabmap)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY databases
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".gitignore" EXCLUDE
)

ament_package()
