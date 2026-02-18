cmake_minimum_required(VERSION 3.5)
project(agi_viz)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
