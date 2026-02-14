cmake_minimum_required(VERSION 3.8)
project(px4_control_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(tracking_controller REQUIRED)

include_directories(include ${Eigen3_INCLUDE_DIRS})

add_executable(px4_tracking_mode_node
  src/agipix_tracking_mode_main.cpp
)

target_link_libraries(px4_tracking_mode_node
  Eigen3::Eigen
  px4_ros2_cpp::px4_ros2_cpp
  rclcpp::rclcpp
)

ament_target_dependencies(px4_tracking_mode_node
  rclcpp
  px4_ros2_cpp
  tracking_controller
)

install(TARGETS
  px4_tracking_mode_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
