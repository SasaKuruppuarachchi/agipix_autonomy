cmake_minimum_required(VERSION 3.5)
project(autonomous_flight)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(onboard_detector REQUIRED)
find_package(global_planner REQUIRED)
find_package(trajectory_planner REQUIRED)
find_package(map_manager REQUIRED)
find_package(time_optimizer REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(px4_ros2_cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Target.msg"
  DEPENDENCIES std_msgs geometry_msgs
)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(EXTRA_LINK_LIBS "")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  message("Detected x86_64 architecture.")
  list(APPEND EXTRA_LINK_LIBS
    ${CMAKE_CURRENT_SOURCE_DIR}/../trajectory_planner/include/trajectory_planner/third_party/lib/x86/libosqp.so
    ${CMAKE_CURRENT_SOURCE_DIR}/../trajectory_planner/include/trajectory_planner/third_party/lib/x86/libOsqpEigen.so.0.7.0
    ${CMAKE_CURRENT_SOURCE_DIR}/../time_optimizer/include/time_optimizer/third_party/lib/x86/libmosek64.so
  )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
  message("Detected ARM architecture.")
  list(APPEND EXTRA_LINK_LIBS
    ${CMAKE_CURRENT_SOURCE_DIR}/../trajectory_planner/include/trajectory_planner/third_party/lib/ARM/libosqp.so
    ${CMAKE_CURRENT_SOURCE_DIR}/../trajectory_planner/include/trajectory_planner/third_party/lib/ARM/libOsqpEigen.so.0.8.0
    ${CMAKE_CURRENT_SOURCE_DIR}/../time_optimizer/include/time_optimizer/third_party/lib/ARM/libmosek64.so
  )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message("Detected ARM64 architecture.")
  list(APPEND EXTRA_LINK_LIBS
    ${CMAKE_CURRENT_SOURCE_DIR}/../trajectory_planner/include/trajectory_planner/third_party/lib/ARM/libosqp.so
    ${CMAKE_CURRENT_SOURCE_DIR}/../trajectory_planner/include/trajectory_planner/third_party/lib/ARM/libOsqpEigen.so.0.8.0
    ${CMAKE_CURRENT_SOURCE_DIR}/../time_optimizer/include/time_optimizer/third_party/lib/ARM/libmosek64.so
  )
else()
  message("Unknown architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(AUTONOMOUS_NODE_EXTRA_LIBS
  ${global_planner_LIBRARIES}
  ${trajectory_planner_LIBRARIES}
  ${time_optimizer_LIBRARIES}
  ${OpenCV_LIBS}
  ${EXTRA_LINK_LIBS}
)

set(AUTONOMOUS_FLIGHT_LIB ${PROJECT_NAME}_core)

add_library(${AUTONOMOUS_FLIGHT_LIB}
  include/${PROJECT_NAME}/px4/flightBase.cpp
  include/${PROJECT_NAME}/px4/inspection.cpp
  include/${PROJECT_NAME}/px4/navigation.cpp
  include/${PROJECT_NAME}/px4/dynamicNavigation.cpp
  include/${PROJECT_NAME}/px4/dynamicInspection.cpp
  include/${PROJECT_NAME}/px4/dynamicExploration.cpp
)

ament_target_dependencies(${AUTONOMOUS_FLIGHT_LIB}
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  octomap
  octomap_ros
  octomap_msgs
  onboard_detector
  global_planner
  trajectory_planner
  map_manager
  time_optimizer
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(${AUTONOMOUS_FLIGHT_LIB} "${cpp_typesupport_target}")

target_link_libraries(${AUTONOMOUS_FLIGHT_LIB}
  ${OCTOMAP_LIBRARIES}
  ${OpenCV_LIBS}
  ${EXTRA_LINK_LIBS}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/autonomous_flight_node.cpp)
add_executable(takeoff_and_hover_node src/px4/takeoff_and_hover_node.cpp)
add_executable(takeoff_and_hover_executor_node src/px4/takeoff_and_hover_executor_node.cpp)
add_executable(takeoff_and_track_circle_node src/px4/takeoff_and_track_circle_node.cpp)
add_executable(takeoff_and_track_circle_executor_node src/px4/takeoff_and_track_circle_executor_node.cpp)
add_executable(mission_tracking_executor_node src/px4/mission_tracking_executor_node.cpp)
add_executable(inspection_node src/px4/inspection_node.cpp)
add_executable(navigation_node src/px4/navigation_node.cpp)
add_executable(dynamic_navigation_node src/px4/dynamic_navigation_node.cpp)
add_executable(dynamic_inspection_node src/px4/dynamic_inspection_node.cpp)
add_executable(dynamic_exploration_node src/px4/dynamic_exploration_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(takeoff_and_hover_node ${AUTONOMOUS_FLIGHT_LIB} ${AUTONOMOUS_NODE_EXTRA_LIBS})
target_link_libraries(takeoff_and_hover_executor_node
  Eigen3::Eigen
  px4_ros2_cpp::px4_ros2_cpp
  rclcpp::rclcpp
)
target_link_libraries(takeoff_and_track_circle_node ${AUTONOMOUS_FLIGHT_LIB} ${AUTONOMOUS_NODE_EXTRA_LIBS})
target_link_libraries(takeoff_and_track_circle_executor_node
  Eigen3::Eigen
  px4_ros2_cpp::px4_ros2_cpp
  rclcpp::rclcpp
)
target_link_libraries(mission_tracking_executor_node
  Eigen3::Eigen
  px4_ros2_cpp::px4_ros2_cpp
  rclcpp::rclcpp
)
target_link_libraries(inspection_node ${AUTONOMOUS_FLIGHT_LIB} ${AUTONOMOUS_NODE_EXTRA_LIBS})
target_link_libraries(navigation_node ${AUTONOMOUS_FLIGHT_LIB} ${AUTONOMOUS_NODE_EXTRA_LIBS})
target_link_libraries(dynamic_navigation_node ${AUTONOMOUS_FLIGHT_LIB} ${AUTONOMOUS_NODE_EXTRA_LIBS})
target_link_libraries(dynamic_inspection_node ${AUTONOMOUS_FLIGHT_LIB} ${AUTONOMOUS_NODE_EXTRA_LIBS})
target_link_libraries(dynamic_exploration_node ${AUTONOMOUS_FLIGHT_LIB} ${AUTONOMOUS_NODE_EXTRA_LIBS})

ament_target_dependencies(takeoff_and_hover_node rclcpp)
ament_target_dependencies(takeoff_and_hover_executor_node rclcpp px4_ros2_cpp visualization_msgs)
ament_target_dependencies(takeoff_and_track_circle_node rclcpp)
ament_target_dependencies(takeoff_and_track_circle_executor_node rclcpp px4_ros2_cpp visualization_msgs)
ament_target_dependencies(mission_tracking_executor_node rclcpp px4_ros2_cpp visualization_msgs)
ament_target_dependencies(inspection_node rclcpp)
ament_target_dependencies(navigation_node rclcpp)
ament_target_dependencies(dynamic_navigation_node rclcpp)
ament_target_dependencies(dynamic_inspection_node rclcpp)
ament_target_dependencies(dynamic_exploration_node rclcpp)
target_link_libraries(takeoff_and_hover_node "${cpp_typesupport_target}")
target_link_libraries(takeoff_and_hover_executor_node "${cpp_typesupport_target}")
target_link_libraries(takeoff_and_track_circle_node "${cpp_typesupport_target}")
target_link_libraries(takeoff_and_track_circle_executor_node "${cpp_typesupport_target}")
target_link_libraries(mission_tracking_executor_node "${cpp_typesupport_target}")
target_link_libraries(inspection_node "${cpp_typesupport_target}")
target_link_libraries(navigation_node "${cpp_typesupport_target}")
target_link_libraries(dynamic_navigation_node "${cpp_typesupport_target}")
target_link_libraries(dynamic_inspection_node "${cpp_typesupport_target}")
target_link_libraries(dynamic_exploration_node "${cpp_typesupport_target}")


install(TARGETS ${AUTONOMOUS_FLIGHT_LIB}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  takeoff_and_hover_node
  takeoff_and_hover_executor_node
  takeoff_and_track_circle_node
  takeoff_and_track_circle_executor_node
  mission_tracking_executor_node
  inspection_node
  navigation_node
  dynamic_navigation_node
  dynamic_inspection_node
  dynamic_exploration_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${AUTONOMOUS_FLIGHT_LIB})
ament_export_dependencies(rosidl_default_runtime)

ament_package()
