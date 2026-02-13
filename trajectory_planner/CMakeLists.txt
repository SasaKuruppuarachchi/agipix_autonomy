cmake_minimum_required(VERSION 3.8)
project(trajectory_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(global_planner REQUIRED)
find_package(map_manager REQUIRED)

include_directories(include)

add_library(
  ${PROJECT_NAME}
  include/${PROJECT_NAME}/path_search/astarOcc.cpp
  include/${PROJECT_NAME}/polyTrajSolver.cpp
  include/${PROJECT_NAME}/polyTrajOctomap.cpp
  include/${PROJECT_NAME}/piecewiseLinearTraj.cpp
  include/${PROJECT_NAME}/bspline.cpp
  include/${PROJECT_NAME}/bsplineTraj.cpp
  include/${PROJECT_NAME}/polyTrajOccMap.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  octomap
  octomap_msgs
  tf2
  tf2_geometry_msgs
  global_planner
  map_manager
  Eigen3
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  message("Detected x86_64 architecture.")
  target_link_libraries(${PROJECT_NAME}
    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/third_party/lib/x86/libosqp.so
    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/third_party/lib/x86/libOsqpEigen.so.0.7.0
  )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
  message("Detected ARM architecture.")
  target_link_libraries(${PROJECT_NAME}
    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/third_party/lib/ARM/libosqp.so
    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/third_party/lib/ARM/libOsqpEigen.so.0.8.0
  )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message("Detected ARM64 architecture.")
  target_link_libraries(${PROJECT_NAME}
    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/third_party/lib/ARM/libosqp.so
    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/third_party/lib/ARM/libOsqpEigen.so.0.8.0
  )
else()
  message("Unknown architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

add_executable(poly_RRT_node src/poly_RRT_node.cpp)
add_executable(poly_RRTStar_node src/poly_RRTStar_node.cpp)
add_executable(poly_RRT_goal_node src/poly_RRT_goal_node.cpp)
add_executable(poly_RRTStar_goal_node src/poly_RRTStar_goal_node.cpp)
add_executable(bspline_node src/bspline_node.cpp)
add_executable(testBspline src/test/testBspline.cpp)
add_executable(testTrajSolver src/test/testTrajSolver.cpp)
add_executable(testEigen src/test/testEigen.cpp)
add_executable(testBsplineFit src/test/testBsplineFit.cpp)

foreach(target poly_RRT_node poly_RRTStar_node poly_RRT_goal_node poly_RRTStar_goal_node bspline_node)
  target_link_libraries(${target} ${PROJECT_NAME})
  ament_target_dependencies(${target}
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    octomap
    octomap_msgs
    tf2
    tf2_geometry_msgs
    global_planner
    map_manager
  )
endforeach()

foreach(target testBspline testTrajSolver testEigen testBsplineFit)
  target_link_libraries(${target} ${PROJECT_NAME})
  ament_target_dependencies(${target}
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    octomap
    octomap_msgs
    tf2
    tf2_geometry_msgs
    global_planner
    map_manager
  )
endforeach()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  poly_RRT_node
  poly_RRTStar_node
  poly_RRT_goal_node
  poly_RRTStar_goal_node
  bspline_node
  testBspline
  testTrajSolver
  testEigen
  testBsplineFit
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  cfg
  rviz
  map
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY INTERFACE_LINK_LIBRARIES
    "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/third_party/lib/x86/libosqp.so;${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/third_party/lib/x86/libOsqpEigen.so.0.7.0"
  )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
  set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY INTERFACE_LINK_LIBRARIES
    "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/third_party/lib/ARM/libosqp.so;${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/third_party/lib/ARM/libOsqpEigen.so.0.8.0"
  )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY INTERFACE_LINK_LIBRARIES
    "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/third_party/lib/ARM/libosqp.so;${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/third_party/lib/ARM/libOsqpEigen.so.0.8.0"
  )
endif()
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  octomap
  octomap_msgs
  tf2
  tf2_geometry_msgs
  global_planner
  map_manager
  Eigen3
)

ament_package()
