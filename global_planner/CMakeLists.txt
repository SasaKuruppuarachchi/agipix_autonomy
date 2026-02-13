cmake_minimum_required(VERSION 3.5)
project(global_planner)

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
find_package(octomap_msgs REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(octomap REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(map_manager REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/globalPlannerLib.cpp
  include/${PROJECT_NAME}/PRMKDTree.cpp
  include/${PROJECT_NAME}/dep.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  octomap_msgs
  octomap_ros
  tf2
  tf2_geometry_msgs
  map_manager
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${OCTOMAP_LIBRARIES}
)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY
  INTERFACE_LINK_LIBRARIES "${OpenCV_LIBS};${OCTOMAP_LIBRARIES}"
)

add_executable(rrt_interactive_node src/rrt_interactive_node.cpp)
add_executable(rrt_star_interactive_node src/rrt_star_interactive_node.cpp)
add_executable(test_dep_node src/test_dep_node.cpp)

# add_executable(rrt_interactive_node src/rrt_interactive_node.cpp)
# target_link_libraries(rrt_interactive_node ${PROJECT_NAME}_lib map_manager::map_manager_lib ${octomap_LIBRARIES})
# ament_target_dependencies(rrt_interactive_node ${PROJECT_DEPENDENCIES})

# add_executable(rrt_star_interactive_node src/rrt_star_interactive_node.cpp)
# target_link_libraries(rrt_star_interactive_node ${PROJECT_NAME}_lib map_manager::map_manager_lib ${octomap_LIBRARIES})
# ament_target_dependencies(rrt_star_interactive_node ${PROJECT_DEPENDENCIES})

# add_executable(test_dep_node src/test_dep_node.cpp)
# target_link_libraries(test_dep_node ${PROJECT_NAME}_lib map_manager::map_manager_lib ${octomap_LIBRARIES})
# ament_target_dependencies(test_dep_node ${PROJECT_DEPENDENCIES})

ament_target_dependencies(rrt_interactive_node
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
)
ament_target_dependencies(rrt_star_interactive_node
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
)
ament_target_dependencies(test_dep_node
  rclcpp
  nav_msgs
  geometry_msgs
)

target_link_libraries(rrt_interactive_node ${PROJECT_NAME})
target_link_libraries(rrt_star_interactive_node ${PROJECT_NAME})
target_link_libraries(test_dep_node ${PROJECT_NAME})

target_link_libraries(rrt_interactive_node ${OCTOMAP_LIBRARIES})
target_link_libraries(rrt_star_interactive_node ${OCTOMAP_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  rrt_interactive_node
  rrt_star_interactive_node
  test_dep_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  rviz
  cfg
  map
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(OpenCV)

ament_package()
