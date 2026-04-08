cmake_minimum_required(VERSION 3.8)
project(onboard_detector)

# Set C++ standard and compile options
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add compile options
add_compile_options(-O3 -Wall)

# Prevent unintended MPI discovery from dependencies/environment (we don't use MPI)
# set(CMAKE_DISABLE_FIND_PACKAGE_MPI ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Threads REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDynamicObstacles.srv"
   DEPENDENCIES geometry_msgs
)

# Package library
add_library(${PROJECT_NAME}_lib SHARED
  include/${PROJECT_NAME}/detectors/dbscan.cpp
  #include/${PROJECT_NAME}/detectors/fakeDetector.cpp
  include/${PROJECT_NAME}/detectors/uvDetector.cpp
  include/${PROJECT_NAME}/tracking/kalmanFilter.cpp
  include/${PROJECT_NAME}/dynamicDetector.cpp
  include/${PROJECT_NAME}/detectors/lidarDetector.cpp
)
target_include_directories(${PROJECT_NAME}_lib
    PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
set(dependencies "ament_cmake" "rclcpp" "visualization_msgs" "sensor_msgs"
                 "cv_bridge" "tf2" "tf2_ros" "tf2_geometry_msgs" "geometry_msgs" "message_filters"
                 "nav_msgs" "vision_msgs" "PCL" "pcl_conversions")
ament_target_dependencies(${PROJECT_NAME}_lib ${dependencies})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_lib "${cpp_typesupport_target}")


# Nodes
add_executable(dynamic_detector_node src/dynamic_detector_node.cpp)
target_link_libraries(dynamic_detector_node ${PROJECT_NAME}_lib)

# # YOLOs-CPP integration (header-only API + ONNX Runtime)
# set(YOLOS_CPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../YOLOs-CPP" CACHE PATH "Path to YOLOs-CPP root")
# if(NOT EXISTS "${YOLOS_CPP_DIR}/include/yolos/tasks/detection.hpp")
#   message(FATAL_ERROR "YOLOs-CPP headers not found under ${YOLOS_CPP_DIR}. Set -DYOLOS_CPP_DIR=<path-to-YOLOs-CPP>.")
# endif()

# #set(ONNXRUNTIME_DIR "/workspaces/agipix_control/src/YOLOs-CPP/onnxruntime-linux-x64-gpu-1.20.1" CACHE PATH "Path to ONNX Runtime include directory")

# if(NOT DEFINED ONNXRUNTIME_DIR)
#   set(
#     ONNXRUNTIME_DIR
#     "${YOLOS_CPP_DIR}/onnxruntime-linux-x64-gpu-1.20.1"
#     CACHE PATH "Path to ONNX Runtime root (contains include/ and lib/)"
#   )
# endif()

# set(ONNXRUNTIME_INCLUDE_DIR "")
# set(ONNXRUNTIME_LIBRARY "")

# if(DEFINED ONNXRUNTIME_DIR AND EXISTS "${ONNXRUNTIME_DIR}/include/onnxruntime_cxx_api.h")
#   set(ONNXRUNTIME_INCLUDE_DIR "${ONNXRUNTIME_DIR}/include")
#   if(EXISTS "${ONNXRUNTIME_DIR}/lib/libonnxruntime.so")
#     # Prefer explicit path to avoid stale CMake cache/find behavior.
#     set(ONNXRUNTIME_LIBRARY "${ONNXRUNTIME_DIR}/lib/libonnxruntime.so")
#   else()
#     unset(ONNXRUNTIME_LIBRARY CACHE)
#     find_library(ONNXRUNTIME_LIBRARY
#       NAMES onnxruntime
#       PATHS "${ONNXRUNTIME_DIR}/lib"
#       NO_DEFAULT_PATH
#     )
#   endif()
# endif()

# if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
#   unset(ONNXRUNTIME_INCLUDE_DIR CACHE)
#   unset(ONNXRUNTIME_LIBRARY CACHE)
#   find_path(ONNXRUNTIME_INCLUDE_DIR
#     NAMES onnxruntime_cxx_api.h
#     PATHS /usr/include /usr/local/include /opt/onnxruntime/include
#     PATH_SUFFIXES onnxruntime
#   )
#   find_library(ONNXRUNTIME_LIBRARY
#     NAMES onnxruntime
#     PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu /opt/onnxruntime/lib
#   )
# endif()

# if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
#   message(FATAL_ERROR
#     "ONNX Runtime not found. Provide -DONNXRUNTIME_DIR=/path/to/onnxruntime "
#     "(must contain include/onnxruntime_cxx_api.h and lib/libonnxruntime.so), "
#     "or install system ONNX Runtime development files.")
# endif()

# message(STATUS "YOLOs-CPP dir: ${YOLOS_CPP_DIR}")
# message(STATUS "ONNX Runtime include: ${ONNXRUNTIME_INCLUDE_DIR}")
# message(STATUS "ONNX Runtime library: ${ONNXRUNTIME_LIBRARY}")

# add_executable(yolo_detector_node src/yolo_detector_node.cpp)
# target_include_directories(yolo_detector_node PRIVATE
#   "${YOLOS_CPP_DIR}/include"
#   "${ONNXRUNTIME_INCLUDE_DIR}"
# )
# target_link_libraries(yolo_detector_node
#   "${ONNXRUNTIME_LIBRARY}"
#   Threads::Threads
# )
# if(DEFINED ONNXRUNTIME_DIR AND UNIX AND NOT APPLE)
#   set_target_properties(yolo_detector_node PROPERTIES
#     BUILD_RPATH "${ONNXRUNTIME_DIR}/lib"
#     INSTALL_RPATH "${ONNXRUNTIME_DIR}/lib"
#   )
# endif()
# ament_target_dependencies(yolo_detector_node
#   rclcpp
#   sensor_msgs
#   vision_msgs
#   cv_bridge
#   std_msgs
#   ament_index_cpp
# )


# headers for client packages
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# library for client packages
install(TARGETS ${PROJECT_NAME}_lib
  EXPORT ${PROJECT_NAME}_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# install executables
install(TARGETS
  dynamic_detector_node
  #yolo_detector_node
  DESTINATION lib/${PROJECT_NAME})

# install config
install(
  DIRECTORY cfg/
  DESTINATION share/${PROJECT_NAME}/cfg
)

# install launch
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install Python executables
install(PROGRAMS
  scripts/yolo_detector.py
  scripts/yolov11_detector_node.py
  scripts/yolov11_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install YOLO python modules and resources so imports like `utils.tool` and `module.detector` work
install(DIRECTORY
  scripts/utils
  scripts/module
  scripts/config
  scripts/weights
  DESTINATION lib/${PROJECT_NAME}
)

set(YOLO26_MODEL_PATH "${YOLOS_CPP_DIR}/models/yolo26n.onnx")
if(EXISTS "${YOLO26_MODEL_PATH}")
  install(FILES "${YOLO26_MODEL_PATH}" DESTINATION share/${PROJECT_NAME}/models)
endif()

# install rviz config
install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_export_targets(${PROJECT_NAME}_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_package()
