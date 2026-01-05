cmake_minimum_required(VERSION 3.10)
project(mono_vo_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)


# --------------------------------------------------
# Paths to core (one level up)
# --------------------------------------------------
set(MONO_VO_CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)

# --------------------------------------------------
# Core library
# --------------------------------------------------
add_library(mono_vo_core
  ${MONO_VO_CORE_DIR}/src/mono_vo_core.cpp
)

target_include_directories(mono_vo_core
  PUBLIC
    ${MONO_VO_CORE_DIR}/include
    ${OpenCV_INCLUDE_DIRS} 
)

add_executable(mono_vo_node src/mono_vo_node.cpp)

target_link_libraries(mono_vo_node
  mono_vo_core
  ${OpenCV_LIBS}
)

ament_target_dependencies(mono_vo_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
)

install(TARGETS mono_vo_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
