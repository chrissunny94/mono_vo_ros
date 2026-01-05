cmake_minimum_required(VERSION 2.8.3)
project(mono_vo_ros1)

# --------------------------------------------------
# Detect ROS1 (catkin)
# --------------------------------------------------
find_package(catkin QUIET)

if(NOT catkin_FOUND)
  message(STATUS "ROS1 (catkin) not detected. Skipping build of mono_vo_ros.")
  return()
endif()

message(STATUS "ROS1 detected. Building mono_vo_ros.")

# --------------------------------------------------
# ROS1 dependencies
# --------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  geometry_msgs
  sensor_msgs
  std_msgs
  tf_conversions
  image_transport
  tf
)

find_package(OpenCV REQUIRED)

# --------------------------------------------------
# catkin configuration
# --------------------------------------------------
catkin_package(
  LIBRARIES
)

# --------------------------------------------------
# Include directories
# --------------------------------------------------
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# --------------------------------------------------
# Build targets
# --------------------------------------------------
add_executable(flow src/flow.cpp)
target_link_libraries(flow
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(mono_vo src/mono_vo.cpp)
target_link_libraries(mono_vo
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
