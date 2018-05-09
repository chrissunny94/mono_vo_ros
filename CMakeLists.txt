cmake_minimum_required(VERSION 2.8.3)
project(mono_vo_ros)
## Find catkin macros and libraries
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

###################################
## catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES cone_detector
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


## Declare a C++ executable


add_executable(subscribe src/subscribe.cpp)
add_dependencies(subscribe ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(subscribe ${OpenCV_LIBS} ${catkin_LIBRARIES}  )
