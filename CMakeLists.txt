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


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
catkin_package(
  LIBRARIES 
)

## Declare a C++ executable



ADD_EXECUTABLE(flow src/flow.cpp)
TARGET_LINK_LIBRARIES(flow ${catkin_LIBRARIES} ${OpenCV_LIBS})

ADD_EXECUTABLE(mono_vo src/mono_vo.cpp)
TARGET_LINK_LIBRARIES(mono_vo ${catkin_LIBRARIES} ${OpenCV_LIBS})


#ADD_EXECUTABLE(compare src/compare.cpp)
#TARGET_LINK_LIBRARIES(compare ${catkin_LIBRARIES})
