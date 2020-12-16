cmake_minimum_required(VERSION 3.0.2)
project(imu_integration)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  sensor_msgs
  rospy
  visualization_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu_integration
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
DEPENDS EIGEN3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(hw3_node src/hw3_node.cpp)
target_link_libraries(hw3_node ${catkin_LIBRARIES})


