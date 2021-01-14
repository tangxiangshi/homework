cmake_minimum_required(VERSION 3.0.2)
project(pcl)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs


)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)


catkin_package(
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs
  DEPENDS  PCL
  DEPENDS  EIGEN3 


)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


add_executable (pass_through src/pcd_read.cpp)
target_link_libraries (pass_through ${PCL_LIBRARIES} ${catkin_LIBRARIES})

