cmake_minimum_required(VERSION 3.0.2)
project(write_pcl)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)


catkin_package(
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs
  DEPENDS EIGEN3
  DEPENDS PCL

)


include_directories(

  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable (write_pcl src/write_pcl.cpp)
target_link_libraries (write_pcl ${PCL_LIBRARIES} ${catkin_LIBRARIES})


