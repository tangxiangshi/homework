cmake_minimum_required(VERSION 3.0.2)
project(pass_through)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
add_executable(pass_through_new src/pass_through_new.cpp)
target_link_libraries(pass_through_new ${catkin_LIBRARIES} ${PCL_LIBRARIES})



