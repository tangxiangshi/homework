cmake_minimum_required(VERSION 3.0.2)
project(my_pcl_tutorial)


find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)
find_package(PCL REQUIRED)


catkin_package()



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(example src/example.cpp)
target_link_libraries(example ${catkin_LIBRARIES} ${PCL_LIBRARIES})
