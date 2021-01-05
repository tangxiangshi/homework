cmake_minimum_required(VERSION 2.8.3)
project(midterm)

set(CMAKE_CXX_FLAGS "-std=c++11")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  rospy
  tf
  image_transport
  visualization_msgs
  cv_bridge
  pcl_ros
  pcl_conversions
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ncrl_px4
#  CATKIN_DEPENDS roscpp
DEPENDS EIGEN3 PCL
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

#自己創造的lib
#ncrl 為動態庫的名稱
add_library(ncrl
    include/midterm/ncrl.h
    src/ncrl.cpp
)
target_link_libraries(ncrl ${catkin_LIBRARIES})
add_dependencies(ncrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#

add_executable(pcd_show src/pcd_show.cpp)
#記得target_link_libraries 要加ncrl
target_link_libraries(pcd_show ncrl ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS} ${CERES_LIBRARIES})

add_executable(csv_test src/csv_test.cpp)
#記得target_link_libraries 要加ncrl
target_link_libraries(csv_test ncrl ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS} ${CERES_LIBRARIES})



#add_executable(FtoFmanager src/FtoFmanager.cpp)
#記得target_link_libraries 要加ncrl
#target_link_libraries(FtoFmanager ncrl ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS} ${CERES_LIBRARIES})
