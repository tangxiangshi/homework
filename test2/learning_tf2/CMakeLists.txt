cmake_minimum_required(VERSION 3.0.2)
project(learning_tf2)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2
  tf2_ros
  turtlesim
)


catkin_package(
   
)

include_directories(

  ${catkin_INCLUDE_DIRS}
)

install(FILES
 start_demo.launch
 # myfile2
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


add_executable(turtle_tf2_broadcaster src/turtle_tf2_broadcaster.cpp)
target_link_libraries(turtle_tf2_broadcaster ${catkin_LIBRARIES})