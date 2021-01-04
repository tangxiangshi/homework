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

add_executable(my_static_tf2_broadcaster src/static_turtle_tf2_broadcaster.cpp)
target_link_libraries(my_static_tf2_broadcaster ${catkin_LIBRARIES})
