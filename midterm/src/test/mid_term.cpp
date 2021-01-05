#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "eigen3/Eigen/Dense"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>

geometry_msgs::PoseStamped this_pose_stamped;
ros::Publisher path_pub;
nav_msgs::Path path_g;
Eigen::Quaterniond q_g;
Eigen::MatrixXd Matrix; 



int main( int argc, char** argv )
{
  ros::init(argc, argv, "mid_term");
  ros::NodeHandle n;
  path_pub = n.advertise<nav_msgs::Path>("/ground_truth/trajectory", 1, true);
  ros::Rate loop_rate(100);
  std::vector<double> matrix;
  Matrix.resize(674,7);
  //readfile
  std::fstream file;
  file.open("/home/louise/autodrive_ws/src/midterm_project/include/ground_truth.csv");
  std::string line;
  int i = 0;
  while (std::getline( file, line,'\n'))  //讀檔讀到跳行字元
	{
	  std::istringstream templine(line); // string 轉換成 stream
	  std::string data;
	  while (std::getline( templine, data,',')) //讀檔讀到逗號
	  {
	    matrix.push_back(atof(data.c_str()));  //string 轉換成數字
	  }
	}
  file.close();
  for(int i = 0; i < 674; i++){
    for(int j = 0; j < 7; j++){
      Matrix(i , j) = matrix[7*i+j];
    }
  }

  while (ros::ok())
  {
    this_pose_stamped.header.frame_id = path_g.header.frame_id  = "my_frame";
    this_pose_stamped.header.stamp = path_g.header.stamp = ros::Time::now();


    this_pose_stamped.pose.position.x = Matrix(i,1);
    this_pose_stamped.pose.position.y = Matrix(i,2);
    this_pose_stamped.pose.position.z = Matrix(i,3);

    q_g =Eigen:: AngleAxisd(Matrix(i,4), Eigen::Vector3d::UnitX())*  Eigen::AngleAxisd(Matrix(i,5), Eigen::Vector3d::UnitY())*  Eigen::AngleAxisd(Matrix(i,6), Eigen::Vector3d::UnitZ());
    this_pose_stamped.pose.orientation.x = q_g.x();
    this_pose_stamped.pose.orientation.y = q_g.y();
    this_pose_stamped.pose.orientation.z = q_g.z();
    this_pose_stamped.pose.orientation.w = q_g.w();

    path_g.poses.push_back(this_pose_stamped);


    path_pub.publish(path_g);
    i++;
    ros::spinOnce();              

    loop_rate.sleep();
  }
  
  return 0;
}
