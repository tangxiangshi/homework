#pragma once
#include<ros/ros.h>
#include<visualization_msgs/Marker.h>
#include<sensor_msgs/Imu.h>
#include<cmath>
#include<Eigen/Dense>

struct Pose
{
    Eigen::Vector3d pose;
    Eigen::Matrix3d orien;
};



