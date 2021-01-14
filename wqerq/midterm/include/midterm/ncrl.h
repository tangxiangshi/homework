#ifndef NCRL_H
#define NCRL_H

#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <cstring>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZI PointType;

class ncrl	 //  class name: Rectangle
{

    public:
        ncrl(){};

    struct Data{
      Eigen::Vector3d acc;
      Eigen::Vector3d ang_vel;
    };

    /*static 出現在 class 的 member variable 的意思是該 variable 並不屬於某個 instance，他屬於這個 class*/
    /*static 出現在 member function 的意思是該 function 並不屬於某個 instance，他也屬於這個 class，所有以此
     class 生成出來的 instance 都共用這個 function*/
    void static set(Data &p, Eigen::Vector3d linear_a, Eigen::Vector3d angular_ );
    Eigen::Quaterniond static rotationMatrix2Quaterniond(Eigen::Matrix3d R);
    void static transform(Eigen::Vector3d &linear_a , Eigen::Vector3d &angular_ ,Eigen::Matrix3d Imu2ZED_odom, Eigen::Quaterniond &quater);
};
#endif
