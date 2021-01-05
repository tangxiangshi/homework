
#include <iostream>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

Eigen::Vector3d x,y,x1,z1;

Eigen::Vector3d Quaternion2Euler(Eigen::Quaterniond Q)
{
      Eigen::Vector3d Euler(0, 0, 0);
      Euler.x() = atan2(2 * (Q.w() * Q.x() + Q.y() * Q.z()), (1 - 2 * (Q.x() * Q.x() + Q.y() * Q.y())));
      Euler.y() = asin(2 * (Q.w() * Q.y() + Q.z() * Q.x()));
      Euler.z() = atan2(2 * (Q.w() * Q.z() + Q.x() * Q.y()), (1 - 2 * (Q.y() * Q.y() + Q.z() * Q.z())));
    return Euler;
}

Eigen::Vector3d rad2deg(Eigen::Vector3d radians)
{
    return radians * 180.0 / M_PI;
}

Eigen::Vector3d deg2rad(Eigen::Vector3d degrees)
{
    return degrees * M_PI / 180.0;
}

int main(int argc, char** argv)
{
    ros::init (argc, argv, "init_guess");
    ros::NodeHandle nh;
    Eigen::Matrix3d rotation;
    Eigen::Quaterniond test(std::cos((30 * M_PI / 180.0f)) , 0,0 ,std::sin(30 * M_PI));
    std::cout<<M_PI<<std::endl;
    std::cout<<test.w()<<std::endl;

    for(int i=0; i<12;i++)
    {
      rotation <<std::cos((15 * i * M_PI) / 180.0), -1 * std::sin((15 * i * M_PI) / 180.0) ,0
                ,std::sin((15 * i* M_PI) / 180.0), std::cos((15 * i * M_PI) / 180.0)      ,0
                ,0          , 0                ,1;

      std::cout<<"Z : \t "<<rotation<<std::endl;

      test = test*test;

      std::cout<<"second : \t"<<test.toRotationMatrix()<<std::endl;
    }

    return 0;
}

