#include <iostream>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
/*
Block of size (p,q), starting at (i,j)
matrix.block<p,q>(i,j);
*/
int main(int argc, char** argv)
{
    ros::init (argc, argv, "init_guess");
    ros::NodeHandle nh;

    Eigen::Matrix4d transform1 , transform2;
       transform1 << 1, 5, 9, 13,
                     2, 6, 10, 14,
                     3, 7, 11, 15,
                     4, 8, 12, 16;

    Eigen::Matrix3d rotation1 = transform1.block<3,3>(0,0);
    Eigen::Vector3d translation1 = transform1.block<3,1>(0,3);

    transform2.block<3,3>(0,0) = rotation1.block<3,3>(0,0);
    transform2.block<3,1>(0,3) = translation1.block<3,1>(0,0);
    transform2(3,3) = 99.0;
    std::cout<<"rotation1"<<rotation1<<std::endl;
    std::cout<<"rotation1"<<translation1<<std::endl;
    std::cout<<"rotation1"<<transform2<<std::endl;
    return 0;
}
