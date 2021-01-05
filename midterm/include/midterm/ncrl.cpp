#include "ncrl.h"

//這邊就不用在加static了
void ncrl::set(ncrl::Data &p, Eigen::Vector3d linear_a, Eigen::Vector3d angular_ )
{
 p.acc = linear_a;
 p.ang_vel = angular_;
}

Eigen::Quaterniond ncrl::rotationMatrix2Quaterniond(Eigen::Matrix3d R)
{
    Eigen::Quaterniond q = Eigen::Quaterniond(R);
    q.normalize();
    return q;
}

void ncrl::transform(Eigen::Vector3d &linear_a , Eigen::Vector3d &angular_ ,Eigen::Matrix3d Imu2ZED_odom ,Eigen::Quaterniond &quater)
{
  Eigen::Quaterniond Q;
  Q = ncrl::rotationMatrix2Quaterniond(Imu2ZED_odom);

  linear_a = Q * linear_a;
  angular_ = Q * angular_;
  quater = Q * quater;
}
