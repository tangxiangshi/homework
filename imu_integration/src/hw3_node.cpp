#include<ros/ros.h>
#include<visualization_msgs/Marker.h>
#include<sensor_msgs/Imu.h>
#include<cmath>
#include<Eigen/Dense>


double latest_time = 0;
Eigen::Vector3d linear_velocity;
Eigen::Vector3d angular_last{0,0,0}; // 這個地方對嗎? 我想的初始速度 爲 0;
Eigen::Vector3d linear_velocity_last{0,0,0};
Eigen::Vector3d pose_last{0,0,0};
Eigen::Vector3d gravity{0,0,9.8};
Eigen::Quaterniond Euler2Quaternion(Eigen::Vector3d euler)  // 這邊爲什麼又可以 euler.x() 不是很理解
{
  Eigen::Quaterniond Q;
  Q = Eigen::AngleAxisd(euler.z(),Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(euler.y(),Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(euler.z(),Eigen::Vector3d::UnitZ());
  return Q;
}
// Eigen::Vector3d Pose = rotation_first();
geometry_msgs::Vector3 pose(const sensor_msgs::Imu::ConstPtr &msg)
{
  double t = msg->header.stamp.toSec();
  double dt = t - latest_time ;
  double latest_time = t;
  
  Eigen::Vector3d linear_velocity;
  float rx = msg->angular_velocity.x;
  float ry = msg->angular_velocity.y;
  float rz = msg->angular_velocity.z;
  Eigen::Vector3d angular_velocity_current{rx, ry, rz};
  Eigen::Vector3d angular_current = angular_last + angular_velocity_current * dt;
  angular_last = angular_current;
  Eigen::Quaterniond Q_rotation = Euler2Quaternion(angular_current);

  float dx = msg->linear_acceleration.x;
  float dy = msg->linear_acceleration.y;
  float dz = msg->linear_acceleration.z;
  Eigen::Vector3d linear_acceleration{dx, dy, dz};
  Eigen::Vector3d linear_velocity_current = linear_velocity_last + (Q_rotation * linear_acceleration - gravity) * dt;
  Eigen::Vector3d linear_velocity_last = linear_velocity_current;
  Eigen::Vector3d velocity_world_frame = Q_rotation * linear_velocity_current;
  Eigen::Vector3d pose_current = pose_last + velocity_world_frame * dt;
  return geometry_msgs::Vector3 pose = pose_current;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hw3_085115");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker",10);
  ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/imu/data",10, pose);
  ros::Rate loop_rate(30);
  while(ros::ok)
  {
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "/global";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "line_strip";
    line_strip.id = 0;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.lifetime = ros::Duration();
    line_strip.color.b = 1.0f;
    line_strip.color.a = 1;
    line_strip.scale.x = 0.1;
    line_strip.points.push_back(pose);   
    marker_pub.publish(line_strip);
    loop_rate.sleep();
  }

}
