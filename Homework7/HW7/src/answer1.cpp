#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include<Eigen/Geometry>

void broadcastTF(const ros::TimerEvent& timer_event)
{
  static tf::TransformBroadcaster br;
  Eigen::Quaterniond q_B_A(sqrt(2)/2,0,0,sqrt(2)/2), q_C_B(sqrt(3)/2,0,0,1/2);// Define q_B_A  q_B_A in Eigen::Quaterniond
  Eigen::Vector3d v_B_A(1,0,0), v_C_B(1,0,0);// Define v_B_A  v_C_B in Eigen::Vector3d
  tf::Transform tf_B_A, tf_C_B;// Define tf_B_A, tf_C_B
  tf::Quaternion tf_q_B_A(q_B_A.x(),q_B_A.y(),q_B_A.z(),q_B_A.w()),  tf_q_C_B(q_C_B.x(),q_C_B.y(),q_C_B.z(),q_C_B.w()); // transfer information from eigen to tf
  tf::Vector3     tf_v_B_A(v_B_A(0),v_B_A(1),v_B_A(2)), tf_v_C_B(v_C_B(0),v_C_B(1),v_C_B(2)); // transfer information from eigen to tf

  
 

  tf_B_A.setOrigin(tf_v_B_A);
  tf_B_A.setRotation(tf_q_B_A);
  tf_C_B.setOrigin(tf_v_C_B);
  tf_C_B.setRotation(tf_q_C_B);
  
 
 
  br.sendTransform(tf::StampedTransform(tf_B_A, // transform
                                        ros::Time::now(), // timestamp with this transform
                                        "B", // paranet frame ID
                                        "A")); // child frame ID

  br.sendTransform(tf::StampedTransform(tf_C_B,
                                        ros::Time::now(),
                                        "C",
                                        "B"));
  
  tf::Transform tf_C_A ;// Define tf_C_A,
  tf::Vector3     tf_v_C_A(3/2,-sqrt(3)/2,0); // For this part ,I calculated by myself, for that I do not kown how to show the vector from two coordinate systems by C++
  tf::Quaternion tf_q_C_A(0,0,sqrt(6)/4+sqrt(2)/4,sqrt(6)/4-sqrt(2)/4);// Define tf_q_C_A , this is by using tf_q_C_B x tf_q_B_A
  tf_C_A.setOrigin(tf_v_C_A);
  tf_C_A.setRotation(tf_q_C_A);
  br.sendTransform(tf::StampedTransform(tf_C_A,
                                        ros::Time::now(),
                                        "C",
                                        "A"));
  
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "answer1");
  ros::NodeHandle nh;
  // Create timer with 2.0 Hz
  ros::Timer timer = nh.createTimer(ros::Duration(0.5), broadcastTF);
  while (ros::ok()){ros::spinOnce();}
  return 0;
}
