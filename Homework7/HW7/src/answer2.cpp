#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include<Eigen/Geometry>

void broadcastTF(const ros::TimerEvent& timer_event)
{
  static tf::TransformBroadcaster br;
  Eigen::Quaterniond q_A_B(sqrt(2)/2,0,0,sqrt(2)/2), q_A_C(sqrt(3)/2,0,0,1/2);// Define q_A_B  q_A_C in Eigen::Quaterniond
  Eigen::Vector3d v_A_B(1,0,0), v_A_C(1,0,0);// Define v_A_B  v_A_C in Eigen::Vector3d
  tf::Transform tf_A_B, tf_A_C;// Define tf_B_A, tf_C_B
  tf::Quaternion tf_q_A_B(q_A_B.x(),q_A_B.y(),q_A_B.z(),q_A_B.w()),  tf_q_A_C(q_A_C.x(),q_A_C.y(),q_A_C.z(),q_A_C.w()); // transfer information from eigen to tf
  tf::Vector3     tf_v_A_B(v_A_B(0),v_A_B(1),v_A_B(2)), tf_v_A_C(v_A_C(0),v_A_C(1),v_A_C(2)); // transfer information from eigen to tf

  
 

  tf_A_B.setOrigin(tf_v_A_B);
  tf_A_B.setRotation(tf_q_A_B);
  tf_A_C.setOrigin(tf_v_A_C);
  tf_A_C.setRotation(tf_q_A_C);
  
 
 
  br.sendTransform(tf::StampedTransform(tf_A_B, // transform
                                        ros::Time::now(), // timestamp with this transform
                                        "A", // paranet frame ID
                                        "B")); // child frame ID

  br.sendTransform(tf::StampedTransform(tf_A_C,
                                        ros::Time::now(),
                                        "A",
                                        "C"));
  
  tf::Transform tf_C_B ;// Define tf_C_B,
  tf::Vector3     tf_v_C_B(1/2,-sqrt(3)/2,0); // For this part ,I calculated by myself, for that I do not kown how to show the vector multiply from two coordinate systems by C++
  tf::Quaternion tf_q_C_B(0,0,sqrt(6)/4-sqrt(2)/4,sqrt(6)/4+sqrt(2)/4);//Define tf_q_C_B
  tf_C_B.setRotation(tf_q_C_B);
  br.sendTransform(tf::StampedTransform(tf_C_B,
                                        ros::Time::now(),
                                        "C",
                                        "B"));
  
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "answer2");
  ros::NodeHandle nh;
  // Create timer with 2.0 Hz
  ros::Timer timer = nh.createTimer(ros::Duration(0.5), broadcastTF);
  while (ros::ok()){ros::spinOnce();}
  return 0;
}
