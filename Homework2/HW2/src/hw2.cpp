// include ros library
#include "ros/ros.h"

// include msg library
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

// include math 
#include <math.h>
#define pi 3.1415926

using namespace std;

turtlesim::Pose pose;
geometry_msgs::Twist vel_msg;
geometry_msgs::Point goal_point;
//Define a data structure to 3D
struct XYZ{
  float x;
  float y;
  float z;
};
//Declare a variable.Its name is pos_err with XYZ data type
struct XYZ pos_err;

// declare call back function(call back the pose of robot)
void pos_cb(const turtlesim::Pose::ConstPtr& msg)
{
  pose = *msg;
} 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tutorial_1");
  ros::NodeHandle n;

  // declare publisher & subscriber
  ros::Subscriber pos_sub = n.subscribe<turtlesim::Pose>("turtle1/pose", 10, pos_cb);
  ros::Publisher turtlesim_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);
  //input your desired position
  ROS_INFO("Please input (x,y). x>0,y>0");
  cout<<"desired_X:";
  cin>>goal_point.x;
  cout<<"desired_Y:";
  cin>>goal_point.y;
  // setting frequency as 10 Hz
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()){

    printf("\ncount : %d\n",count);
    printf("goal x : %f \t y : %f\n",goal_point.x,goal_point.y);
    printf("pose x : %f \t y : %f\n",pose.x,pose.y);

    // Calculate position error(feedback term)
    pos_err.x = goal_point.x - pose.x;
    pos_err.y = goal_point.y - pose.y;

    float error_theta;


    error_theta = atan2(-sin(pose.theta)*pos_err.x + cos(pose.theta)*pos_err.y , cos(pose.theta)*pos_err.x + sin(pose.theta)*pos_err.y);
    if(error_theta>(-0.05) && error_theta<0.05)
    {
        vel_msg.angular.z = 0 ;
	}
    else
    {
        vel_msg.angular.z = 1 ;
	}
    
    if((pos_err.x * pos_err.x + pos_err.y * pos_err.y)>0.05 && vel_msg.angular.z==0) 
    {
        vel_msg.linear.x=5;
    }
    else 
    {
        vel_msg.linear.x=0;
     }	
    
/*    float desired_theta;
    float error_theta;

    if	 (pos_err.x==0&&pos_err.y==0)   desired_theta=0;
    else if(pos_err.x>0&&pos_err.y==0)  desired_theta=0;
    else if(pos_err.x&&pos_err.y>0)   desired_theta=atan(pos_err.y/pos_err.x);
    else if(pos_err.x&&pos_err.y>0)  desired_theta=pi/2;
    else if(pos_err.x&&pos_err.y>0)   desired_theta=atan(pos_err.y/pos_err.x)+pi;
    else if(pos_err.x<0&&pos_err.y==0)  desired_theta=pi;
    else if(pos_err.x<0&&pos_err.y<0)   desired_theta=atan(pos_err.y/pos_err.x)+pi;
    else if(pos_err.x==0&&pos_err.y<0)  desired_theta=pi/2+pi;
    else if(pos_err.x>0&&pos_err.y<0)   desired_theta=atan(pos_err.y/pos_err.x)+2*pi;

    error_theta = pose.theta-desired_theta;
  if(error_theta>pi){
    error_theta=-(2*pi-error_theta);
  }
  else if(error_theta<-pi){
   error_theta=2*pi+error_theta;
  }

  if(error_theta<-0 && error_theta>-pi)  vel_msg.angular.z=pi;
  else if(error_theta>0 && error_theta<pi)  vel_msg.angular.z=-pi;
  else vel_msg.angular.z=0;
  
  if((pos_err.x * pos_err.x + pos_err.y * pos_err.y)>0.01 && vel_msg.angular.z==0) vel_msg.linear.x=5;
  else vel_msg.linear.x=0.0;
*/

	
    /*Your error-driven controller design
     

    /*vel_msg.linear.x = (Your control input design);
     *vel_msg.angular.z = (Your control input design);*/
    turtlesim_pub.publish(vel_msg);

    count ++;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}



