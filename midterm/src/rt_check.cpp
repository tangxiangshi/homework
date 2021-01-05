#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <string>
#include "eigen3/Eigen/Dense"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include<sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>   // TicToc

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
PointCloudT::Ptr scan_last (new PointCloudT);
PointCloudT::Ptr scan_current (new PointCloudT);
PointCloudT::Ptr map_current (new PointCloudT);
Eigen::Matrix4d T_g;
Eigen::Vector4d  X_r_hat;
Eigen::Quaterniond X_q_hat;
geometry_msgs::PoseStamped X_state_hat;
ros::Publisher X_state_hat_pub,path_X_pub,aligned_pc2_pub;



void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

/*void pose_cb(const geometry_msgs::PoseStamped::ConstPtr & car_pose_init)
{
    X_state_hat = *car_pose_init;


   /*******************************System initialization*******************************
    //initialize the pose of car
    X_r_hat << X_state_hat.pose.position.x,  X_state_hat.pose.position.y, X_state_hat.pose.position.z, 1;//car_position_init  : Xr_hat(0)
    X_q_hat.x() = X_state_hat.pose.orientation.x;
    X_q_hat.y() = X_state_hat.pose.orientation.y;
    X_q_hat.z() = X_state_hat.pose.orientation.z;
    X_q_hat.w() = X_state_hat.pose.orientation.w;//car_attitude_init : Xq_hat(0)

    //initialize the transformation matrix : T_hat(0)
    T_g.block<3,3>(0,0) = X_q_hat.toRotationMatrix();//R_hat(0)
    T_g.block<4,1>(0,3) = X_r_hat;//t_hat(0)

    //switch
}*/


/*void scan_init_cb(const sensor_msgs::PointCloud2ConstPtr & scan_pc2)
{
    /*******************only for the first time************************
    //initialization for scan_current
    pcl::fromROSMsg (*scan_pc2, *scan_last);
    //switch
}*/

void scan_pc2_cb(const sensor_msgs::PointCloud2ConstPtr & scan_pc2)
{
    /*******************only for the first time************************
    //initialization for scan_current
    pcl::fromROSMsg (*scan_pc2, *scan_last); */
    //switch
    pcl::fromROSMsg (*scan_pc2, *scan_current);
}

void map_pc2_cb(const sensor_msgs::PointCloud2ConstPtr & map_pc2)
{
    pcl::fromROSMsg (*map_pc2, *map_current);
}

void f2f_icp(PointCloudT::Ptr scan_current)
{
    //f2f matching
    pcl::IterativeClosestPoint<PointT, PointT> f2f_icp;
    f2f_icp.setInputSource (scan_last);
    f2f_icp.setInputTarget (scan_current);
    PointCloudT::Ptr aligned_f2f (new PointCloudT);
    *aligned_f2f = *scan_last;
    f2f_icp.align (*aligned_f2f);

    //after matching
    Eigen::Matrix4d T_f2f;
    T_f2f = f2f_icp.getFinalTransformation().cast<double>();//data type change from float to double

    //predict car_pose
    X_r_hat = T_f2f *  X_r_hat;//Xr_hat(k) = T_f2f(k) * Xr_hat(k)
    X_q_hat = T_f2f.block<3,3>(0,0) *  X_q_hat;//Xq_hat(k) = R_f2f(k) * Xq_hat(k)

    /*********************************************Result*********************************************/
    X_state_hat.header.frame_id = "my_frame";
    X_state_hat.header.stamp = ros::Time::now();
    X_state_hat.pose.position.x = X_r_hat(0);
    X_state_hat.pose.position.y = X_r_hat(1);
    X_state_hat.pose.position.z = X_r_hat(2);
    X_state_hat.pose.orientation.x = X_q_hat.x();
    X_state_hat.pose.orientation.y = X_q_hat.y();
    X_state_hat.pose.orientation.z = X_q_hat.z();
    X_state_hat.pose.orientation.w = X_q_hat.w();
    X_state_hat_pub.publish(X_state_hat);//for map_initialization
}

void f2m_icp(PointCloudT::Ptr scan_current,PointCloudT::Ptr map_current )
{
    //scan_current b2g transformation
    T_g.block<3,3>(0,0) = X_q_hat.toRotationMatrix();
    T_g.block<4,1>(0,3) = X_r_hat;
    PointCloudT::Ptr scan_current_g (new PointCloudT);
    pcl::transformPointCloud (*scan_current, *scan_current_g, T_g);

    pcl::IterativeClosestPoint<PointT, PointT> f2m_icp;
    f2m_icp.setInputSource (scan_current_g);
    f2m_icp.setInputTarget (map_current);
    PointCloudT::Ptr aligned_f2m (new PointCloudT);
    *aligned_f2m = *scan_current_g;
    f2m_icp.align (*aligned_f2m);

    //After matching
    sensor_msgs::PointCloud2 aligned_pc2;
    pcl::toROSMsg(*aligned_f2m,aligned_pc2);
    Eigen::Matrix4d T_f2m;
    T_f2m = f2m_icp.getFinalTransformation().cast<double>();//data type change from float to double

    //Correct the pose of car
    Eigen::Vector4d  X_r;
    Eigen::Quaterniond X_q;
    X_r = T_f2m *  X_r_hat;//Xr(k) = T_f2m(k) * Xr_hat(k)
    X_q = T_f2m.block<3,3>(0,0) *  X_q_hat;//Xq(k) = R_f2m(k) * Xq_hat(k)


    /*********************************************Result*********************************************/
    Eigen::Vector3d euler_X;
    euler_X = X_q.toRotationMatrix().eulerAngles(2, 1, 0);//csv output

    geometry_msgs::PoseStamped X_state;
    nav_msgs::Path path_X;
    X_state.header.frame_id = path_X.header.frame_id  = aligned_pc2.header.frame_id = "my_frame";
    X_state.header.stamp = path_X.header.stamp = aligned_pc2.header.stamp = ros::Time::now();
    X_state.pose.position.x = X_r(0);
    X_state.pose.position.y = X_r(1);
    X_state.pose.position.z = X_r(2);
    X_state.pose.orientation.x = X_q.x();
    X_state.pose.orientation.y = X_q.y();
    X_state.pose.orientation.z = X_q.z();
    X_state.pose.orientation.w = X_q.w();
    path_X.poses.push_back(X_state);
    path_X_pub.publish(path_X);
    aligned_pc2_pub.publish(aligned_pc2);


    /***********************************************Termination***********************************************/
    //for next time step initailization
    X_r_hat =  X_r;//Xr_hat(k+1) = Xr(k)
    X_q_hat =  X_q;//Xq_hat(k+1) = Xq(k)
    *scan_last = *scan_current;//scan_last (k+1) = scan_current (k)
}


int main( int argc, char** argv )
{
    ros::init(argc, argv, "icp");
    ros::NodeHandle n;


    /*************************************System initialization*************************************/
    //initialize the pose of car
    X_r_hat <<-2.03211,226.254,-1.11179, 1; //car_position_init : Xr_hat(0)
    X_q_hat = Eigen::AngleAxisd(-2.26029, Eigen::Vector3d::UnitX())
                       *Eigen::AngleAxisd(0.054405, Eigen::Vector3d::UnitY())
                       *Eigen::AngleAxisd(-0.0042194, Eigen::Vector3d::UnitZ());//car_attitude_init : Xq_hat(0)

   //initialize the transformation matrix : T_hat(0)
    T_g.setZero();
    T_g.block<3,3>(0,0) = X_q_hat.toRotationMatrix();//R_hat(0)
    T_g.block<4,1>(0,3) = X_r_hat;//t_hat(0)


    /***********************************Subscriber and Publisher***********************************/
    //subscribe the car_pose initial_guess
    //ros::Subscriber pose_sub = n.subscribe<geometry_msgs::PoseStamped>("car_pose_init", 10, pose_cb);
    ros::Subscriber scan_pc2_sub = n.subscribe<sensor_msgs::PointCloud2>("/lidar_points", 10, scan_pc2_cb);

   //subscribe map after preprocessing
    ros::Subscriber map_pc2_sub = n.subscribe<sensor_msgs::PointCloud2>("/map_pc2", 10, map_pc2_cb);

    //publish car_pose_hat for map initialization
    X_state_hat_pub = n.advertise<geometry_msgs::PoseStamped>("/car_pose_hat", 10);

    //publish car_trajectory
    path_X_pub= n.advertise<nav_msgs::Path>("/X_trajectory", 10);

    //publish aligned cloud after ICP
    aligned_pc2_pub  = n.advertise<sensor_msgs::PointCloud2> ("/aligned_pc2", 10);


    ros::spin();
    return (0);
}
