#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/registration/icp.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>
#include <cmath>
#include <geometry_msgs/PointStamped.h>
#include <queue>
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // map cloud
ros::Publisher sensor_pub;
// std::queue<sensor_msgs::PointCloud2ConstPtr> surfLastBuf;
// 声明一个储存lidar msg的容器
std::queue<sensor_msgs::PointCloud2::ConstPtr> lidar_msg_queue;
void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr &msg) // msg is lidar cloud
{
  // 这里的pcl_msgs为lidar扫描的point，还需要lidar to imuframe ，再由imuframe to world frame
  // 怎样将msg push到这个容器中呢？
  lidar_msg_queue.push(msg); // 怎么样才能将容器中的msg当中的time stamp对齐呢 与imu得到信息

  // sensor_pub.publish(*msg); 暂时用不到

  // pcl::fromROSMsg(*msg,pcl_msgs);

  // 1.为了解决坐标转换问题，需要考虑lidar frame to body frame 再 body frame to world frame，因为angular velocity及linear accerleration 在imu frame计算

  // 2.再解决在imu frame上的坐标吧，所有的坐标转到t0时刻吧

  // 3.最终进行坐标转换 需要有一个initial guess,通过GPS作为initial guess （通过订阅话题/fix）

  // 这里我下面写的code有问题，因为没有经过坐标转换，所以应该都是lidar上面看的点云
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  // *cloud_in = pcl_msgs;
  // cloud_out = cloud;
  // pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
  // icp.setInputSource(cloud_in);
  // icp.setInputTarget(cloud_out);
  // pcl::PointCloud<pcl::PointXYZ> Final;
  // icp.align(Final);
  // std::cout << "has converged: " << icp.hasConverged() << " score " << 
  // icp.getFitnessScore() << std::endl;
  // std::cout << icp.getFinalTransformation() << std::endl;
  
}

// For rotation in imu
void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
  
// 在这里我想将能够对其时间戳的lidar msg 转换

}

// for initial guess in ICP
void gps_callback(const geometry_msgs::PointStamped::ConstPtr &msg) 
{


}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "pcl_reader");
  ros::NodeHandle n;
  ros::Publisher pcl_pub = n.advertise<sensor_msgs::PointCloud2>("pcl_pub",1);
  
  sensor_msgs::PointCloud2 cloud_sensor; // pcl cloud in sensor format

  if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/tangxiangshi/catkin_ws/src/write_pcl/src/test.pcd", *cloud) ==  -1)
  {
    PCL_ERROR ("Couldn't read file map.pcd \n");
    return (-1);  
  }
  pcl::toROSMsg (*cloud, cloud_sensor);
  pcl::visualization::CloudViewer viewer("cloud viewer");
  viewer.showCloud(cloud);  
  pcl_pub.publish(cloud_sensor);
  // ros::Subscriber lidar_sub = n.subscribe<sensor_msgs::PointCloud2>("/lidar_points",1,lidar_callback);
 
  // ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/imu_data",1,imu_callback);

  // ros::Subscriber gps_sub = n.subscribe<geometry_msgs::PointStamped>("/fix",1,gps_callback);

  ros::spin();
  

}
