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
#include <pcl/filters/passthrough.h>


int main(int argc, char**argv)
{
    ros::init(argc, argv, "pass_through_new");
    ros::NodeHandle n;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // load pcd
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/tangxiangshi/catkin_ws/src/pcl/include/test.pcd", *cloud) ==  -1) 
    {
        PCL_ERROR ("Couldn't read file map.pcd \n");
        return (-1);  
    }
    // pcl passthrough filter implementation
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0,10);
    pass.filter(*cloud_filtered);
    // pcl viewer
    pcl::visualization::CloudViewer viewer("pcl_viewer_window");  // 创建一个显示窗口
    viewer.showCloud(cloud_filtered);
    ros::spin();

}