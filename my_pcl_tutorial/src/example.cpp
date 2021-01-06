#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

// ros::Publisher pub;

// void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
// {
//     pcl::PCLPointCloud2* cloud(new pcl::PCLPointCloud2);
//     pcl::PCLPointCloud2::ConstPtr cloudPtr(cloud);  // ConstPtr 为常量指针，指针的指向可以修改，指针指向的值不可以修改
//     pcl::PCLPointCloud2 cloud_filtered;
//     pcl_conversions::toPCL(*input, *cloud);
//     pcl::VoxelGrid<pcl::PCLPointCloud2> sor; // 体素栅格底下采样对象
//     sor.setInputCloud(cloudPtr);
//     sor.setLeafSize(0.1,0.1,0.1); // 设置采样体素大小   
//     sor.filter(cloud_filtered);

//     sensor_msgs::PointCloud2 output;
//     pcl_conversions::moveFromPCL(cloud_filtered, output); 
//     // pcl_conversions::fromPCL() 用法并不一样，http://docs.ros.org/en/indigo/api/pcl_conversions/html/namespacepcl__conversions.html#a803fd03ec5921d84d9e6cb4dd2a3c164，参考
//     pub.publish(output);

// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "example");
    ros::NodeHandle n;
    // ros::Subscriber sub = n.subscribe("input", 1, cloud_cb);
    // pub = n.advertise<sensor_msgs::PointCloud2>("output",1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("/home/tangxiangshi/catkin_ws/src/pcl/include/test.pcd", *cloud);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.1,0.1,0.1);
    sor.filter(*cloud_filtered);
    pcl::visualization::CloudViewer viewer("pcl_viewer");
    viewer.showCloud(cloud_filtered);

    ros::spin();
    
}
