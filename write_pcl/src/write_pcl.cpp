#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <cmath>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZ>);
bool is_first_scan = true;

void call_back(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud_input);
    if (is_first_scan)
    {
        cloud_output = cloud_input;
        is_first_scan = false;
    }
    pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
    // Set the input source and target
    icp.setInputCloud (cloud_input);
    icp.setInputTarget (cloud_output);

    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (0.05);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (50);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (1);
    pcl::PointCloud<pcl::PointXYZ> final;
    icp.align(final);
    *cloud_output = final;
    pcl::io::savePCDFile("test.pcd",final);



}

int main(int argc, char** argv)
{
    ros::init(argc, argv,"write_pcl");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/lidar_points", 1, call_back);

    ros::spin();
}
