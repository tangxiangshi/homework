#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Point.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <midterm/ncrl.h>
#include <geometry_msgs/PointStamped.h>
#include <mutex>

std::mutex mbuf;

ros::Publisher map_pub,map_part_pub;

pcl::PointCloud<PointType>::Ptr map_cloud(new pcl::PointCloud<PointType>());
//match的map cloud
pcl::PointCloud<PointType>::Ptr map_icp(new pcl::PointCloud<PointType>());

//set cube of size is 0.5cm^3
double voxel_size = 1.0;

pcl::VoxelGrid<PointType> downSizeCloud;
pcl::PassThrough<PointType> pass;
Eigen::Vector3d init_point_last(0,0,0);

void gps_data(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  init_point_last(0) = msg->point.x;
  init_point_last(1) = msg->point.y;
  init_point_last(2) = msg->point.z;
  //std::cout<<inital_trslation.transpose()<<std::endl;
}

void show_map(pcl::PointCloud<PointType>::Ptr map_cloud)
{
  std::cout<<"show_map"<<std::endl;
  sensor_msgs::PointCloud2 output,output1;
  std::vector<int> map_indices;

  pcl::io::loadPCDFile("/home/ee405423/homework/src/map/map.pcd", *map_cloud);

  pcl::removeNaNFromPointCloud(*map_cloud, *map_cloud, map_indices);

  //要放pointer adress
  downSizeCloud.setInputCloud(map_cloud);
  downSizeCloud.setLeafSize(voxel_size, voxel_size, voxel_size);
  downSizeCloud.filter(*map_cloud);

  *map_icp = *map_cloud;
  std::cout<<"map -> init_point_last"<<init_point_last.transpose()<<std::endl;
//  pass.setFilterLimitsNegative(false);
  //設置顯示範圍大小(看能不能y方向也濾一點)
  pass.setInputCloud(map_icp);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(init_point_last(0)-100,init_point_last(0)+100);
  pass.filter(*map_icp);

////  pass.setFilterFieldName("z");
////  pass.setFilterLimits(0.1, 50.0);
////  pass.filter(*map_cloud);

  pass.setFilterFieldName("y");
  pass.setFilterLimits(init_point_last(1)-100, init_point_last(1)+100);
  //pass.setFilterLimitsNegative(true);
  pass.filter(*map_icp);

  pcl::toROSMsg(*map_icp, output1);
  output1.header.stamp = ros::Time::now();
  output1.header.frame_id = "/map";
  map_part_pub.publish(output1);


  pcl::toROSMsg(*map_cloud, output);
  output.header.stamp = ros::Time::now();
  output.header.frame_id = "/map";
  map_pub.publish(output);
}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "init_guess_");
  ros::NodeHandle nh;

  // /poseToFindMapRange
  ros::Subscriber gps_sub = nh.subscribe<geometry_msgs::PointStamped> ("/fix", 1,gps_data);

  map_pub = nh.advertise<sensor_msgs::PointCloud2> ("map_cloud", 1);
  map_part_pub = nh.advertise<sensor_msgs::PointCloud2> ("map_part_cloud", 1);


  std::cout<<"<<<"<<init_point_last.transpose()<<std::endl;
  if(init_point_last(0) != 0)
  {
    show_map(map_cloud);
    std::cout<<"fuck"<<std::endl;
  }
  ros::spin();
  return 0;
}
