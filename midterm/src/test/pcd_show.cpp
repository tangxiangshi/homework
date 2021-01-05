#include <ros/ros.h>
#include <cstring>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl/io/pcd_io.h>//which contains the required definitions to load and store point clouds to PCD and other file formats.
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Point.h>
#include <pcl/registration/icp.h>
#include <midterm/ncrl.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>

/*
pcl::fromROSMsg(const sensor_msgs::PointCloud2 & ,	cloud,pcl::PointCloud< T >)
pcl::toROSMsg(const sensor_msgs::PointCloud2 & , sensor_msgs::PointCloud2)
*/
geometry_msgs::PoseStamped inital_guess;
ros::Publisher scan_pub,corr_pub,init_pub;
bool flag = true ,icp_flag = true;

//set cube of size is 0.5cm^3
double voxel_size = 1.0;

Eigen::Vector3d init_point_last(0,0,0);

//一定要有new 因為要初始化他,配置一塊空的記憶體
pcl::PointCloud<PointType>::Ptr scan_cloud(new pcl::PointCloud<PointType>());
//以rt完的點雲
pcl::PointCloud<PointType>::Ptr cloud_icp(new pcl::PointCloud<PointType>());
//match的map cloud
pcl::PointCloud<PointType>::Ptr map_icp(new pcl::PointCloud<PointType>());

void map_data(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  pcl::fromROSMsg(*msg,*map_icp);
}

//要在開gps_transoform
void gps_data(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  init_point_last(0) = msg->point.x;
  init_point_last(1) = msg->point.y;
  init_point_last(2) = msg->point.z;

  //std::cout<<inital_trslation.transpose()<<std::endl;
}

//確認要處理的是PCL還是Sensor的型態
void show_scan(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
  sensor_msgs::PointCloud2 output;
  pcl::fromROSMsg(*laserCloudMsg,*scan_cloud);

  pcl::toROSMsg(*scan_cloud, output);
  output.header.stamp = ros::Time::now();
  output.header.frame_id = "/map";

  scan_pub.publish(output);
}

void solve_icp(pcl::PointCloud<PointType>::Ptr scan_cloud, pcl::PointCloud<PointType>::Ptr map_cloud, Eigen::Vector3d init_point_last)
{
  //count->records best matrix
  int count = 0;
  double max_socre = 999999.0;
  Eigen::Vector3d trans;
  sensor_msgs::PointCloud2 output;
  pcl::PointCloud<PointType> Final;
  Eigen::Quaterniond ro(1,0,0,0) , r2(1,0,0,0),inital_quaternion(1,0,0,0);
  pcl::IterativeClosestPoint<PointType, PointType> icp;
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Matrix4d best_matrix = Eigen::Matrix4d::Identity();

  icp.setInputTarget(map_icp);

  //std::cout<<"scan_cloud->points.size()"<<scan_cloud->points.size()<<std::endl;
  //std::cout<<"map_cloud->points.size()"<<map_cloud->points.size()<<std::endl;
  //icp.setMaxCorrespondenceDistance(0.5);
  //icp.setTransformationEpsilon(1e-10);
  //icp.setEuclideanFitnessEpsilon(0.001);
  //icp.setMaximumIterations(100);

/*********************compute attitude every 15 degree******************/
  for(int i = 0 ;i < 24 ;i++)
  {
    rotation <<std::cos((15 * i * M_PI) / 180.0), -1 * std::sin((15 * i * M_PI) / 180.0)  ,0
              ,std::sin((15 * i * M_PI) / 180.0), std::cos((15 * i * M_PI) / 180.0)       ,0
              ,0                               , 0                                        ,1;

    /*//有時候沒得到值,需要寫個方法確認gps_part
    std::cout<<"init_point_last"<<init_point_last.transpose()<<std::endl;
    */
    for(int j=0; j < cloud_icp->points.size(); j++)
    {
      //std::cout<<"come2"<<std::endl;
      //先旋轉載平移
      trans << scan_cloud->points[j].x ,scan_cloud->points[j].y ,scan_cloud->points[j].z ;
      ro = rotation;
      trans = ro * trans;
      cloud_icp->points[j].x = trans(0) + init_point_last(0);
      cloud_icp->points[j].y = trans(1) + init_point_last(1);
      cloud_icp->points[j].z = trans(2) + init_point_last(2);
    }
    icp.setInputSource(cloud_icp);
    //這個絕對要照官網上的
    //final要另外放值進去
    icp.align(Final);

    std::cout <<"score : "<<icp.getFitnessScore() <<"max : "<<max_socre<<std::endl;
    if(icp.getFitnessScore() <= max_socre)
    {
        max_socre = icp.getFitnessScore();
        //Get the final transformation matrix estimated by the registration method.
        std::cout <<"Transformation"<<icp.getFinalTransformation() << std::endl;
        //icp.getFinalTransformation()得到的是Matrix4f不是Matrix4d
        best_matrix = icp.getFinalTransformation().cast<double>();
        count = i;
        ROS_WARN("here");
        pcl::toROSMsg(Final, output);
        output.header.stamp = ros::Time::now();
        output.header.frame_id = "/map";
        corr_pub.publish(output);
    }
    ROS_WARN("R here");
    std::cout << "has converged: " << icp.hasConverged() <<std::endl;

 }
  /********records the best rotation to match********/
  std::cout<<"count"<<count<<std::endl;

  rotation <<std::cos((15 * count * M_PI) / 180.0), -1 * std::sin((15 * count * M_PI) / 180.0) ,0
            ,std::sin((15 * count * M_PI) / 180.0), std::cos((15 * count * M_PI) / 180.0)      ,0
            ,0          , 0                ,1;


  ro = rotation;
  r2 = best_matrix.block<3,3>(0,0);

  for(int i=0; i < cloud_icp->points.size(); i++)
  {
    trans<<scan_cloud->points[i].x,scan_cloud->points[i].y,scan_cloud->points[i].z;
    trans = ro * trans + init_point_last;
    trans = r2 * trans;

    cloud_icp->points[i].x = trans(0) + best_matrix(0,3);
    cloud_icp->points[i].y = trans(1) + best_matrix(1,3);
    cloud_icp->points[i].z = trans(2) + best_matrix(2,3);

  }
  inital_quaternion = r2 *ro;

  inital_guess.pose.position.x = trans(0) + best_matrix(0,3);
  inital_guess.pose.position.y = trans(1) + best_matrix(1,3);
  inital_guess.pose.position.z = trans(2) + best_matrix(2,3);

  inital_guess.pose.orientation.w = inital_quaternion.w();
  inital_guess.pose.orientation.x = inital_quaternion.x();
  inital_guess.pose.orientation.y = inital_quaternion.y();
  inital_guess.pose.orientation.z = inital_quaternion.z();

  init_pub.publish(inital_guess);
  pcl::toROSMsg(*cloud_icp, output);
  output.header.stamp = ros::Time::now();
  output.header.frame_id = "/map";
  ROS_WARN("are here ?");
  corr_pub.publish(output);
  std::cout << "has converged: " << icp.hasConverged() <<std::endl;
}

int main (int argc, char **argv)
{
  ros::init (argc, argv, "init_guess");
  ros::NodeHandle nh;

  ros::Subscriber map_sub = nh.subscribe<sensor_msgs::PointCloud2>("/map_part_cloud", 100, map_data);
  ros::Subscriber gps_sub = nh.subscribe<geometry_msgs::PointStamped>("/fix", 100, gps_data);
  ros::Subscriber scanline_sub = nh.subscribe<sensor_msgs::PointCloud2>("/lidar_points",100,show_scan);

  scan_pub = nh.advertise<sensor_msgs::PointCloud2> ("scan_cloud", 1);
  corr_pub = nh.advertise<sensor_msgs::PointCloud2> ("corr_cloud", 1);
  init_pub = nh.advertise<geometry_msgs::PoseStamped> ("init_guess", 1);

  std::cout<<"init_point_last"<<init_point_last(0)<<std::endl;
  if ((scan_cloud->points.size()>0) && (icp_flag == true) && (map_icp->size() >0))
  {
    std::vector<int> scan_indices;
    pcl::removeNaNFromPointCloud(*scan_cloud, *scan_cloud, scan_indices);
    *cloud_icp =  *scan_cloud;
    std::cout<<"init_point_last"<<init_point_last.transpose()<<std::endl;
    solve_icp(scan_cloud,map_icp,init_point_last);
    icp_flag = false;
  }

  ros::spin();

  return 0;
}


