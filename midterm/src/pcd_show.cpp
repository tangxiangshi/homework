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
#include <pcl/filters/passthrough.h>
#include <midterm/ncrl.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>
#include <queue>
#include <iostream>
#include <fstream>

/*
pcl::fromROSMsg(const sensor_msgs::PointCloud2 & ,	cloud,pcl::PointCloud< T >)
pcl::toROSMsg(const cloud,pcl::PointCloud< T > & , sensor_msgs::PointCloud2)
*/
const double perDegree = 10;
const double thres = 1.5;
geometry_msgs::PoseStamped inital_guess;
ros::Publisher map_pub,corr_pub,map_part_pub,current_pub,last_pub,predict_pub;
bool gps_flag = true,icp_flag = true;

pcl::VoxelGrid<PointType> downSizeCloud;
pcl::PassThrough<PointType> pass;
//set cube of size is 0.5cm^3
double voxel_size = 1.0;


Eigen::Vector3d init_point_last(0,0,0);
Eigen::Quaterniond inital_quaternion(1,0,0,0);

pcl::PointCloud<PointType>::Ptr update_cloud(new pcl::PointCloud<PointType>());
//一定要有new 因為要初始化他,配置一塊空的記憶體
pcl::PointCloud<PointType>::Ptr map_cloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr scan_cloud(new pcl::PointCloud<PointType>());

//以rt完的點雲
pcl::PointCloud<PointType>::Ptr cloud_icp(new pcl::PointCloud<PointType>());
//match的map cloud
pcl::PointCloud<PointType>::Ptr map_icp(new pcl::PointCloud<PointType>());

//f2f
pcl::PointCloud<PointType>::Ptr scanCloudLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr scanCloudCurrent(new pcl::PointCloud<PointType>());
std::queue<sensor_msgs::PointCloud2ConstPtr> scanCloudBuf;
std::mutex mBuf;

//base_link
Eigen::Vector3d lidar_imu_t(0.46,0,3.46);
Eigen::Quaterniond lidar_imu_r(-0.0051505,0.018102,-0.019207,0.99964);

//rocord
std::queue<double> t_x,t_y,t_z;
std::queue<double> r_x,r_y,r_z;
std::ofstream myfile;
Eigen::Vector3d eulerAngle;
//要在開gps_transoform
void gps_data(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  if(((msg->point.x * msg->point.x + msg->point.y * msg->point.x + msg->point.z * msg->point.z) != 0) && gps_flag == true)
  {
    init_point_last(0) = msg->point.x;
    init_point_last(1) = msg->point.y;
    init_point_last(2) = msg->point.z;
    gps_flag = false;
  }
}

void show_map(pcl::PointCloud<PointType>::Ptr map_cloud)
{
  Eigen::Vector3d trans;
  sensor_msgs::PointCloud2 output,output1;
  std::vector<int> map_indices;

  pcl::io::loadPCDFile("/home/ee405423/homework/src/map/map.pcd", *map_cloud);

  pcl::removeNaNFromPointCloud(*map_cloud, *map_cloud, map_indices);

  //要放pointer adress
  downSizeCloud.setInputCloud(map_cloud);
  downSizeCloud.setLeafSize(voxel_size, voxel_size, voxel_size);
  downSizeCloud.filter(*map_cloud);

  *map_icp = *map_cloud;
//  pass.setFilterLimitsNegative(false);
  //設置顯示範圍大小(看能不能y方向也濾一點)
  pass.setInputCloud(map_icp);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(init_point_last(0)-80,init_point_last(0)+80);
  pass.filter(*map_icp);

  pass.setFilterFieldName("y");
  pass.setFilterLimits(init_point_last(1)-80, init_point_last(1)+80);
  //pass.setFilterLimitsNegative(true);
  pass.filter(*map_icp);

//  pass.setFilterFieldName("z");
//  pass.setFilterLimits(init_point_last(2)+0.1,init_point_last(2)+120);
//  pass.filter(*map_icp);

  pcl::toROSMsg(*map_icp, output1);
  output1.header.stamp = ros::Time::now();
  output1.header.frame_id = "/map";
  map_part_pub.publish(output1);


  pcl::toROSMsg(*map_cloud, output);
  output.header.stamp = ros::Time::now();
  output.header.frame_id = "/map";
  map_pub.publish(output);
}

//確認要處理的是PCL還是Sensor的型態
void scanCloudHandler(const sensor_msgs::PointCloud2ConstPtr &scanCloud)
{  mBuf.lock();
  scanCloudBuf.push(scanCloud);
  mBuf.unlock();
}

void solve_icp(pcl::PointCloud<PointType>::Ptr scan_cloud, pcl::PointCloud<PointType>::Ptr map_cloud, Eigen::Vector3d init_point_last)
{
  //count->records best matrix
  int count = 0;
  double max_socre = 999999.0;
  Eigen::Vector3d trans,trans_;
  sensor_msgs::PointCloud2 output;
  pcl::PointCloud<PointType> Final;
  Eigen::Quaterniond ro(1,0,0,0) , r2(1,0,0,0);
  pcl::IterativeClosestPoint<PointType, PointType> icp;
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Matrix4d best_matrix = Eigen::Matrix4d::Identity();

  icp.setInputTarget(map_icp);

/*********************compute attitude every 15 degree******************/
  for(int i = 0 ;i < (360/perDegree); i++)
  {
    rotation <<std::cos((perDegree * i * M_PI) / 180.0), -1 * std::sin((perDegree * i * M_PI) / 180.0)  ,0
              ,std::sin((perDegree * i * M_PI) / 180.0), std::cos((perDegree * i * M_PI) / 180.0)       ,0
              ,0                               , 0                                      ,1;

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

    //std::cout <<"score : "<<icp.getFitnessScore() <<"max : "<<max_socre<<std::endl;
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
    //std::cout << "has converged: " << icp.hasConverged() <<std::endl;

 }
  /********records the best rotation to match********/
  std::cout<<"count"<<count<<std::endl;

  rotation <<std::cos((perDegree * count * M_PI) / 180.0), -1 * std::sin((perDegree * count * M_PI) / 180.0) ,0
            ,std::sin((perDegree * count * M_PI) / 180.0), std::cos((perDegree * count * M_PI) / 180.0)      ,0
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
  pcl::toROSMsg(*cloud_icp, output);

  inital_quaternion = r2 *ro;
  inital_guess.header.stamp = output.header.stamp;
  inital_guess.header.frame_id = "/map";

  //車子r 和 t
  inital_guess.pose.position.x = init_point_last(0) + trans_(0);
  inital_guess.pose.position.y = init_point_last(1) + trans_(1);
  inital_guess.pose.position.z = init_point_last(2) + trans_(2);

  inital_guess.pose.orientation.w = inital_quaternion.w();
  inital_guess.pose.orientation.x = inital_quaternion.x();
  inital_guess.pose.orientation.y = inital_quaternion.y();
  inital_guess.pose.orientation.z = inital_quaternion.z();
/*
  std::cout<<"solve w"<<inital_quaternion.w()<<std::endl;
  std::cout<<"solve x"<<inital_quaternion.x()<<std::endl;
  std::cout<<"solve y"<<inital_quaternion.y()<<std::endl;
  std::cout<<"solve z"<<inital_quaternion.z()<<std::endl;
*/
  output.header.stamp = ros::Time::now();
  output.header.frame_id = "/map";
  ROS_WARN("are here ?");
  corr_pub.publish(output);
  std::cout << "has converged: " << icp.hasConverged() <<std::endl;
}

/*
X_r_hat last轉到world or current轉到world
*/
void f2f_icp(pcl::PointCloud<PointType>::Ptr scanCloudCurrent,pcl::PointCloud<PointType>::Ptr scanCloudLast,geometry_msgs::PoseStamped &inital_guess)
{
    sensor_msgs::PointCloud2 output,Current_output,last_output;
    Eigen::Matrix4d T_f2f;
    Eigen::Vector3d  X_r_hat,best_trslation,trans,car_pose;
    Eigen::Quaterniond X_q_hat,best_matrix;
    pcl::PointCloud<PointType> aligned_f2f;
    pcl::IterativeClosestPoint<PointType, PointType> f2f_ICP;

    std::cout<<"scanCloudCurrent"<<scanCloudCurrent->points.size()<<std::endl;
    std::cout<<"scanCloudLast"<<scanCloudLast->points.size()<<std::endl;
    //f2f matching
    f2f_ICP.setInputSource(scanCloudCurrent);
    f2f_ICP.setInputTarget(scanCloudLast);
    f2f_ICP.align(aligned_f2f);

    pcl::toROSMsg(*scanCloudCurrent, Current_output);
    pcl::toROSMsg(*scanCloudLast, last_output);
    Current_output.header.stamp = last_output.header.stamp = ros::Time::now();
    Current_output.header.frame_id = last_output.header.frame_id = "/map";
    current_pub.publish(Current_output);
    last_pub.publish(last_output);

    T_f2f = f2f_ICP.getFinalTransformation().cast<double>();//data type change from float to double

    //std::cout<<"T_f2f"<<T_f2f<<std::endl;

    best_matrix = T_f2f.block<3,3>(0,0);
    //std::cout<<"T_f2f"<<best_matrix.toRotationMatrix()<<std::endl;
    best_trslation = T_f2f.block<3,1>(0,3);
    //std::cout<<"T_f2f"<<best_trslation.transpose()<<std::endl;

    X_r_hat << inital_guess.pose.position.x, inital_guess.pose.position.y, inital_guess.pose.position.z;
    X_q_hat.w() = inital_guess.pose.orientation.w;
    X_q_hat.x() = inital_guess.pose.orientation.x;
    X_q_hat.y() = inital_guess.pose.orientation.y;
    X_q_hat.z() = inital_guess.pose.orientation.z;

    std::cout<<"X_q_hat"<<X_q_hat.coeffs()<<std::endl;

    //predict car_pose
    //current 轉到 world
    //q = RP + t
    //X_r_hat = (best_matrix * X_r_hat) + best_trslation;//Xr_hat(k) = T_f2f(k) * Xr_hat(k)
    //X_q_hat = best_matrix.toRotationMatrix()*X_q_hat.toRotationMatrix();//Xq_hat(k) = R_f2f(k) * Xq_hat(k)
    X_q_hat = X_q_hat.toRotationMatrix() * best_matrix.toRotationMatrix();
    X_r_hat = X_r_hat + X_q_hat * best_trslation;
    eulerAngle = X_q_hat.toRotationMatrix().eulerAngles(2,1,0);

    //將lidar frame 轉到 imu frame
    car_pose = lidar_imu_r.inverse() * (X_r_hat - lidar_imu_t);
    inital_guess.pose.position.x = X_r_hat(0);
    inital_guess.pose.position.y = X_r_hat(1);
    inital_guess.pose.position.z = X_r_hat(2);

    inital_guess.pose.orientation.w = X_q_hat.w();
    inital_guess.pose.orientation.x = X_q_hat.x();
    inital_guess.pose.orientation.y = X_q_hat.y();
    inital_guess.pose.orientation.z = X_q_hat.z();

    init_point_last = X_r_hat.block<3,1>(0,0);
    std::cout<<"car_pose"<<car_pose.transpose()<<std::endl;
    std::cout<<"lidar_pose"<<init_point_last.transpose()<<std::endl;

    for(int k =0; k <scanCloudLast->points.size();k++)
    {
      //參考老師的講義
      trans << scanCloudLast->points[k].x,scanCloudLast->points[k].y,scanCloudLast->points[k].z;
      //要確認是不是X_q_hat * trans - X_r_hat
      trans =  X_q_hat * trans + X_r_hat;

      scanCloudLast->points[k].x = trans(0);
      scanCloudLast->points[k].y = trans(1);
      scanCloudLast->points[k].z = trans(2);

    }
    pcl::toROSMsg(*scanCloudLast,output);
    output.header.stamp = ros::Time::now();
    output.header.frame_id = "/map";
    corr_pub.publish(output);
}

//frmae to map
//scan_current 要從body轉到world
void f2m_icp(pcl::PointCloud<PointType>::Ptr scan_current,geometry_msgs::PoseStamped &inital_guess )
{
  sensor_msgs::PointCloud2 output;
  Eigen::Matrix4d T_f2m;
  Eigen::Vector3d  X_r_hat,best_trslation,trans;
  Eigen::Quaterniond X_q_hat , best_matrix;

  X_r_hat(0) = inital_guess.pose.position.x;
  X_r_hat(1) = inital_guess.pose.position.y;
  X_r_hat(2) = inital_guess.pose.position.z;

  X_q_hat.w() = inital_guess.pose.orientation.w;
  X_q_hat.x() = inital_guess.pose.orientation.x;
  X_q_hat.y() = inital_guess.pose.orientation.y;
  X_q_hat.z() = inital_guess.pose.orientation.z;

  //新增容器是為了不要讓他干擾,到下次的scan_clod的資料
  pcl::PointCloud<PointType>::Ptr scan_current_g (new pcl::PointCloud<PointType>());
  *scan_current_g = *scan_current;
  std::cout<<"scan_current_g"<<scan_current_g->points.size()<<std::endl;
  for(int k =0; k <scan_current_g->points.size();k++)
  {
    /*參考老師的講義*/
    trans << scan_current->points[k].x,scan_current->points[k].y,scan_current->points[k].z;
    //要確認是不是X_q_hat * trans - X_r_hat
    trans =  X_q_hat * trans + X_r_hat;

    scan_current_g->points[k].x = trans(0);
    scan_current_g->points[k].y = trans(1);
    scan_current_g->points[k].z = trans(2);
  }
    pcl::IterativeClosestPoint<PointType,PointType> f2m_icp;

    f2m_icp.setInputSource(scan_current_g);
    f2m_icp.setInputTarget(map_icp);
    pcl::PointCloud<PointType> aligned_f2m;
    f2m_icp.align(aligned_f2m);
    double max_socre = f2m_icp.getFitnessScore();
    std::cout<<"current score:"<<max_socre<<std::endl;

    if (max_socre < thres) {

      T_f2m = f2m_icp.getFinalTransformation().cast<double>();//data type change from float to double
      best_matrix = T_f2m.block<3,3>(0,0);
      best_trslation = T_f2m.block<3,1>(0,3);
      X_r_hat = best_trslation  +  best_matrix * X_r_hat;
      X_q_hat = best_matrix.toRotationMatrix() * X_q_hat.toRotationMatrix();

      //做完的點當成下次的inital guess
      inital_guess.pose.position.x = X_r_hat(0);
      inital_guess.pose.position.y = X_r_hat(1);
      inital_guess.pose.position.z = X_r_hat(2);

      inital_guess.pose.orientation.w = X_q_hat.w();
      inital_guess.pose.orientation.x = X_q_hat.x();
      inital_guess.pose.orientation.y = X_q_hat.y();
      inital_guess.pose.orientation.z = X_q_hat.z();


      for(int k =0; k < scan_current_g->points.size();k++)
      {
        //參考老師的講義
        trans << scan_current_g->points[k].x,scan_current_g->points[k].y,scan_current_g->points[k].z;
        //要確認是不是X_q_hat * trans - X_r_hat


        trans = best_matrix * trans + best_trslation;

        scan_current_g->points[k].x = trans(0);
        scan_current_g->points[k].y = trans(1);
        scan_current_g->points[k].z = trans(2);
      }

      for(int k =0; k < scan_current->points.size();k++)
      {
        //參考老師的講義
        trans << scan_current->points[k].x,scan_current->points[k].y,scan_current->points[k].z;
        //要確認是不是X_q_hat * trans - X_r_hat


        trans = best_matrix * trans + best_trslation;

        scan_current->points[k].x = trans(0);
        scan_current->points[k].y = trans(1);
        scan_current->points[k].z = trans(2);
      }


      pcl::toROSMsg(*scan_current_g,output);
      output.header.stamp = ros::Time::now();
      output.header.frame_id = "/map";
      predict_pub.publish(output);
    }
    *update_cloud = *scan_current;
}

int main (int argc, char **argv)
{
  ros::init (argc, argv, "init_guess");
  ros::NodeHandle nh;
  //subscribe 的size太小,所以他最多只能裝100筆,太稿笑了QQ
  ros::Subscriber gps_sub = nh.subscribe<geometry_msgs::PointStamped>("/fix", 500, gps_data);
  ros::Subscriber scanline_sub = nh.subscribe<sensor_msgs::PointCloud2>("/lidar_points",500,scanCloudHandler);

  map_pub = nh.advertise<sensor_msgs::PointCloud2> ("map_cloud", 1);
  corr_pub = nh.advertise<sensor_msgs::PointCloud2> ("corr_cloud", 1);
  map_part_pub = nh.advertise<sensor_msgs::PointCloud2> ("map_part_cloud", 1);
  current_pub = nh.advertise<sensor_msgs::PointCloud2> ("current_cloud", 1);
  last_pub = nh.advertise<sensor_msgs::PointCloud2> ("last_cloud", 1);
  predict_pub = nh.advertise<sensor_msgs::PointCloud2> ("predict_cloud", 1);

  bool flag_csv =true , flag_up = true;
  //std::cout<<"here1"<<std::endl;
  ros::Rate loop_rate(10);
  int count = 0;
  while(ros::ok)
  {
    //std::cout<<"here2"<<std::endl;
    show_map(map_cloud);
    //std::cout<<"here3"<<std::endl;

    if(!scanCloudBuf.empty())
    {
      pcl::fromROSMsg(*scanCloudBuf.front(), *scanCloudCurrent);

      //std::cout<<scanCloudLast->points.size()<<std::endl;
      std::cout<<"here4"<<std::endl;
      if((scanCloudCurrent->points.size()>0) && (icp_flag == true))
        {
          std::vector<int> scan_indices;
          pcl::removeNaNFromPointCloud(*scanCloudCurrent, *scanCloudCurrent, scan_indices);
          *cloud_icp =  *scanCloudCurrent;
          std::cout<<"init_point_last"<<init_point_last.transpose()<<std::endl;
          solve_icp(scanCloudCurrent,map_cloud,init_point_last);
          icp_flag = false;
        }
      //(scanCloudBuf.size()>1)
      if((icp_flag == false) && (scanCloudBuf.size() > 1.0))
      {
        std::vector<int> scan_indices_1 ,scan_indices_2;
        //std::cout<<"here5"<<std::endl;
        scanCloudLast->clear();
        scanCloudCurrent->clear();
        pcl::fromROSMsg(*scanCloudBuf.front(), *scanCloudLast);
//        pass.setFilterFieldName("z");
//        pass.setFilterLimits(init_point_last(2)+0.1,init_point_last(2)+120);
//        pass.filter(*scanCloudLast);
        pcl::removeNaNFromPointCloud(*scanCloudLast, *scanCloudLast, scan_indices_1);
        scanCloudBuf.pop();
        pcl::fromROSMsg(*scanCloudBuf.front(), *scanCloudCurrent);
        pcl::removeNaNFromPointCloud(*scanCloudCurrent, *scanCloudCurrent, scan_indices_2);
//        pass.setFilterFieldName("z");
//        pass.setFilterLimits(init_point_last(2)+0.1,init_point_last(2)+120);
//        pass.filter(*scanCloudCurrent);
        if(flag_up == true)
        {
          flag_up = false;
        }
        else
        {
          *scanCloudLast = *update_cloud;
        }
        f2f_icp(scanCloudCurrent,scanCloudLast,inital_guess);
        f2m_icp(scanCloudCurrent,inital_guess);
        std::cout<<"scanCloudBuf.size()"<<scanCloudBuf.size()<<std::endl;
        t_x.push(inital_guess.pose.position.x);
        t_y.push(inital_guess.pose.position.y);
        t_z.push(inital_guess.pose.position.z);

        r_x.push(eulerAngle(0));
        r_y.push(eulerAngle(1));
        r_z.push(eulerAngle(2));

        std::cout<<"flag_csv"<<flag_csv<<std::endl;
        if(scanCloudBuf.size() <= 1.0  && flag_csv == true)
        {
          std::cout<<"csv"<<std::endl;
          myfile.open ("/home/ee405423/Desktop/test.csv");
          std::cout<<"qt_x.size()"<<t_x.size()<<std::endl;
          int size_s = t_x.size();
          for(int q =0; q < size_s ;q++)
          {
            myfile << scanCloudBuf.front()->header.stamp.toSec() << ","
                   << t_x.front() <<","
                   << t_y.front() << ","
                   << t_z.front()<< ","
                   << r_x.front()<< ","
                   << r_y.front()<< ","
                   << r_z.front()<< ","
                   << count << ","
                   << "\n";
            /*std::cout<<t_x.front()<<"\t"<<t_y.front()<<"\t"<<t_z.front()<<"\t"<<r_x.front()
                     <<r_y.front()<<"\t"<<r_z.front()<<std::endl;*/
            t_x.pop();t_y.pop();t_z.pop();r_x.pop();r_y.pop();r_z.pop();
          }
          flag_csv = false;
          myfile.close();
        }
        count ++;
      }
      std::cout<<"data"<<inital_guess.pose.position.x<<"\t"<<inital_guess.pose.position.y<<"\t"
               <<inital_guess.pose.position.z<<"\t"<<eulerAngle(0)<<"\t"<<eulerAngle(1)<<"\t"<<eulerAngle(2)<<std::endl;
     }
      std::cout<<"!scanCloudBuf.empty()"<<!scanCloudBuf.empty()<<std::endl;
      ros::spinOnce();
      loop_rate.sleep();
    }
  return 0;
}


