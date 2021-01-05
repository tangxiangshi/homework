#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <iostream>
#include <queue>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include "midterm/common.h"

sensor_msgs::PointCloud2 container;
geometry_msgs::PoseStamped init_guess_rt;
std::queue<geometry_msgs::PoseStamped::ConstPtr> correctPoseLastBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> scanCloudBuf;
//std::queue<sensor_msgs::PointCloud2ConstPtr> mapCloudBuf;
std::mutex mBuf;
ros::Publisher predictPoseCurrent_pub;
geometry_msgs::PoseStamped correctPoseLast;
bool systemInit = true;
double timeScan = 0;
double timeCorrectPose = 0;
pcl::PointCloud<PointType>::Ptr scanCloudLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr scanCloudCurrent(new pcl::PointCloud<PointType>());

void correctPoseLastHandler(const geometry_msgs::PoseStamped::ConstPtr &correctPoseLast) {
    mBuf.lock();
    correctPoseLastBuf.push(correctPoseLast);
    mBuf.unlock();
}

void scanCloudHandler(const sensor_msgs::PointCloud2ConstPtr &scanCloud) {
    mBuf.lock();
    scanCloudBuf.push(scanCloud);
    mBuf.unlock();
}

/*void mapCloudHandler(const sensor_msgs::PointCloud2ConstPtr &mapCloud) {
    mBuf.lock();
    mapCloudBuf.push(mapCloud);
    mBuf.unlock();
}*/

void f2f_icp(pcl::PointCloud<PointType>::Ptr scanCloudCurrent,pcl::PointCloud<PointType>::Ptr scanCloudLast,const geometry_msgs::PoseStamped &correctPoseLast)
{
    //f2f matching
    pcl::IterativeClosestPoint<PointType, PointType> f2f_ICP;
    f2f_ICP.setInputSource (scanCloudLast);
    f2f_ICP.setInputTarget (scanCloudCurrent);
    pcl::PointCloud<PointType>::Ptr aligned_f2f (new pcl::PointCloud<PointType>());
    *aligned_f2f = *scanCloudLast;
    f2f_ICP.align (*aligned_f2f);

    //after matching
    Eigen::Matrix4d T_f2f;
    T_f2f = f2f_ICP.getFinalTransformation().cast<double>();//data type change from float to double
    Eigen::Vector4d  X_r_hat;
    Eigen::Quaterniond X_q_hat;
    X_r_hat<<correctPoseLast.pose.position.x,correctPoseLast.pose.position.y,correctPoseLast.pose.position.z,1;
    X_q_hat.w() = correctPoseLast.pose.orientation.w;
    X_q_hat.x() = correctPoseLast.pose.orientation.x;
    X_q_hat.y() = correctPoseLast.pose.orientation.y;
    X_q_hat.z() = correctPoseLast.pose.orientation.z;

    //predict car_pose
    X_r_hat = T_f2f *  X_r_hat;//Xr_hat(k) = T_f2f(k) * Xr_hat(k)
    X_q_hat = T_f2f.block<3,3>(0,0) *  X_q_hat;//Xq_hat(k) = R_f2f(k) * Xq_hat(k)

    /*********************************************Result*********************************************/
    //Find the current map required searching(advertise to F2M_manager)
    geometry_msgs::PoseStamped predictPoseCurrent;
    pcl::toROSMsg(*scanCloudCurrent,container);
    predictPoseCurrent.header.frame_id = "map";//Need to define a world and body frame in rviz
    predictPoseCurrent.header.stamp = container.header.stamp;//Need to use the timestamp of the current scan cloud
    predictPoseCurrent.pose.position.x = X_r_hat(0);
    predictPoseCurrent.pose.position.y = X_r_hat(1);
    predictPoseCurrent.pose.position.z = X_r_hat(2);
    predictPoseCurrent.pose.orientation.x = X_q_hat.x();
    predictPoseCurrent.pose.orientation.y = X_q_hat.y();
    predictPoseCurrent.pose.orientation.z = X_q_hat.z();
    predictPoseCurrent.pose.orientation.w = X_q_hat.w();
    ROS_WARN("pred");
    predictPoseCurrent_pub.publish(predictPoseCurrent);//for map_initialization
}
void process() {
    int detect = 0;
    while (true) {

        //Buffer with data will enter this while
        while (!correctPoseLastBuf.empty() && !scanCloudBuf.empty()) {
            mBuf.lock();

            //Ensure to F2F_ICP with "two point cloud"
            if (systemInit)
            {
                //lastCorrectPose = correctInitPoseBuf.front();
                scanCloudLast->clear();
                pcl::fromROSMsg(*scanCloudBuf.front(), *scanCloudLast);
                scanCloudBuf.pop();
                systemInit = false;
                detect++;
            }

            if (scanCloudBuf.empty())
            {
                mBuf.unlock();
                break;
            }

            timeScan = scanCloudBuf.front()->header.stamp.toSec();
            timeCorrectPose = correctPoseLast.header.stamp.toSec();
            if (timeScan != timeCorrectPose)
            {
                if (detect == 1)
                {
                    //ROS_INFO("Don't worry. Not time asynchronous");
                }
                //ROS_WARN("time scan %f correct %f  \n", timeScan, timeCorrectPose);
                //ROS_WARN("time asynchronous \n");
                mBuf.unlock();
                break;
            }

            //sensorMsg to pcl and ready to input F2F_ICP
            scanCloudCurrent->clear();
            pcl::fromROSMsg(*scanCloudBuf.front(), *scanCloudCurrent);
            scanCloudBuf.pop();
            correctPoseLast = *correctPoseLastBuf.front();
            mBuf.unlock();

            /********F2F_ICP**********/
            f2f_icp(scanCloudCurrent,scanCloudLast,correctPoseLast);

            /********Termination********/
            *scanCloudLast = *scanCloudCurrent;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "F2Fmanager");
    ros::NodeHandle n;
    //subscribe the initial_guess
    ros::Subscriber correctPoseLast = n.subscribe<geometry_msgs::PoseStamped>("init_guess", 10, correctPoseLastHandler);
    ros::Subscriber scanCloud = n.subscribe<sensor_msgs::PointCloud2>("scan_cloud", 10, scanCloudHandler);
    predictPoseCurrent_pub = n.advertise<geometry_msgs::PoseStamped>("/poseToFindMapRange",10);
    //subscribe map after preprocessing
    //ros::Subscriber mapCloud = n.subscribe<sensor_msgs::PointCloud2>("/map_pc2", 10, mapCloudHandler);

    //std::thread relocalProcess{process}; // OK
    std::thread predictProcess{process}; // OK
    ros::spin();
    return 0;
}

//void process() {
//    int detect = 0;
//    while (true) {

//        //Buffer with data will enter this while
//        while (!correctPoseBuf.empty() && !scanCloudBuf.empty() && !mapCloudBuf.empty()) {
//            mBuf.lock();

//            //Ensure to F2F_ICP with "two point cloud"
//            if (systemInit) {
//                //lastCorrectPose = correctInitPoseBuf.front();
//                scanCloudLast->clear();
//                pcl::fromROSMsg(*scanCloudBuf.front(), *scanCloudLast);
//                scanCloudBuf.pop();
//                systemInit = false;
//                detect++;
//            }
//            if (scanCloudBuf.empty()) {
//                mBuf.unlock();
//                break;
//            }

//            //map time must > scan time
//            while (!mapCloudBuf.empty()
//            && mapCloudBuf.front()->header.stamp.toSec() < scanCloudBuf.front()->header.stamp.toSec()) {
//                mapCloudBuf.pop();
//            }
//            if (mapCloudBuf.empty()) {
//                mBuf.unlock();
//                break;
//            }
//            timeScan = scanCloudBuf.front()->header.stamp.toSec();
//            timeCorrectPose = lastCorrectPose.header.stamp.toSec();
//            if (timeScan != timeCorrectPose) {
//                if (detect == 1) {
//                    ROS_INFO("Don't worry. Not time asynchronous");
//                }
//                ROS_WARN("time scan %f correct %f  \n", timeScan, timeCorrectPose);
//                ROS_WARN("time asynchronous \n");
//                mBuf.unlock();
//                break;
//            }

//            //sensorMsg to pcl and ready to input F2F_ICP
//            scanCloudCurrent->clear();
//            pcl::fromROSMsg(*scanCloudBuf.front(), *scanCloudCurrent);
//            scanCloudBuf.pop();
//            lastCorrectPose = correctPoseBuf.front();

//            /********F2F_ICP**********/
//            f2f_icp(scanCloudCurrent,scanCloudLast,lastCorrectPose);

//            /********Termination********/
//            scanCloudLast = scanCloudCurrent;
//        }
//    }
//}
