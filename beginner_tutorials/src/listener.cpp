#include "ros/ros.h"
#include "std_msgs/String.h"
 
 /**
  32  * This tutorial demonstrates simple receipt of messages over the ROS system.
  33  */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
ROS_INFO("I heard: [%s]", msg->data.c_str());
}
 
int main(int argc, char **argv)
{
 /**
  42    * The ros::init() function needs to see argc and argv so that it can perform
  43    * any ROS arguments and name remapping that were provided at the command line.
  44    * For programmatic remappings you can use a different version of init() which takes
  45    * remappings directly, but for most command-line programs, passing argc and argv is
  46    * the easiest way to do it.  The third argument to init() is the name of the node.
  47    *
  48    * You must call one of the versions of ros::init() before using any other
  49    * part of the ROS system.
  50    */
ros::init(argc, argv, "listener");

/**
  54    * NodeHandle is the main access point to communications with the ROS system.
  55    * The first NodeHandle constructed will fully initialize this node, and the last
  56    * NodeHandle destructed will close down the node.
  57    */
ros::NodeHandle n;

/**
  61    * The subscribe() call is how you tell ROS that you want to receive messages
  62    * on a given topic.  This invokes a call to the ROS
  63    * master node, which keeps a registry of who is publishing and who
  64    * is subscribing.  Messages are passed to a callback function, here
  65    * called chatterCallback.  subscribe() returns a Subscriber object that you
  66    * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
  67    * object go out of scope, this callback will automatically be unsubscribed from
  68    * this topic.
  69    *
  70    * The second parameter to the subscribe() function is the size of the message
  71    * queue.  If messages are arriving faster than they are being processed, this
  72    * is the number of messages that will be buffered up before beginning to throw
  73    * away the oldest ones.
  74    */
ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

/**
  78    * ros::spin() will enter a loop, pumping callbacks.  With this version, all
  79    * callbacks will be called from within this thread (the main one).  ros::spin()
  80    * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
  81    */
ros::spin();

  return 0;
}
