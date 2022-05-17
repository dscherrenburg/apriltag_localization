#include <ros/ros.h>

#include <nodelet/loader.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tag_world_node");

  ros::NodeHandle nh;

  ROS_INFO("Hello");
  ros::spin();
  return 0;
}
