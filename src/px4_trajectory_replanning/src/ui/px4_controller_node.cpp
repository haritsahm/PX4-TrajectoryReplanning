#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "px4_ui_controller_node");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");
}
