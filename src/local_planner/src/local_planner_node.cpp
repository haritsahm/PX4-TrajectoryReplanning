#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "local_planner_node");
  ros::NodeHandle n;
  ros::Rate rate(120);

  while(ros::ok())
  {

     ros::spinOnce();
     rate.sleep();
  }

  return 0;
}
