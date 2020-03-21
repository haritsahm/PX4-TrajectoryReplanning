#include "px4_trajectory_replanning/ui/qnode.h"

QNode::QNode(int argc, char** argv)
  : init_argc_(argc),
    init_argv_(argv)
{
  debug_ = false;

}

QNode::~QNode()
{
  if (ros::isStarted())
  {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init()
{
  ros::init(init_argc_, init_argv_, "px4_controller_node");

  if (!ros::master::check())
  {
    return false;
  }

  ros::start();  // explicitly needed since our nodehandle is going out of scope.

  ros::NodeHandle ros_node;

  start();

  return true;
}

void QNode::run()
{
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  emit rosShutdown();  // used to signal the gui for a shutdown (useful to roslaunch)
}
