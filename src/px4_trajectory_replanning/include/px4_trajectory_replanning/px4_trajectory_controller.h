#ifndef PX4_TRAJECTORY_CONTROLLER_H
#define PX4_TRAJECTORY_CONTROLLER_H

#include <thread>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <mutex>
#include <chrono>
#include <ctime>   // localtime
#include <sstream> // stringstream
#include <iomanip> // put_time
#include <string>  // string
#include <map>
#include <algorithm>
#include <math.h>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Eigen>


#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <ros/time.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/message_filter.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>

#include <px4_trajectory_replanning/Configuration.h>
#include <px4_trajectory_replanning/MAV_MISSION_COMMAND.h>
#include <px4_trajectory_replanning/POS_CONTROLLER_COMMAND.h>
#include <px4_trajectory_replanning/common.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <ewok/polynomial_3d_optimization.h>
#include <ewok/uniform_bspline_3d_optimization.h>
#include <ewok/uniform_bspline_3d.h>
#include <ewok/rrtstar3d.h>

template <typename T>
std::ostream& operator<<(std::ostream& out, const Eigen::Vector3d& v)
{
  {
    out << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")";
  }
  return out;
}

class TrajectoryController
{
public:

TrajectoryController(ros::NodeHandle & nh);

void spin();
void loadParam(const std::string path);
void saveParam();

void init();
void missionWaypoint();

void queueThread();
bool missionCommandParam(px4_trajectory_replanning::MAV_MISSION_COMMAND::Request &req,
                     px4_trajectory_replanning::MAV_MISSION_COMMAND::Response &res);
void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg);
void cameraInfoCallback(const sensor_msgs::CameraInfo & msg);
void RRTPublisher(const ros::TimerEvent& event);
void RRTProcess(const ros::TimerEvent& event);
void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);

private:

double dt;
int init_argc_;
char** init_argv_;
ros::NodeHandle nh_;

int mission_state, prev_mission_state;
bool running_allow;
bool running_hold;
bool core_debug, process_debug;
bool initialized;
bool mission_use_wp;
bool gazebo_sim;
std::string mission_map;
bool new_wp_initialized;
bool save_log, flat_height;

boost::thread queue_thread;
boost::mutex mutex;
ros::CallbackQueue callback_queue;

ros::Publisher occ_marker_pub, free_marker_pub, trajectory_pub, current_traj_pub, command_pt_pub, command_pt_viz_pub;
ros::Publisher traj_marker_pub, traj_checker_pub;
ros::Publisher rrt_property_pub, rrt_tree_pub, rrt_solution_pub;
ros::ServiceServer mission_command_server;
ros::ServiceClient mission_controller_cmd_client;
ros::Subscriber camera_info_sub_, robot_pos_subscriber;
message_filters::Subscriber<sensor_msgs::Image> depth_image_sub_;
tf::TransformListener listener;
image_transport::Subscriber sub_image;
//tf::MessageFilter<sensor_msgs::Image> * tf_filter_;
sensor_msgs::CameraInfo camera_info_msg_;
visualization_msgs::Marker traj_checker_marker;
std_msgs::ColorRGBA c_obs, c_free;


px4_trajectory_replanning::Configuration config;
std::string config_path, log_path, file_name;
YAML::Node mission_config;

ewok::PolynomialTrajectory3D<10, double>::Ptr traj;
ewok::EuclideanDistanceRingBuffer<6, int16_t, double>::Ptr edrb;
ewok::RRTStar3D<6, double>::Ptr path_planner;

};

#endif // PX4_MISSION_CONTROLLER_H
