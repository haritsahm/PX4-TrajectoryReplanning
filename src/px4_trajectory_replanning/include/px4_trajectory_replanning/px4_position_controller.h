#ifndef PX4_POSITION_CONTROLLER_H
#define PX4_POSITION_CONTROLLER_H

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/conversions.h>

#include <ewok/uniform_bspline_3d.h>
#include <ewok/polynomial_3d_optimization.h>
#include <px4_trajectory_replanning/common.h>
#include <px4_trajectory_replanning/POS_CONTROLLER_COMMAND.h>
#include <px4_trajectory_replanning/GetPOS_CONTROLLER_STATE.h>
#include <controller_msgs/FlatTarget.h>
#include <boost/chrono.hpp>
#include <boost/bind.hpp>
#include <eigen3/Eigen/Eigen>
#include <stdio.h>
#include <algorithm>
#include <math.h>

using namespace ewok;
using namespace tf;

enum TrajType {
  TRAJ_TAKEOFF = 21,
  TRAJ_LAND = 22,
  TRAJ_MISSION = 23
};

class LeePositionController
{
public:
    LeePositionController(ros::NodeHandle &nh);

    inline Eigen::Vector3d toEigen(const geometry_msgs::Point& p);
    bool commandCallback(px4_trajectory_replanning::POS_CONTROLLER_COMMAND::Request  &req,
             px4_trajectory_replanning::POS_CONTROLLER_COMMAND::Response &res);
    bool missionCommandCallback(px4_trajectory_replanning::POS_CONTROLLER_COMMAND::Request  &req,
             px4_trajectory_replanning::POS_CONTROLLER_COMMAND::Response &res);
    bool getControllerState(px4_trajectory_replanning::GetPOS_CONTROLLER_STATE::Request &req,
                                                   px4_trajectory_replanning::GetPOS_CONTROLLER_STATE::Response &res);
    void PointCallback(const geometry_msgs::PointConstPtr & point_msg);
    void mavPosCallback(const geometry_msgs::PoseStamped& msg);

    void pubrefState();
    void followTraj();
    void generateTraj(Eigen::Vector3d start, Eigen::Vector3d tar, Eigen::Vector4d limit);
    void getTrajectoryPoint(double t, mav_msgs::EigenTrajectoryPoint& command_trajectory, bool & yaw_from_traj);

    void followBSpline();
    void spin();

private:

    std::string namespace_;

    //subscribers
    ros::Subscriber cmd_poly_sub_;
    ros::Subscriber position_sub_;
    ros::Publisher referencePub_;
    ros::ServiceServer get_pos_state_server;
    ros::ServiceServer cmd_req_server;
    ros::ServiceServer mission_controller_cmd_server;


    ros::NodeHandle nh_;

    double dt; double time_elapsed;


    ewok::UniformBSpline3D<6, double>::Ptr b_spline_;
    ros::Time init_time;
    double last_yaw;
    bool arrived;
    int controller_state; bool controller_ready;

    int controller_cmd; bool req_cmd_tol=false;
    bool request_hold = false;
    int mission_controller_state;
    bool mission_hold;

    nav_msgs::Odometry odom_;
    Eigen::Vector3d mavPos_, mavVel_, mavRate_, lastPos_;
    Eigen::Vector3d p_targ, v_targ, a_targ;
    Eigen::Quaterniond mavAtt_, lastAtt_;

    Eigen::Affine3d pose_;
    EigenOdometry odometry;
    Eigen::Vector3f offset_;

    ewok::PolynomialTrajectory3D<10>::Ptr traj;
    Eigen::Vector3d traj_tar;
    TrajType traj_type;


};

#endif // PX4_POSITION_CONTROLLER_H
