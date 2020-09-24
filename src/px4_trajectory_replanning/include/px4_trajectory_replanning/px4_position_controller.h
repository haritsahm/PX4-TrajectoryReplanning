// Copyright (C) 2020 haritsahm
// 
// PX4-TrajectoryReplanning is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// PX4-TrajectoryReplanning is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with PX4-TrajectoryReplanning. If not, see <http://www.gnu.org/licenses/>.

#ifndef PX4_MOTION_CONTROLLER_H
#define PX4_MOTION_CONTROLLER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/conversions.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include "rotors_control/lee_position_controller.h"
#include "rotors_control/common.h"
#include "rotors_control/parameters_ros.h"
#include <ewok/uniform_bspline_3d.h>
#include <ewok/polynomial_3d_optimization.h>
#include <px4_trajectory_replanning/common.h>
#include <px4_trajectory_replanning/POS_CONTROLLER_COMMAND.h>
#include <px4_trajectory_replanning/GetPOS_CONTROLLER_STATE.h>
#include <px4_trajectory_replanning/MAV_MISSION_COMMAND.h>
#include <controller_msgs/FlatTarget.h>
#include <boost/chrono.hpp>
#include <boost/bind.hpp>
#include <eigen3/Eigen/Eigen>
#include <stdio.h>
#include <algorithm>
#include <math.h>
#include <yaml-cpp/yaml.h>

using namespace ewok;
using namespace tf;

enum TrajType {
  TRAJ_TAKEOFF = 21,
  TRAJ_LAND = 22,
  TRAJ_MISSION = 23
};

class MotionController
{
public:
    MotionController(ros::NodeHandle &nh);

    inline Eigen::Vector3d toEigen(const geometry_msgs::Point& p);
    bool commandCallback(px4_trajectory_replanning::POS_CONTROLLER_COMMAND::Request  &req,
             px4_trajectory_replanning::POS_CONTROLLER_COMMAND::Response &res);
    bool missionCommandCallback(px4_trajectory_replanning::MAV_MISSION_COMMAND::Request  &req,
             px4_trajectory_replanning::MAV_MISSION_COMMAND::Response &res);
    bool getControllerState(px4_trajectory_replanning::GetPOS_CONTROLLER_STATE::Request &req,
                                                   px4_trajectory_replanning::GetPOS_CONTROLLER_STATE::Response &res);
    void PointCallback(const geometry_msgs::PointConstPtr & point_msg);
    void mavPosCallback(const geometry_msgs::PoseStamped& msg);
    void mavtwistCallback(const geometry_msgs::TwistStamped& msg);

    void loadParam(std::string path);
    void pubRateCommands(const Eigen::Vector4d &cmd);
    void computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd);
    Eigen::Vector4d geometric_attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc, Eigen::Vector4d &curr_att);
    Eigen::Vector4d acc2quaternion(const Eigen::Vector3d vector_acc, double yaw);
    Eigen::Vector4d quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p);
    Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4d q);
    Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d R);
    Eigen::Vector3d matrix_hat_inv(const Eigen::Matrix3d &m);
    void followTraj();
    void generateTraj(Eigen::Vector3d start, Eigen::Vector3d tar, Eigen::Vector4d limit);

    void followBSpline();
    void spin();

private:

    std::string namespace_;

    //subscribers
    ros::Subscriber cmd_poly_sub_;
    ros::Subscriber position_sub_, odometry_sub_, mavtwistSub_;
    ros::Publisher referencePub_, yawPub_, pathPub_, bodyRatePub_;
    ros::ServiceServer get_pos_state_server;
    ros::ServiceServer cmd_req_server;
    ros::ServiceServer mission_command_server;

    ros::NodeHandle nh_;

    double dt; double time_elapsed; int traj_pt_counter;
    YAML::Node global_cofing, controller_config;
    rotors_control::LeePositionController lee_position_controller_;

    ewok::UniformBSpline3D<6, double>::Ptr b_spline_;
    ros::Time init_time;
    double last_yaw, targ_yaw;
    bool arrived;
    int controller_state; bool controller_ready;

    int controller_cmd; bool req_cmd_tol=false;
    bool request_hold = false;
    int mission_controller_state;
    bool mission_hold;
    bool flag_reset;
    ros::Time hold_stamp;

    Eigen::Vector3d g_;
    Eigen::Vector4d q_des;
    Eigen::Vector4d cmdBodyRate_; //{wx, wy, wz, Thrust}
    Eigen::Vector3d Kpos_, Kvel_, D_;
    Eigen::Vector3d a0, a1, tau;
    double tau_x, tau_y, tau_z;
    double Kpos_x_, Kpos_y_, Kpos_z_, Kvel_x_, Kvel_y_, Kvel_z_;
    double attctrl_tau_;
    double norm_thrust_const_, norm_thrust_offset_;
    double max_fb_acc_;
    double dx_, dy_, dz_;
    double mavYaw_;
    bool yaw_from_traj;
    double zrate_targ;

    nav_msgs::Odometry odom_;
    Eigen::Vector3d mavPos_, mavVel_, mavRate_, lastPos_;
    Eigen::Vector3d p_targ, v_targ, a_targ;
    Eigen::Quaterniond mavAtt_, lastAtt_;

    Eigen::Affine3d pose_;
    rotors_control::EigenOdometry odometry;
    Eigen::Vector3f offset_;
    bool bspline_allowed;

    ewok::PolynomialTrajectory3D<10>::Ptr traj;
    Eigen::Vector3d traj_tar;
    TrajType traj_type;


};

#endif // PX4_POSITION_CONTROLLER_H
