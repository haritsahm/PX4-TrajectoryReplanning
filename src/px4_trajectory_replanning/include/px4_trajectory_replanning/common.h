#ifndef COMMON_H
#define COMMON_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Eigen>

enum TOLState{
  TOL_STATE_LAND = 0,
  TOL_STATE_TAKEOFF=1
};

// From UI to offboard controller
enum OffbCMD {
  TOL_CMD_LAND = 10,
  TOL_CMD_TAKEOFF = 11,
  TOL_CMD_OFFBOARD = 12,
  TOL_CMD_ROTOR_ARM = 13,
  TOL_CMD_HOLD = 14,
  TOL_CMD_MISSION = 15
};

enum ControllerState{
  CONTROLLER_STATE_LAND = 0,
  CONTROLLER_STATE_TAKEOFF=1,
  CONTROLLER_STATE_HOLD = 2,
  CONTROLLER_STATE_MISSION_FOLLOW=3
};

// from offboard controller to position controller
enum POSControllerCMD {
  POS_CMD_LAND = 10,
  POS_CMD_TAKEOFF = 11,
  POS_CMD_HOLD = 12,
  POS_CMD_MISSION_FOLLOW = 13,
  POS_CMD_FOL_TRAJ = 14,
  POS_CMD_NONE = 15
};

struct EigenOdometry {
  EigenOdometry()
      : position(0.0, 0.0, 0.0),
        orientation(Eigen::Quaterniond::Identity()),
        velocity(0.0, 0.0, 0.0),
        angular_velocity(0.0, 0.0, 0.0) {};

  EigenOdometry(const Eigen::Vector3d& _position,
                const Eigen::Quaterniond& _orientation,
                const Eigen::Vector3d& _velocity,
                const Eigen::Vector3d& _angular_velocity) {
    position = _position;
    orientation = _orientation;
    velocity = _velocity;
    angular_velocity = _angular_velocity;
  };

  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d velocity; // Velocity is expressed in the Body frame!
  Eigen::Vector3d angular_velocity;
};

inline void eigenOdometryFromMsg(const nav_msgs::OdometryConstPtr& msg,
                                 EigenOdometry* odometry) {
  odometry->position = mav_msgs::vector3FromPointMsg(msg->pose.pose.position);
  odometry->orientation = mav_msgs::quaternionFromMsg(msg->pose.pose.orientation);
  odometry->velocity = mav_msgs::vector3FromMsg(msg->twist.twist.linear);
  odometry->angular_velocity = mav_msgs::vector3FromMsg(msg->twist.twist.angular);
}

#endif // COMMON_H
