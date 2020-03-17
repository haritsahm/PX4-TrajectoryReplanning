#ifndef COMMON_H
#define COMMON_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Eigen>

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
