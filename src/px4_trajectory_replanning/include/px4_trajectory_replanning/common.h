#ifndef COMMON_H
#define COMMON_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Eigen>
#include <iostream>

typedef std::map<int, std::string> Log;

enum TOLState
{
  TOL_STATE_LAND = 0,
  TOL_STATE_TAKEOFF = 1
};

// From UI to offboard controller
enum OffbCMD
{
  TOL_CMD_LAND = 10,
  TOL_CMD_TAKEOFF = 11,
  TOL_CMD_OFFBOARD = 12,
  TOL_CMD_ROTOR_ARM = 13,
  TOL_CMD_HOLD = 14,
  TOL_CMD_MISSION = 15
};

enum ControllerState
{
  CONTROLLER_STATE_LAND = 0,
  CONTROLLER_STATE_TAKEOFF = 1,
  CONTROLLER_STATE_HOLD = 2,
  CONTROLLER_STATE_MISSION_FOLLOW = 3
};

// from offboard controller to position controller
enum POSControllerCMD
{
  POS_CMD_LAND = 10,
  POS_CMD_TAKEOFF = 11,
  POS_CMD_HOLD = 12,
  POS_CMD_MISSION_FOLLOW = 13,
  POS_CMD_FOL_TRAJ = 14,
  POS_CMD_NONE = 15,

  // from mission controller to position controller
  POS_CMD_MISSION_START = 16,
  POS_CMD_MISSION_STOP = 17,
  POS_CMD_MISSION_HOLD = 18,
};

enum MissionCMD
{
  MISSION_START = 0,
  MISSION_STOP = 1,
  MISSION_HOLD = 2, 
  MISSION_PUB = 3
};

namespace YAML
{
template <>
struct convert<Eigen::Vector3d>
{
  static Node encode(const Eigen::Vector3d& rhs)
  {
    Node node;
    node.push_back(rhs.x());
    node.push_back(rhs.y());
    node.push_back(rhs.z());
    return node;
  }

  static bool decode(const Node& node, Eigen::Vector3d& rhs)
  {
    if (!node.IsSequence() || node.size() != 3)
    {
      return false;
    }

    rhs.x() = node[0].as<double>();
    rhs.y() = node[1].as<double>();
    rhs.z() = node[2].as<double>();
    return true;
  }
};

template <>
struct convert<Eigen::Vector4d>
{
  static Node encode(const Eigen::Vector4d& rhs)
  {
    Node node;
    node.push_back(rhs.x());
    node.push_back(rhs.y());
    node.push_back(rhs.z());
    node.push_back(rhs.w());
    return node;
  }

  static bool decode(const Node& node, Eigen::Vector4d& rhs)
  {
    if (!node.IsSequence() || node.size() != 3)
    {
      return false;
    }

    rhs.x() = node[0].as<double>();
    rhs.y() = node[1].as<double>();
    rhs.z() = node[2].as<double>();
    rhs.w() = node[3].as<double>();
    return true;
  }
};

}  // namespace YAML

#endif  // COMMON_H
