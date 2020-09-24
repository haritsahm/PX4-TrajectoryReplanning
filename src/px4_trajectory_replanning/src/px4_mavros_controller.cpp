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


/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <px4_trajectory_replanning/GetMAV_STATE.h>
#include <px4_trajectory_replanning/GetPOS_CONTROLLER_STATE.h>
#include <px4_trajectory_replanning/MAV_CONTROLLER_COMMAND.h>
#include <px4_trajectory_replanning/MAV_MISSION_COMMAND.h>
#include <px4_trajectory_replanning/POS_CONTROLLER_COMMAND.h>
#include <px4_trajectory_replanning/POS_Controller_State.h>
#include <px4_trajectory_replanning/common.h>
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>

#include <iostream>
#include <sstream>

mavros_msgs::State current_state;
Eigen::Vector3d mavPos_;
Eigen::Quaterniond mavAtt_;

int controller_cmd;

bool mode_offboard = false;
bool req_offboard = false;
bool armed = false;
int tol_state = TOLState::TOL_STATE_LAND;
bool req_cmd_tol = false;
int req_int_tol = 0;
bool cmd_takeoff = false;
bool req_takeoff = false;
bool cmd_land = true;
bool req_land = false;
bool rotor_state_req = false;
bool req_rotor = false;

bool waiting_response = false;
bool debug = true;

px4_trajectory_replanning::POS_Controller_State pos_controller_state;

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
  current_state = *msg;
}

bool getParameterCallback(px4_trajectory_replanning::GetMAV_STATE::Request &req,
                          px4_trajectory_replanning::GetMAV_STATE::Response &res)
{
  res.controller_state.mav_state = current_state;
  res.controller_state.offboard = mode_offboard;
  res.controller_state.tol_state = tol_state;

  return true;
}

bool commandReqCallback(px4_trajectory_replanning::MAV_CONTROLLER_COMMAND::Request &req,
                        px4_trajectory_replanning::MAV_CONTROLLER_COMMAND::Response &res)
{
  req_cmd_tol = true;

  controller_cmd = req.mode_req;
  rotor_state_req = req.set_rotor;

  res.success = true;

  return true;
}

void mavpos_cb(const geometry_msgs::PoseStamped &msg)
{
  geometry_msgs::Point p = msg.pose.position;
  mavPos_ = Eigen::Vector3d(p.x, p.y, p.z);
  mavAtt_.w() = msg.pose.orientation.w;
  mavAtt_.x() = msg.pose.orientation.x;
  mavAtt_.y() = msg.pose.orientation.y;
  mavAtt_.z() = msg.pose.orientation.z;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "px4_mavros_controller_node");
  ros::NodeHandle nh;

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber mavpos_sub = nh.subscribe("/mavros/local_position/pose", 1, mavpos_cb);
  ros::NodeHandle node_handle;
  ros::CallbackQueue callback_queue;
  node_handle.setCallbackQueue(&callback_queue);

  ros::ServiceClient arming_client = node_handle.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = node_handle.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  ros::ServiceClient cmd_takeoff_client = node_handle.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
  ros::ServiceClient cmd_land_client = node_handle.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
  ros::ServiceClient pos_controller_cmd_client =
      node_handle.serviceClient<px4_trajectory_replanning::POS_CONTROLLER_COMMAND>("controllers/pos_controller_cmd");
  ros::ServiceClient mission_command_client =
      node_handle.serviceClient<px4_trajectory_replanning::MAV_MISSION_COMMAND>("controllers/mission_command_param");
  ros::ServiceClient pos_controller_state_client =
      node_handle.serviceClient<px4_trajectory_replanning::GetPOS_CONTROLLER_STATE>("controllers/"
                                                                                    "get_pos_controller_state");

  ros::ServiceServer cmd_req_server = node_handle.advertiseService("controllers/offboard_command", commandReqCallback);
  ros::ServiceServer get_mavstate_server =
      node_handle.advertiseService("controllers/get_mavstate", getParameterCallback);

  ros::AsyncSpinner spinner(0, &callback_queue);
  spinner.start();
  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(100.0);

  // wait for FCU connection
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

  debug = true;

  ROS_INFO_COND(debug, "READY TO START");

  ros::Time last_request = ros::Time::now();

  while (ros::ok())
  {
    if (req_cmd_tol)
    {
      switch (controller_cmd)
      {
        case OffbCMD::TOL_CMD_OFFBOARD:
        {
          mavros_msgs::SetMode offb_set_mode;
          offb_set_mode.request.custom_mode = "OFFBOARD";

          if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
          {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
              mode_offboard = true;
              ROS_DEBUG_STREAM_COND(debug, "MAVROS CONTROLLER : OFFBOARD ENABLED");
            }
          }
          else if (current_state.mode == "OFFBOARD")
            ROS_DEBUG_STREAM_COND(debug, "MAVROS CONTROLLER : ALREADY IN OFFBOARD MODE");
          break;
        }

        case OffbCMD::TOL_CMD_TAKEOFF:
        {
          px4_trajectory_replanning::POS_CONTROLLER_COMMAND pos_cmd_srv;
          if (pos_controller_state.controller_ready)
          {
            pos_cmd_srv.request.cmd_req = POSControllerCMD::POS_CMD_TAKEOFF;
            if (pos_controller_cmd_client.call(pos_cmd_srv))
            {
                ROS_DEBUG_STREAM_COND(debug, "MAVROS CONTROLLER : Request TOL to Takeoff");
                tol_state = TOLState::TOL_STATE_TAKEOFF;
            }
            else
              ROS_DEBUG_STREAM_COND(debug, "MAVROS CONTROLLER : Cant Call POS Command Service");
          }
          else
            ROS_WARN_COND(debug, "MAVROS CONTROLLER : POS Controller Not Ready");
          break;
        }

        case OffbCMD::TOL_CMD_LAND:
        {
          px4_trajectory_replanning::POS_CONTROLLER_COMMAND pos_cmd_srv;
          if (pos_controller_state.controller_ready)
          {
            pos_cmd_srv.request.cmd_req = POSControllerCMD::POS_CMD_LAND;
            if (pos_controller_cmd_client.call(pos_cmd_srv))
            {
                ROS_DEBUG_STREAM_COND(debug, "MAVROS CONTROLLER : Request TOL to Land");
                tol_state = TOLState::TOL_STATE_LAND;
            }
            else
              ROS_DEBUG_STREAM_COND(debug, "MAVROS CONTROLLER : Cant Call POS Command Service");
          }
          else
            ROS_WARN_COND(debug, "MAVROS CONTROLLER : POS Controller Not Ready");
          break;
        }

        case OffbCMD::TOL_CMD_ROTOR_ARM:
        {
          ROS_DEBUG_STREAM_COND(debug, "MAVROS CONTROLLER : Request ROTOR" << std::boolalpha << rotor_state_req);
          mavros_msgs::CommandBool arm_cmd;
          arm_cmd.request.value = rotor_state_req;

          if (current_state.armed != rotor_state_req && (ros::Time::now() - last_request > ros::Duration(3.0)))
          {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
              ROS_INFO("ARM sent %d", arm_cmd.response.success);
              ROS_DEBUG_STREAM_COND(debug, "MAVROS CONTROLLER : ROTOR " << rotor_state_req);
            }
          }
          break;
        }

        case OffbCMD::TOL_CMD_HOLD:
        {
          px4_trajectory_replanning::POS_CONTROLLER_COMMAND pos_cmd_srv;
          if (pos_controller_state.controller_ready)
          {
            pos_cmd_srv.request.cmd_req = POSControllerCMD::POS_CMD_HOLD;
            if (pos_controller_cmd_client.call(pos_cmd_srv))
              ROS_DEBUG_STREAM_COND(debug, "MAVROS CONTROLLER : Request TOL to Hold Alt");
            else
              ROS_DEBUG_STREAM_COND(debug, "MAVROS CONTROLLER : Cant Call POS Command Service");
          }
          else
            ROS_WARN_COND(debug, "MAVROS CONTROLLER : POS Controller Not Ready");
          break;
        }

        case OffbCMD::TOL_CMD_MISSION:
        {
          px4_trajectory_replanning::POS_CONTROLLER_COMMAND pos_cmd_srv;
          px4_trajectory_replanning::MAV_MISSION_COMMAND mission_cmd_srv;
          if (pos_controller_state.controller_ready)
          {
            pos_cmd_srv.request.cmd_req = POSControllerCMD::POS_CMD_MISSION_FOLLOW;
            if (pos_controller_cmd_client.call(pos_cmd_srv))
              ROS_DEBUG_STREAM_COND(debug, "MAVROS CONTROLLER : Request TOL to Follow Mission");
            else
              ROS_DEBUG_STREAM_COND(debug, "MAVROS CONTROLLER : Cant Call POS Command Service");
          }
          else
            ROS_WARN_COND(debug, "MAVROS CONTROLLER : Mission Controller Not Ready");

          mission_cmd_srv.request.allowed = true;
          mission_cmd_srv.request.header.frame_id = "Offboard Controller";
          if (mission_command_client.exists())
            if (mission_command_client.call(mission_cmd_srv))
              ROS_DEBUG_STREAM_COND(debug, "MAVROS CONTROLLER : Allowed Mission to Run");
          else
            ROS_DEBUG_STREAM_COND(debug, "MAVROS CONTROLLER : Cant Call Mission Command Service");

          break;
        }

        default:
          break;
      }

      last_request = ros::Time::now();
      req_cmd_tol = false;
    }

    // Get POS Controller State
    px4_trajectory_replanning::GetPOS_CONTROLLER_STATE pos_state_req;
    if (pos_controller_state_client.call(pos_state_req))
      pos_controller_state = pos_state_req.response.controller_state;
    else
      ROS_WARN_COND(debug, "MAVROS CONTROLLER : Cant Call POS Controller");

    callback_queue.callAvailable(ros::WallDuration(0.001));
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
