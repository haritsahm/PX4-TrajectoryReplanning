/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <px4_trajectory_replanning/POS_Controller_State.h>
#include <px4_trajectory_replanning/POS_CONTROLLER_COMMAND.h>
#include <px4_trajectory_replanning/MAV_CONTROLLER_COMMAND.h>
#include <px4_trajectory_replanning/GetMAV_STATE.h>
#include <px4_trajectory_replanning/GetPOS_CONTROLLER_STATE.h>
#include <eigen3/Eigen/Eigen>

#include <iostream>
#include <sstream>

mavros_msgs::State current_state;
Eigen::Vector3d mavPos_;
Eigen::Quaterniond mavAtt_;

enum TOLState{TOL_STATE_LAND = 0, TOL_STATE_TAKEOFF=1};

enum ControllerREQ {
  TOL_REQ_LAND = 10,
  TOL_REQ_TAKEOFF = 11,
  TOL_CMD_OFFBOARD = 12,
  TOL_CMD_ROTOR_ARM = 13,
  TOL_CMD_HOLD_ALT = 14,
  TOL_CMD_MISSION = 15
};

ControllerREQ controller_cmd;

bool mode_offboard = false; bool req_offboard = false;
bool armed = false;
int tol_state = TOL_STATE_LAND; bool req_cmd_tol=false; int req_int_tol = 0;
bool cmd_takeoff = false; bool req_takeoff = false;
bool cmd_land = true; bool req_land = false;
bool rotor_state_req = false; bool req_rotor = false;

bool waiting_response =false;
bool debug = true;



px4_trajectory_replanning::POS_Controller_State pos_controller_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
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

bool commandReqCallback(px4_trajectory_replanning::MAV_CONTROLLER_COMMAND::Request  &req,
                        px4_trajectory_replanning::MAV_CONTROLLER_COMMAND::Response &res)
{
  req_cmd_tol = true;

  if(req.mode_req == req.PX4_CMD_LANDING)
    controller_cmd = TOL_REQ_LAND;
  else if(req.mode_req == req.PX4_CMD_TAKEOFF)
    controller_cmd = TOL_REQ_TAKEOFF;
  else if(req.mode_req == req.PX4_MODE_OFFBOARD)
    controller_cmd = TOL_CMD_OFFBOARD;
  else if(req.mode_req == req.PX4_SET_HOLD)
    controller_cmd = TOL_CMD_HOLD_ALT;
  else if(req.mode_req == req.PX4_SET_MISSION)
    controller_cmd = TOL_CMD_MISSION;
  else if(req.mode_req == req.PX4_SET_ROTOR)
  {
    controller_cmd = TOL_CMD_ROTOR_ARM;
    rotor_state_req = req.set_rotor;
  }

  res.success = true;

  return true;
}

void mavpos_cb(const geometry_msgs::PoseStamped& msg)
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

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
      ("mavros/state", 10, state_cb);
  ros::Subscriber mavpos_sub = nh.subscribe("/mavros/local_position/pose", 1,
                                            mavpos_cb);

  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
      ("mavros/setpoint_position/local", 10);

  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
      ("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
      ("mavros/set_mode");
  ros::ServiceClient cmd_takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
      ("mavros/cmd/takeoff");
  ros::ServiceClient cmd_land_client = nh.serviceClient<mavros_msgs::CommandTOL>
      ("mavros/cmd/land");
  ros::ServiceClient pos_controller_cmd_client = nh.serviceClient<px4_trajectory_replanning::POS_CONTROLLER_COMMAND>
      ("controllers/pos_controller_cmd");
  ros::ServiceClient pos_controller_state_client = nh.serviceClient<px4_trajectory_replanning::GetPOS_CONTROLLER_STATE>
      ("controllers/get_pos_controller_state");

  ros::ServiceServer cmd_req_server = nh.advertiseService("controllers/offboard_command", commandReqCallback);
  ros::ServiceServer get_mavstate_server = nh.advertiseService("controllers/get_mavstate", getParameterCallback);

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  // wait for FCU connection
  while(ros::ok() && !current_state.connected){
    ros::spinOnce();
    rate.sleep();
  }

  debug = true;

  ROS_INFO_COND(debug, "READY TO START");


  ros::Time last_request = ros::Time::now();

  while(ros::ok()){

    if(req_cmd_tol)
    {
      switch (controller_cmd) {
      case TOL_CMD_OFFBOARD:
      {
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(3.0)))
        {
          if( set_mode_client.call(offb_set_mode) &&
              offb_set_mode.response.mode_sent)
          {
            mode_offboard = true;
            ROS_DEBUG_STREAM_COND(debug, "MAVROS CONTROLLER : OFFBOARD ENABLED");
          }
        }
        else if(current_state.mode == "OFFBOARD")
          ROS_DEBUG_STREAM_COND(debug, "MAVROS CONTROLLER : ALREADY IN OFFBOARD MODE");
        break;
      }

      case TOL_REQ_TAKEOFF:
      {
        px4_trajectory_replanning::POS_CONTROLLER_COMMAND pos_cmd_srv;
        if(pos_controller_state.controller_ready)
        {
          pos_cmd_srv.request.cmd_req = pos_cmd_srv.request.CMD_TAKEOFF;
          if(pos_controller_cmd_client.call(pos_cmd_srv))
            ROS_DEBUG_STREAM_COND(debug, "MAVROS CONTROLLER : Request TOL to Takeoff");
          else
            ROS_DEBUG_STREAM_COND(debug, "MAVROS CONTROLLER : Cant Call POS Command Service");
        }
        else
          ROS_WARN_COND(debug, "MAVROS CONTROLLER : POS Controller Not Ready");
        break;
      }

      case TOL_REQ_LAND:
      {
        px4_trajectory_replanning::POS_CONTROLLER_COMMAND pos_cmd_srv;
        if(pos_controller_state.controller_ready)
        {
          pos_cmd_srv.request.cmd_req = pos_cmd_srv.request.CMD_LANDING;
          if(pos_controller_cmd_client.call(pos_cmd_srv))
            ROS_DEBUG_STREAM_COND(debug, "MAVROS CONTROLLER : Request TOL to Land");
          else
            ROS_DEBUG_STREAM_COND(debug, "MAVROS CONTROLLER : Cant Call POS Command Service");
        }
        else
          ROS_WARN_COND(debug, "MAVROS CONTROLLER : POS Controller Not Ready");
        break;
      }

      case TOL_CMD_ROTOR_ARM:
      {
        ROS_DEBUG_STREAM_COND(debug, "MAVROS CONTROLLER : Request ROTOR" << std::boolalpha << rotor_state_req);
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = rotor_state_req;

        if( current_state.armed != rotor_state_req &&
            (ros::Time::now() - last_request > ros::Duration(3.0))){
          if( arming_client.call(arm_cmd) &&
              arm_cmd.response.success){
            ROS_INFO("ARM sent %d", arm_cmd.response.success);
            ROS_DEBUG_STREAM_COND(debug, "MAVROS CONTROLLER : ROTOR " << rotor_state_req);
          }
        }
        break;
      }

      case TOL_CMD_HOLD_ALT:
      {
        px4_trajectory_replanning::POS_CONTROLLER_COMMAND pos_cmd_srv;
        if(pos_controller_state.controller_ready)
        {
          pos_cmd_srv.request.cmd_req = pos_cmd_srv.request.CMD_HOLD_ALT;
          if(pos_controller_cmd_client.call(pos_cmd_srv))
            ROS_DEBUG_STREAM_COND(debug, "MAVROS CONTROLLER : Request TOL to Hold Alt");
          else
            ROS_DEBUG_STREAM_COND(debug, "MAVROS CONTROLLER : Cant Call POS Command Service");
        }
        else
          ROS_WARN_COND(debug, "MAVROS CONTROLLER : POS Controller Not Ready");
        break;
      }

      case TOL_CMD_MISSION:
      {
        px4_trajectory_replanning::POS_CONTROLLER_COMMAND pos_cmd_srv;
        if(pos_controller_state.controller_ready)
        {
          pos_cmd_srv.request.cmd_req = pos_cmd_srv.request.CMD_MISSION_FOLLOW;
          if(pos_controller_cmd_client.call(pos_cmd_srv))
            ROS_DEBUG_STREAM_COND(debug, "MAVROS CONTROLLER : Request TOL to Follow Mission");
          else
            ROS_DEBUG_STREAM_COND(debug, "MAVROS CONTROLLER : Cant Call POS Command Service");
        }
        else
          ROS_WARN_COND(debug, "MAVROS CONTROLLER : POS Controller Not Ready");
        break;
      }

      }

      last_request = ros::Time::now();
      req_cmd_tol = false;
    }

    // Get POS Controller State
    px4_trajectory_replanning::GetPOS_CONTROLLER_STATE pos_state_req;
    if(pos_controller_state_client.call(pos_state_req))
      pos_controller_state = pos_state_req.response.controller_state;
    else
      ROS_WARN_COND(debug, "MAVROS CONTROLLER : Cant Call POS Controller");




    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
