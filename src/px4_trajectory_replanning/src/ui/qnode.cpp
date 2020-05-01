#include "px4_trajectory_replanning/ui/qnode.h"

using namespace px4_trajectory_replanning;

QNode::QNode(int argc, char** argv) : init_argc_(argc), init_argv_(argv)
{
  debug_ = true;
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
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
  ros::init(init_argc_, init_argv_, "px4_interface_node");

  if (!ros::master::check())
  {
    return false;
  }

  ros::start();  // explicitly needed since our nodehandle is going out of scope.

  queue_thread = boost::thread(boost::bind(&QNode::queueThread, this));

  ros::NodeHandle ros_node;

  get_mavstate_client = ros_node.serviceClient<px4_trajectory_replanning::GetMAV_STATE>("controllers/get_mavstate");
  request_command_client = ros_node.serviceClient<px4_trajectory_replanning::MAV_CONTROLLER_COMMAND>("controllers/"
                                                                                                     "offboard_"
                                                                                                     "command");
  mission_command_client = 
  ros_node.serviceClient<px4_trajectory_replanning::MAV_MISSION_COMMAND>("controllers/mission_command_param");

  px4_trajectory_replanning::MAV_MISSION_COMMAND mission_cmd;
  mission_cmd.request.request_param = true;

  if (mission_command_client.exists())
    if (mission_command_client.call(mission_cmd))
    {
      ROS_DEBUG_STREAM_COND(debug_, "CONTROLLER UI : Parameter Updated");
      if (mission_cmd.response.response)
        emit updateMissionParam(mission_cmd.response.config);
    }
  else
    ROS_DEBUG_STREAM_COND(debug_, "CONTROLLER UI : Failed to call service Mission Command");

  start();

  return true;
}

void QNode::queueThread()
{
  ros::NodeHandle nh;
  ros::CallbackQueue callback_queue;
  nh.setCallbackQueue(&callback_queue);

  ros::WallDuration duration(0.001);
  while (nh.ok())
    callback_queue.callAvailable(duration);
}

void QNode::getParam()
{
  px4_trajectory_replanning::GetMAV_STATE srv;
  if (get_mavstate_client.call(srv))
  {
    MavState state;
    state.insert(std::pair<int, int>(QTOLState::QSTATE_OFFBOARD, BOOL2INT(srv.response.controller_state.offboard)));
    state.insert(std::pair<int, int>(QTOLState::QSTATE_ROTOR, BOOL2INT(srv.response.controller_state.mav_state.armed)));
    state.insert(std::pair<int, int>(QTOLState::QSTATE_TOL, srv.response.controller_state.tol_state));
    emit updateUI(state);
  }
  else
  {
    ROS_ERROR("Failed to call service GetMavState");
  }
}

void QNode::sendingControllerCommand(ReqControllerCMD req)
{
  px4_trajectory_replanning::MAV_CONTROLLER_COMMAND srv;

  switch (req)
  {
    case REQ_CONTROLLER_LAND:
    {
      srv.request.mode_req = OffbCMD::TOL_CMD_LAND;
      break;
    }

    case REQ_CONTROLLER_TAKEOFF:
    {
      srv.request.mode_req = OffbCMD::TOL_CMD_TAKEOFF;

      break;
    }

    case REQ_CONTROLLER_OFFBOARD:
    {
      srv.request.mode_req = OffbCMD::TOL_CMD_OFFBOARD;
      break;
    }

    case REQ_CONTROLLER_ROTOR_ARMED:
    {
      srv.request.mode_req = OffbCMD::TOL_CMD_ROTOR_ARM;
      srv.request.set_rotor = true;
      break;
    }

    case REQ_CONTROLLER_ROTOR_DISARMED:
    {
      srv.request.mode_req = OffbCMD::TOL_CMD_ROTOR_ARM;
      srv.request.set_rotor = false;
      break;
    }

    case REQ_CONTROLLER_HOLD:
    {
      srv.request.mode_req = OffbCMD::TOL_CMD_HOLD;
      break;
    }
    case REQ_CONTROLLER_MISSION:
    {
      srv.request.mode_req = OffbCMD::TOL_CMD_MISSION;
      break;
    }
  }

  if (request_command_client.exists())
  {
    if (request_command_client.call(srv))
      ROS_INFO("REQUEST SENT");
    else
      ROS_ERROR("Failed to call service Request Command");
  }
}

void QNode::sendingMissionCommand(Parameter param)
{
  px4_trajectory_replanning::MAV_MISSION_COMMAND srv;

  switch (param.mission_req)
  {
    case REQ_MISSION_START:
    {
      srv.request.mission_command = MissionCMD::MISSION_START;
      break;
    }

    case REQ_MISSION_STOP:
    {
      srv.request.mission_command = MissionCMD::MISSION_STOP;
      break;
    }

    case REQ_MISSION_HOLD:
    {
      srv.request.mission_command = MissionCMD::MISSION_HOLD;
      break;
    }

    case REQ_MISSION_GET_PARAM:
    {
      srv.request.request_param = true;
      srv.request.set_param = false;
      break;
    }

    case REQ_MISSION_SET_PARAM:
    {
      srv.request.request_param = false;
      srv.request.set_param = true;
      srv.request.save = false;
      srv.request.config = param.config;
      break;
    }

    case REQ_MISSION_SAVE_PARAM:
    {
      srv.request.request_param = false;
      srv.request.set_param = true;
      srv.request.save = true;
      break;
    }
    
    case REQ_MISSION_RESET_PARAM:
    {
      srv.request.reset_param = true;
      break;
    }
  }
  srv.request.header.frame_id = "UI Interface";

  if (mission_command_client.exists())
  {
    if (mission_command_client.call(srv))
    {
      ROS_DEBUG_STREAM_COND(debug_, "CONTROLLER UI : Mission Command Sent");
      if (srv.response.response && param.mission_req == REQ_MISSION_GET_PARAM)
        emit updateMissionParam(srv.response.config);
    }
   }
  else
    ROS_DEBUG_STREAM_COND(debug_, "CONTROLLER UI : Failed to call service Mission Command");

  if (param.mission_req == REQ_MISSION_STOP)
  {
    ReqControllerCMD cmd = ReqControllerCMD::REQ_CONTROLLER_HOLD;
    sendingControllerCommand(cmd);
  }
}

void QNode::run()
{
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    getParam();

    ros::spinOnce();
    loop_rate.sleep();
  }

  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  emit rosShutdown();  // used to signal the gui for a shutdown (useful to roslaunch)
}
