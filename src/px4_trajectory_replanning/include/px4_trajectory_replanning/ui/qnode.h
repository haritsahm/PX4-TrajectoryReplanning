#ifndef QNODE_H
#define QNODE_H

#ifndef Q_MOC_RUN

#include <QThread>
#include <stdio.h>
#include <string>
#include <boost/thread.hpp>
#include <map>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <px4_trajectory_replanning/GetMAV_STATE.h>
#include <px4_trajectory_replanning/MAV_CONTROLLER_COMMAND.h>
#include <px4_trajectory_replanning/MAV_MISSION_COMMAND.h>
#include <px4_trajectory_replanning/Configuration.h>
#include <px4_trajectory_replanning/common.h>


#endif

#define BOOL2INT(x) (x ? 1 : 0)

//std::map<int, std::string> mav_state_key;

//mav_state_key.insert(std::make_pair(0, "MAV_STATE_UNINIT"));
//mav_state_key.insert(std::make_pair(1, "MAV_STATE_BOOT"));
//mav_state_key.insert(std::make_pair(2, "MAV_STATE_CALIBRATING"));
//mav_state_key.insert(std::make_pair(3, "MAV_STATE_STANDBY"));
//mav_state_key.insert(std::make_pair(4, "MAV_STATE_ACTIVE"));
//mav_state_key.insert(std::make_pair(5, "MAV_STATE_CRITICAL"));
//mav_state_key.insert(std::make_pair(6, "MAV_STATE_EMERGENCY"));
//mav_state_key.insert(std::make_pair(7, "MAV_STATE_POWEROFF"));
//mav_state_key.insert(std::make_pair(8, "MAV_STATE_FLIGHT_TERMINATION"));

//mav_state_key[0] = "MAV_STATE_UNINIT";
//mav_state_key[1] = "MAV_STATE_BOOT";
//mav_state_key[2] = "MAV_STATE_CALIBRATING";
//mav_state_key[3] = "MAV_STATE_STANDBY";
//mav_state_key[4] = "MAV_STATE_ACTIVE";
//mav_state_key[5] = "MAV_STATE_CRITICAL";
//mav_state_key[6] = "MAV_STATE_EMERGENCY";
//mav_state_key[7] = "MAV_STATE_POWEROFF";
//mav_state_key[8] = "MAV_STATE_FLIGHT_TERMINATION";



struct MavState
{
    bool offboard_state;
    bool rotor_state;
    bool tol_state;
    QString mode;
    QString system_status;
};

enum QTOLState{
  QTOL_STATE_LAND = 40,
  QTOL_STATE_TAKEOFF= 41,
  QSTATE_OFFBOARD = 40,
  QSTATE_ROTOR = 41,
  QSTATE_TOL = 42
};

enum ReqControllerCMD{
  REQ_CONTROLLER_TAKEOFF=51,
  REQ_CONTROLLER_LAND=52,
  REQ_CONTROLLER_OFFBOARD=53,
  REQ_CONTROLLER_ROTOR_ARMED=54,
  REQ_CONTROLLER_ROTOR_DISARMED=55,
  REQ_CONTROLLER_HOLD=56,
  REQ_CONTROLLER_MISSION=57
};

enum ReqMissionCMD{
  REQ_MISSION_SAVE_PARAM=60,
  REQ_MISSION_GET_PARAM=61,
  REQ_MISSION_SET_PARAM=62,
  REQ_MISSION_START=63,
  REQ_MISSION_STOP=64,
  REQ_MISSION_HOLD=65,
  REQ_MISSION_RESET_PARAM=66
};

struct Parameter
{
  px4_trajectory_replanning::Configuration config;
  bool save;
  bool getParam;
  ReqMissionCMD mission_req;
};

class QNode : public QThread
{
  Q_OBJECT

public:
  QNode(int argc, char** argv);
  virtual ~QNode();

  bool init();
  void run();
  void getParam();
  void queueThread();

  std::map<int, std::string> mav_state_key{
      {0, "MAV_STATE_UNINIT"},
      {1, "MAV_STATE_BOOT"},
      {2, "MAV_STATE_CALIBRATING"},
      {3, "MAV_STATE_STANDBY"},
      {4, "MAV_STATE_ACTIVE"},
      {5, "MAV_STATE_CRITICAL"},
      {6, "MAV_STATE_EMERGENCY"},
      {7, "MAV_STATE_POWEROFF"},
      {8, "MAV_STATE_FLIGHT_TERMINATION"}
  };

public slots:
  void sendingControllerCommand(ReqControllerCMD req);
  void sendingMissionCommand(Parameter param);

signals:
  void rosShutdown();
  void updateUI(MavState state);
  void updateMissionParam(px4_trajectory_replanning::Configuration config);

private:
  int init_argc_;
  char** init_argv_;
  bool debug_;
  boost::thread queue_thread;

  ros::ServiceClient get_mavstate_client, request_command_client;
  ros::ServiceClient mission_command_client, mission_pos_cmd_client;
  px4_trajectory_replanning::Configuration config;


};

#endif // QNODE_H
