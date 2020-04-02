#ifndef QNODE_H
#define QNODE_H

#ifndef Q_MOC_RUN

#include <QThread>
#include <stdio.h>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <px4_trajectory_replanning/GetMAV_STATE.h>
#include <px4_trajectory_replanning/MAV_CONTROLLER_COMMAND.h>
#include <px4_trajectory_replanning/GetMAV_MISSION_PARAM.h>
#include <px4_trajectory_replanning/SetMAV_MISSION_PARAM.h>
#include <px4_trajectory_replanning/MAV_MISSION_COMMAND.h>
#include <px4_trajectory_replanning/Configuration.h>
#include <px4_trajectory_replanning/common.h>


#endif

#define BOOL2INT(x) (x ? 1 : 0)

typedef std::map<int, int> MavState;

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
  REQ_MISSION_HOLD=65
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

  ros::ServiceClient get_mavstate_client, request_command_client;
  ros::ServiceClient mission_command_client, mission_command_param_client;
  px4_trajectory_replanning::Configuration config;


};

#endif // QNODE_H
