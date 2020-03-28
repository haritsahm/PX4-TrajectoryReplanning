#ifndef UI_CONTROLLER_H
#define UI_CONTROLLER_H

#include <QMainWindow>
#include <QWidget>
#include <QPixmap>
#include <QObject>

#include "px4_trajectory_replanning/ui_offboardcontroller.h"
#include "qnode.h"

#include <ros/package.h>
#include <stdio.h>
#include <string>

namespace Ui {
class OffboardController;
}

class OffboardController : public QWidget
{
  Q_OBJECT

public:
  explicit OffboardController(int argc, char** argv, QWidget *parent = 0);
  ~OffboardController();

public slots:
  void on_mode_mission_pressed();
  void on_mode_hold_pressed();
  void on_mode_landing_pressed();
  void on_mode_takeoff_pressed();
  void on_mode_offboard_pressed();
  void on_button_arm_vehicle_pressed();

  void setMavState(MavState state);
  void setMissionParam(px4_trajectory_replanning::Configuration config);

  void on_mission_rrt_get_param_pressed();
  void on_mission_rrt_set_param_pressed();
  void on_mission_rrt_save_param_pressed();
  void on_mission_rrt_start_pressed();
  void on_mission_rrt_hold_pressed();
  void on_mission_rrt_stop_pressed();

signals:
  void requestControllerCommand(ReqControllerCMD req);
  void requestMissionCommand(Parameter req);


private:
  Ui::OffboardController *ui;
  QNode *qnode_;

  QPixmap red_led, green_led;

};

#endif // UI_CONTROLLER_H
