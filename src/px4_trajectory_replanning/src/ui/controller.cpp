#include <px4_trajectory_replanning/ui/controller.h>

OffboardController::OffboardController(int argc, char** argv, QWidget *parent) :
  QWidget(parent),
  ui(new Ui::OffboardController)
{
  ui->setupUi(this);
  qnode_ = new QNode(argc,argv);

  std::string green_led_path = ros::package::getPath("px4_trajectory_replanning")+"/resources/green_led.png";
  std::string red_led_path = ros::package::getPath("px4_trajectory_replanning")+"/resources/red_led.png";

  if(green_led.load(QString::fromStdString(green_led_path)))
    green_led = green_led.scaled(ui->led_hold->size(),Qt::KeepAspectRatio);

  if(red_led.load(QString::fromStdString(red_led_path)))
    red_led = red_led.scaled(ui->led_hold->size(),Qt::KeepAspectRatio);

  ui->led_hold->setPixmap(red_led);
  ui->led_landing->setPixmap(red_led);
  ui->led_takeoff->setPixmap(red_led);
  ui->led_offboard->setPixmap(red_led);
  ui->led_vehicle_arm->setPixmap(red_led);
  ui->button_arm_vehicle->setText("Armed");

  qRegisterMetaType<MavState>("MavState");
  connect(qnode_, SIGNAL(updateUI(MavState)), this, SLOT(setMavState(MavState)));
  connect(this, &OffboardController::requestControllerCommand, qnode_, &QNode::sendingControllerCommand);

  qRegisterMetaType<Parameter>("Parameter");
  connect(this, SIGNAL(requestMissionParam(Parameter)), qnode_, SLOT(sendingMissionCommand(Parameter)));

  qRegisterMetaType<px4_trajectory_replanning::Configuration>("px4_trajectory_replanning::Configuration");
  connect(qnode_, SIGNAL(updateMissionParam(px4_trajectory_replanning::Configuration)), this, SLOT(setMissionParam(px4_trajectory_replanning::Configuration)));
  QObject::connect(qnode_, SIGNAL(rosShutdown()), this, SLOT(close()));

  qnode_->init();

}

OffboardController::~OffboardController()
{
  delete ui;
}

void OffboardController::setMavState(MavState state)
{
      for (MavState::iterator itr = state.begin(); itr != state.end(); ++itr) {
        if(itr->first == QTOLState::QSTATE_OFFBOARD)
        {
          if (itr->second == 0)
          {
            ui->led_offboard->setPixmap(red_led);
            ui->label_mode_status->setText(QString("Unknown"));
          }
          else
          {
            ui->led_offboard->setPixmap(green_led);
            ui->label_mode_status->setText(QString("Offboard"));
          }

        }
        else if(itr->first == QTOLState::QSTATE_ROTOR)
        {
          if (itr->second == 0)
          {
            ui->button_arm_vehicle->setText("Arm");
            ui->label_motor_status->setText("Disarm");
            ui->led_vehicle_arm->setPixmap(red_led);
          }
          else
          {
            ui->button_arm_vehicle->setText("Disarm");
            ui->label_motor_status->setText("Arm");
            ui->led_vehicle_arm->setPixmap(green_led);
          }
        }

        else if(itr->first == QTOLState::QSTATE_TOL)
        {
          if(itr->second == TOL_STATE_LAND)
          {
            ui->led_landing->setPixmap(green_led);
            ui->led_takeoff->setPixmap(red_led);
            ui->label_command_status->setText("Landing");
          }
          else if(itr->second == TOL_STATE_TAKEOFF)
          {
            ui->led_landing->setPixmap(red_led);
            ui->led_takeoff->setPixmap(green_led);
            ui->label_command_status->setText("Takeoff");
          }
        }

      }
}

void OffboardController::setMissionParam(px4_trajectory_replanning::Configuration config)
{
  ui->dspin_param_spline_maxvel->setValue(config.max_velocity);
  ui->dspin_param_spline_accel->setValue(config.max_acceleration);
  ui->dspin_param_spline_numpts->setValue(config.num_opt_points);
  ui->dspin_param_buff_res->setValue(config.resolution);

  ui->dspin_param_rrt_stepsize->setValue(config.step_size);
  ui->dspin_param_rrt_factor->setValue(config.rrt_factor);
  ui->dspin_param_rrt_solvt->setValue(config.max_solve_t);
  ui->dspin_param_rrt_maxiter->setValue(config.num_iter);
  ui->dspin_param_rrt_uavrad->setValue(config.uav_radius);

  ui->dspin_param_path_ptwindow->setValue(config.num_pt_window);
  ui->dspin_param_path_cleardist->setValue(config.clear_distance);
  ui->dspin_param_path_checkdist->setValue(config.max_check_dist);

}

/*
 * Command to Offboard Controller
 */


void OffboardController::on_mode_hold_pressed()
{
  emit requestControllerCommand(ReqControllerCMD::REQ_CONTROLLER_HOLD);
}

void OffboardController::on_mode_landing_pressed()
{
  emit requestControllerCommand(ReqControllerCMD::REQ_CONTROLLER_LAND);
}

void OffboardController::on_mode_takeoff_pressed()
{
  emit requestControllerCommand(ReqControllerCMD::REQ_CONTROLLER_TAKEOFF);
}

void OffboardController::on_mode_offboard_pressed()
{
  emit requestControllerCommand(ReqControllerCMD::REQ_CONTROLLER_OFFBOARD);
}

void OffboardController::on_button_arm_vehicle_pressed()
{
  if(ui->button_arm_vehicle->text() == QString("Arm"))
    emit requestControllerCommand(ReqControllerCMD::REQ_CONTROLLER_ROTOR_ARMED);
  if(ui->button_arm_vehicle->text() == QString("Disarm"))
    emit requestControllerCommand(ReqControllerCMD::REQ_CONTROLLER_ROTOR_DISARMED);
}

void OffboardController::on_mode_mission_pressed()
{
  Parameter param;
  param.mission_req = REQ_MISSION_GET_PARAM;

  emit requestMissionCommand(param);
  emit requestControllerCommand(ReqControllerCMD::REQ_CONTROLLER_MISSION);
}

/*
 * Command to Mission Controller
 */

void OffboardController::on_mission_rrt_hold_pressed()
{
  Parameter param;
  param.mission_req = REQ_MISSION_HOLD;

  emit requestMissionCommand(param);
}

void OffboardController::on_mission_rrt_stop_pressed()
{
  Parameter param;
  param.mission_req = REQ_MISSION_STOP;

  emit requestMissionCommand(param);
}

void OffboardController::on_mission_rrt_start_pressed()
{
  Parameter param;
  param.mission_req = REQ_MISSION_START;

  emit requestMissionCommand(param);
}

void OffboardController::on_mission_rrt_save_param_pressed()
{
  Parameter param;
  param.mission_req = REQ_MISSION_SAVE_PARAM;

  emit requestMissionCommand(param);
}

void OffboardController::on_mission_rrt_get_param_pressed()
{
  Parameter param;
  param.mission_req = REQ_MISSION_GET_PARAM;

  emit requestMissionCommand(param);
}

void OffboardController::on_mission_rrt_set_param_pressed()
{
  Parameter param;
  param.config.max_velocity = ui->dspin_param_spline_maxvel->value();
  param.config.max_acceleration = ui->dspin_param_spline_accel->value();
  param.config.num_opt_points = ui->dspin_param_spline_numpts->value();
  param.config.resolution =  ui->dspin_param_buff_res->value();

  param.config.step_size = ui->dspin_param_rrt_stepsize->value();
  param.config.rrt_factor = ui->dspin_param_rrt_factor->value();
  param.config.max_solve_t = ui->dspin_param_rrt_solvt->value();
  param.config.num_iter = ui->dspin_param_rrt_maxiter->value();
  param.config.uav_radius = ui->dspin_param_rrt_uavrad->value();

  param.config.num_pt_window = ui->dspin_param_path_ptwindow->value();
  param.config.clear_distance = ui->dspin_param_path_cleardist->value();
  param.config.max_check_dist = ui->dspin_param_path_checkdist->value();

  param.mission_req = REQ_MISSION_SET_PARAM;

  emit requestMissionCommand(param);
}
