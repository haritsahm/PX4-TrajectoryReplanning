#include <px4_trajectory_replanning/px4_position_controller.h>

LeePositionController::LeePositionController(ros::NodeHandle &nh):
  nh_(nh)
{
  cmd_poly_sub_ = nh_.subscribe(
        "command/point", 10,
        &LeePositionController::PointCallback, this);


  position_sub_ = nh_.subscribe("/mavros/local_position/pose", 1,
                                &LeePositionController::mavPosCallback, this);

  referencePub_ = nh_.advertise<controller_msgs::FlatTarget>("reference/flatsetpoint", 1);


  cmd_req_server = nh_.advertiseService("controllers/pos_controller_cmd",
                                        &LeePositionController::commandCallback, this);
  mission_controller_cmd_server = nh_.advertiseService("controllers/mission_controller_cmd",
                                        &LeePositionController::missionCommandCallback, this);
  get_pos_state_server = nh_.advertiseService("controllers/get_pos_controller_state",
                                              &LeePositionController::getControllerState, this);

  ros::NodeHandle pnh("~");
  pnh.param("dt", dt, 0.5);

  b_spline_.reset(new ewok::UniformBSpline3D<6, double>(dt));
  time_elapsed = 0;
  controller_cmd = POSControllerCMD::POS_CMD_NONE;
  controller_state = ControllerState::CONTROLLER_STATE_LAND;
  controller_ready = false;
  request_hold = true;
  mission_hold = true;
  mission_controller_state = POSControllerCMD::POS_CMD_MISSION_HOLD;

  p_targ = v_targ = a_targ = Eigen::Vector3d::Zero();

  spin();

}

inline Eigen::Vector3d LeePositionController::toEigen(const geometry_msgs::Point& p) {
  Eigen::Vector3d ev3(p.x, p.y, p.z);
  return ev3;
}

bool LeePositionController::missionCommandCallback(px4_trajectory_replanning::POS_CONTROLLER_COMMAND::Request &req,
                                                   px4_trajectory_replanning::POS_CONTROLLER_COMMAND::Response &res)
{
  mission_controller_state =  req.cmd_req;
}

bool LeePositionController::commandCallback(px4_trajectory_replanning::POS_CONTROLLER_COMMAND::Request  &req,
                                            px4_trajectory_replanning::POS_CONTROLLER_COMMAND::Response &res)
{
  req_cmd_tol = true;
  ROS_INFO_STREAM("Incoming Command " << req.cmd_req);
  controller_cmd = req.cmd_req;

  if(req.cmd_req == POSControllerCMD::POS_CMD_HOLD)
    request_hold = true;
  else
    request_hold = false;

  return true;
}

bool LeePositionController::getControllerState(px4_trajectory_replanning::GetPOS_CONTROLLER_STATE::Request &req,
                                               px4_trajectory_replanning::GetPOS_CONTROLLER_STATE::Response &res)
{
  res.controller_state.controller_state = controller_state;
  res.controller_state.controller_ready = controller_ready;
  return true;
}

void LeePositionController::PointCallback(const geometry_msgs::PointConstPtr & point_msg)
{
  ROS_INFO("PointCallback: %d points in spline", b_spline_->size());

  Eigen::Vector3d p(point_msg->x, point_msg->y, point_msg->z);

  if(b_spline_->size() != 0) b_spline_->push_back(p);
}

void LeePositionController::mavPosCallback(const geometry_msgs::PoseStamped& msg)
{
  mavPos_ = toEigen(msg.pose.position);
  mavAtt_.w() = msg.pose.orientation.w;
  mavAtt_.x() = msg.pose.orientation.x;
  mavAtt_.y() = msg.pose.orientation.y;
  mavAtt_.z() = msg.pose.orientation.z;

  if(!request_hold)
  {
    lastPos_ = mavPos_;
    lastAtt_ = mavAtt_;
  }

}

void LeePositionController::pubrefState(){
  controller_msgs::FlatTarget msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.type_mask = 1; //
  msg.position.x = p_targ(0);
  msg.position.y = p_targ(1);
  msg.position.z = p_targ(2);
  msg.velocity.x = v_targ(0);
  msg.velocity.y = v_targ(1);
  msg.velocity.z = v_targ(2);
  msg.acceleration.x = a_targ(0);
  msg.acceleration.y = a_targ(1);
  msg.acceleration.z = a_targ(2);
  referencePub_.publish(msg);
}

void LeePositionController::followTraj()
{
  p_targ = traj->evaluate(time_elapsed, 0);
  v_targ = traj->evaluate(time_elapsed, 1);
  a_targ = traj->evaluate(time_elapsed, 2);

  Eigen::Vector3d target_pos = traj->evaluate(time_elapsed, 0);
  if(Eigen::Vector3d(target_pos - mavPos_).norm() < 0.2)
  {
    if(time_elapsed < traj->duration())
      time_elapsed += dt;
  }
}

void LeePositionController::generateTraj(Eigen::Vector3d start, Eigen::Vector3d tar, Eigen::Vector4d limits)
{
  Eigen::Vector3d mid_point = (start + tar)/2;

  traj_tar = tar;

  ewok::Polynomial3DOptimization<10> po(limits);

  typename ewok::Polynomial3DOptimization<10>::Vector3Array vec;
  vec.push_back(start);
  vec.push_back(mid_point);
  vec.push_back(tar);

  traj = po.computeTrajectory(vec);
}

void LeePositionController::getTrajectoryPoint(double t,
                                               mav_msgs::EigenTrajectoryPoint& command_trajectory, bool & yaw_from_traj) {


  command_trajectory.position_W = b_spline_->evaluate(t, 0);
  command_trajectory.velocity_W = b_spline_->evaluate(t, 1);
  command_trajectory.acceleration_W = b_spline_->evaluate(t, 2);

  static const double eps = 0.1;
  static const double delta = 0.02;

  Eigen::Vector3d d_t = b_spline_->evaluate(t + eps, 0) - command_trajectory.position_W;

  yaw_from_traj = false;

  if(std::abs(d_t[0]) > delta || std::abs(d_t[1]) > delta) {
    double yaw = std::atan2(d_t[1], d_t[0]);
    yaw_from_traj = true;

    command_trajectory.setFromYaw(yaw);


    Eigen::Vector3d d_t_e = b_spline_->evaluate(t + 2*eps, 0) - b_spline_->evaluate(t + eps, 0);

    if(std::abs(d_t_e[0]) > delta || std::abs(d_t_e[1]) > delta) {
      double yaw_e = std::atan2(d_t_e[1], d_t_e[0]);
      double yaw_rate = (yaw_e - yaw) / eps;
      command_trajectory.setFromYawRate(yaw_rate);
    } else {
      command_trajectory.setFromYawRate(0);
    }

  }

}

void LeePositionController::followBSpline()
{
  if(b_spline_->size() == 0) {
    for(int i=0; i<6; i++) b_spline_->push_back(mavPos_);
    init_time = ros::Time::now();
    last_yaw = mav_msgs::yawFromQuaternion(mavAtt_);
  }

  if(time_elapsed > b_spline_->maxValidTime() || mission_hold) {
    b_spline_->push_back(b_spline_->getControlPoint(b_spline_->size()-1));
    ROS_WARN("Adding last point once again!");
  }

  bool yaw_from_traj;
  mav_msgs::EigenTrajectoryPoint command_trajectory;
  getTrajectoryPoint(time_elapsed, command_trajectory, yaw_from_traj);

  if(!yaw_from_traj) {
    command_trajectory.setFromYaw(last_yaw);
    command_trajectory.setFromYawRate(0);
  } else {
    last_yaw = command_trajectory.getYaw();
  }

  Eigen::Vector3d target_pos = b_spline_->evaluate(time_elapsed, 0);
  if(Eigen::Vector3d(target_pos - mavPos_).norm() < 0.5)
  {
    if(time_elapsed < b_spline_->maxValidTime())
      time_elapsed += dt;
  }

}

void LeePositionController::spin()
{
  ros::Rate rate(100);
  controller_ready = true;

  while(ros::ok())
  {
    switch (controller_cmd) {
    case POSControllerCMD::POS_CMD_LAND:
    {
      generateTraj(mavPos_, Eigen::Vector3d(mavPos_.x(),mavPos_.y(), -0.1), Eigen::Vector4d(0.2, 0.2, 0, 0));
      time_elapsed = 0;
      traj_type = TRAJ_LAND;
      controller_cmd = POSControllerCMD::POS_CMD_FOL_TRAJ;
      break;
    }

    case POSControllerCMD::POS_CMD_TAKEOFF:
    {
      generateTraj(mavPos_, Eigen::Vector3d(mavPos_.x(),mavPos_.y(),2), Eigen::Vector4d(0.2, 0.2, 0, 0));
      time_elapsed = 0;
      traj_type = TRAJ_TAKEOFF;
      controller_cmd = POSControllerCMD::POS_CMD_FOL_TRAJ;
      break;
    }

    case POSControllerCMD::POS_CMD_FOL_TRAJ:
    {
      if(traj_type == TRAJ_MISSION)
      {
        switch (mission_controller_state) {
        case POSControllerCMD::POS_CMD_MISSION_HOLD:
        {
          mission_hold = true;
          break;
        }
        case POSControllerCMD::POS_CMD_MISSION_START:
        {
          mission_hold = true;
          break;
        }
        case POSControllerCMD::POS_CMD_MISSION_STOP:
        {
          request_hold = true;
          controller_cmd = POSControllerCMD::POS_CMD_HOLD;
          break;
        }
        }

        followBSpline();
      }

      else {
        followTraj();
        double dist = Eigen::Vector3d(traj_tar-mavPos_).norm();
        if(dist < 0.2)
        {
          request_hold = true;
          controller_cmd = POSControllerCMD::POS_CMD_HOLD;
        }
      }
      break;
    }

    case POSControllerCMD::POS_CMD_HOLD:
    {
      p_targ = lastPos_;
      v_targ = Eigen::Vector3d::Zero();
      a_targ = Eigen::Vector3d::Zero();
      controller_state = CONTROLLER_STATE_HOLD;
      break;
    }

    case POSControllerCMD::POS_CMD_MISSION_FOLLOW:
    {
      time_elapsed = 0;
      traj_type = TRAJ_MISSION;
      controller_cmd = POSControllerCMD::POS_CMD_FOL_TRAJ;
      controller_state = CONTROLLER_STATE_MISSION_FOLLOW;
      break;
    }

    case POSControllerCMD::POS_CMD_NONE:
    {
      break;
    }

    }

    pubrefState(); //publish to geometric controller


    ros::spinOnce();
    rate.sleep();
  }

  ros::shutdown();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "px4_position_controller_node");
  ros::NodeHandle nh;
  LeePositionController controller(nh);

  return 0;
}
