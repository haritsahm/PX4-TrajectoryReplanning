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
  get_pos_state_server = nh_.advertiseService("controllers/get_pos_controller_state",
                                              &LeePositionController::getControllerState, this);

  ros::NodeHandle pnh("~");
  pnh.param("dt", dt, 0.5);

  b_spline_.reset(new ewok::UniformBSpline3D<6, double>(dt));
  time_elapsed = 0;
  controller_cmd = TOL_CMD_LAND;
  controller_state = CONTROLLER_STATE_LAND;
  controller_ready = false;

  spin();

}

inline Eigen::Vector3d LeePositionController::toEigen(const geometry_msgs::Point& p) {
  Eigen::Vector3d ev3(p.x, p.y, p.z);
  return ev3;
}

bool LeePositionController::commandCallback(px4_trajectory_replanning::POS_CONTROLLER_COMMAND::Request  &req,
                                            px4_trajectory_replanning::POS_CONTROLLER_COMMAND::Response &res)
{
  ROS_INFO("POSITION CONTROLLER : Incoming Command");

  req_cmd_tol = true;

  if(req.cmd_req == req.CMD_HOLD_ALT)
    request_hold = true;
  else
    request_hold = false;


  if(req.cmd_req == req.CMD_LANDING)
    controller_cmd = TOL_CMD_LAND;
  else if(req.cmd_req == req.CMD_TAKEOFF)
    controller_cmd = TOL_CMD_TAKEOFF;
  else if(req.cmd_req == req.CMD_HOLD_ALT)
    controller_cmd = TOL_CMD_HOLD;
  else if(req.cmd_req == req.CMD_MISSION_FOLLOW)
    controller_cmd = TOL_CMD_MISSION_FOLLOW;

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

void LeePositionController::generateTraj(Eigen::Vector3d init, Eigen::Vector3d tar, Eigen::Vector4d limits)
{
  Eigen::Vector3d start_point(-5, -5, 1), middle_point(2,0,1), end_point(5, 5, 1);

  ewok::Polynomial3DOptimization<10> po(limits);

  typename ewok::Polynomial3DOptimization<10>::Vector3Array vec;
  vec.push_back(start_point);
  vec.push_back(middle_point);
  vec.push_back(end_point);

  auto traj = po.computeTrajectory(vec);

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

  if(time_elapsed > b_spline_->maxValidTime()) {
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

  ROS_INFO("POSITION CONTROLLRE RUNNIGN");

  while(ros::ok())
  {
    switch (controller_cmd) {
    case TOL_CMD_LAND:
    {
      p_targ = Eigen::Vector3d::Zero();
      v_targ = Eigen::Vector3d(0, 0, 0.01);
      a_targ = Eigen::Vector3d(0, 0, 0.01);
      controller_state = CONTROLLER_STATE_LAND;
      break;
    }

    case TOL_CMD_TAKEOFF:
    {
      p_targ = Eigen::Vector3d(0,0,2);
      v_targ = Eigen::Vector3d(0,0, 0.01);
      a_targ = Eigen::Vector3d(0, 0, 0.01);
      controller_state = CONTROLLER_STATE_TAKEOFF;
      break;
    }

    case TOL_CMD_HOLD:
    {
      p_targ = lastPos_;
      v_targ = Eigen::Vector3d::Zero();
      a_targ = Eigen::Vector3d::Zero();
      controller_state = CONTROLLER_STATE_HOLD;
      break;
    }

    case TOL_CMD_MISSION_FOLLOW:
    {
      followBSpline();
      controller_state = CONTROLLER_STATE_MISSION_FOLLOW;
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
