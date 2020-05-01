#include <px4_trajectory_replanning/px4_position_controller.h>

MotionController::MotionController(ros::NodeHandle &nh):
  nh_(nh)
{
  cmd_poly_sub_ = nh_.subscribe(
        "trajectory/command/point", 10,
        &MotionController::PointCallback, this);


  position_sub_ = nh_.subscribe("/mavros/local_position/pose", 10,
                                &MotionController::mavPosCallback, this);
  odometry_sub_ = nh_.subscribe("/mavros/local_position/odom", 10,
                               &MotionController::OdometryCallback, this);

  referencePub_ = nh_.advertise<controller_msgs::FlatTarget>("reference/flatsetpoint", 1);
  yawPub_ = nh_.advertise<std_msgs::Float32>("reference/yaw", 1);
  pathPub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/command/trajectory", 1);

  cmd_req_server = nh_.advertiseService("controllers/pos_controller_cmd",
                                        &MotionController::commandCallback, this);
  mission_controller_cmd_server = nh_.advertiseService("controllers/mission_controller_cmd",
                                        &MotionController::missionCommandCallback, this);
  get_pos_state_server = nh_.advertiseService("controllers/get_pos_controller_state",
                                              &MotionController::getControllerState, this);

  std::string path = ros::package::getPath("px4_trajectory_replanning") + "/config/config.yaml";
  loadParam(path);

  ros::NodeHandle pnh("~");
  pnh.param("dt", dt, global_cofing["dt"].as<double>());

  GetVehicleParameters(pnh, &lee_position_controller_.vehicle_parameters_);
  lee_position_controller_.InitializeParameters();

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

void MotionController::loadParam(const std::string path)
{
  YAML::Node node;

  try
  {
    // load yaml
    node = YAML::LoadFile(path.c_str());
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
  }

  controller_config = node["controllers"];
  lee_position_controller_.controller_parameters_.position_gain_ = controller_config["position_gain"].as<Eigen::Vector3d>();
  lee_position_controller_.controller_parameters_.velocity_gain_ = controller_config["velocity_gain"].as<Eigen::Vector3d>();
  lee_position_controller_.controller_parameters_.attitude_gain_ = controller_config["attitude_gain"].as<Eigen::Vector3d>();
  lee_position_controller_.controller_parameters_.angular_rate_gain_ = controller_config["angular_rate_gain"].as<Eigen::Vector3d>();

  global_cofing = node["global"];
}

inline Eigen::Vector3d MotionController::toEigen(const geometry_msgs::Point& p) {
  Eigen::Vector3d ev3(p.x, p.y, p.z);
  return ev3;
}

bool MotionController::missionCommandCallback(px4_trajectory_replanning::POS_CONTROLLER_COMMAND::Request &req,
                                                   px4_trajectory_replanning::POS_CONTROLLER_COMMAND::Response &res)
{
  mission_controller_state =  req.cmd_req;
}

bool MotionController::commandCallback(px4_trajectory_replanning::POS_CONTROLLER_COMMAND::Request  &req,
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

bool MotionController::getControllerState(px4_trajectory_replanning::GetPOS_CONTROLLER_STATE::Request &req,
                                               px4_trajectory_replanning::GetPOS_CONTROLLER_STATE::Response &res)
{
  res.controller_state.controller_state = controller_state;
  res.controller_state.controller_ready = controller_ready;
  return true;
}

void MotionController::PointCallback(const geometry_msgs::PointConstPtr & point_msg)
{
  ROS_INFO("PointCallback: %d points in spline", b_spline_->size());

  Eigen::Vector3d p(point_msg->x, point_msg->y, point_msg->z);

  if(b_spline_->size() != 0) b_spline_->push_back(p);
}

void MotionController::mavPosCallback(const geometry_msgs::PoseStamped& msg)
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

void MotionController::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{
  rotors_control::eigenOdometryFromMsg(odometry_msg, &odometry);
}

void MotionController::pubrefState(){
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
  msg.jerk.x = 0;
  msg.jerk.y = 0;
  msg.jerk.z = 0;
  referencePub_.publish(msg);
}

void MotionController::pubBodyRate()
{
  
}

void MotionController::calculateBodyRate()
{

}

void MotionController::followTraj()
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

void MotionController::generateTraj(Eigen::Vector3d start, Eigen::Vector3d tar, Eigen::Vector4d limits)
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

void MotionController::getTrajectoryPoint(double t,
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

void MotionController::followBSpline()
{
  if(b_spline_->size() == 0) {
    for(int i=0; i<7; i++) b_spline_->push_back(mavPos_);
    init_time = ros::Time::now();
    traj_pt_counter = 0;
    last_yaw = mav_msgs::yawFromQuaternion(mavAtt_);
  }

  if(time_elapsed > b_spline_->maxValidTime() || mission_hold) {
    if(traj_pt_counter < 8)
      {     
        b_spline_->push_back(b_spline_->getControlPoint(b_spline_->size()-1));
        ROS_WARN("Adding last point once again!");
      }
    traj_pt_counter++;
  }

  bool yaw_from_traj;
  mav_msgs::EigenTrajectoryPoint command_trajectory;
  // getTrajectoryPoint(time_elapsed, command_trajectory, yaw_from_traj);
  // lee_position_controller_.SetTrajectoryPoint(command_trajectory);
  // lee_position_controller_.SetOdometry(odometry);

  p_targ = b_spline_->evaluate(time_elapsed, 0);
  v_targ = b_spline_->evaluate(time_elapsed, 1);
  a_targ = b_spline_->evaluate(time_elapsed, 2);

  static const double eps = 0.1;
  static const double delta = 0.02;

  Eigen::Vector3d d_t = b_spline_->evaluate(time_elapsed + eps, 0) - command_trajectory.position_W;

  yaw_from_traj = false;

  if(std::abs(d_t[0]) > delta || std::abs(d_t[1]) > delta) {
    double yaw = std::atan2(d_t[1], d_t[0]);
    yaw_from_traj = true;

    targ_yaw = yaw;
    // command_trajectory.setFromYaw(yaw);

  //   Eigen::Vector3d d_t_e = b_spline_->evaluate(time_elapsed + 2*eps, 0) - b_spline_->evaluate(time_elapsed + eps, 0);

  //   if(std::abs(d_t_e[0]) > delta || std::abs(d_t_e[1]) > delta) {
  //     double yaw_e = std::atan2(d_t_e[1], d_t_e[0]);
  //     double yaw_rate = (yaw_e - yaw) / eps;
  //     command_trajectory.setFromYawRate(yaw_rate);
  //   } else {
  //     command_trajectory.setFromYawRate(0);
  //   }

  }

  if(!yaw_from_traj) {
    targ_yaw = last_yaw;
    // command_trajectory.setFromYaw(last_yaw);
    // command_trajectory.setFromYawRate(0);
  } else {
    last_yaw = mav_msgs::yawFromQuaternion(mavAtt_);
  }

  Eigen::Vector3d target_pos = b_spline_->evaluate(time_elapsed, 0);
  if(Eigen::Vector3d(target_pos - mavPos_).norm() < 0.2)
  {
    if(time_elapsed < b_spline_->maxValidTime())
      {
        time_elapsed += dt;
        traj_pt_counter=0;
        }
  }

  std_msgs::Float32 msg;
  msg.data = targ_yaw;
  yawPub_.publish(msg);

}

void MotionController::spin()
{
  ros::Rate rate(1/0.01);
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
          mission_hold = false;
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
  MotionController controller(nh);

  return 0;
}
