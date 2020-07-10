#include <px4_trajectory_replanning/px4_position_controller.h>

MotionController::MotionController(ros::NodeHandle &nh) : nh_(nh)
{
  cmd_poly_sub_ = nh_.subscribe("trajectory/command/point", 10, &MotionController::PointCallback, this);

  position_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &MotionController::mavPosCallback, this);
  mavtwistSub_ = nh_.subscribe("/mavros/local_position/velocity_local", 1, &MotionController::mavtwistCallback, this,
                               ros::TransportHints().tcpNoDelay());

  referencePub_ = nh_.advertise<controller_msgs::FlatTarget>("reference/flatsetpoint", 1);
  yawPub_ = nh_.advertise<std_msgs::Float32>("reference/yaw", 1);
  pathPub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/command/trajectory", 1);
  bodyRatePub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
  cmd_req_server = nh_.advertiseService("controllers/pos_controller_cmd", &MotionController::commandCallback, this);
  mission_command_server =
      nh_.advertiseService("controllers/mission_command_pos_param", &MotionController::missionCommandCallback, this);

  get_pos_state_server =
      nh_.advertiseService("controllers/get_pos_controller_state", &MotionController::getControllerState, this);

  std::string path = ros::package::getPath("px4_trajectory_replanning") + "/config/config.yaml";
  loadParam(path);

  ros::NodeHandle pnh("~");
  pnh.param("dt", dt, global_cofing["dt"].as<double>());

  GetVehicleParameters(pnh, &lee_position_controller_.vehicle_parameters_);
  lee_position_controller_.InitializeParameters();

  pnh.param<double>("max_acc", max_fb_acc_, 9.0);
  pnh.param<double>("drag_dx", dx_, 0.0);
  pnh.param<double>("drag_dy", dy_, 0.0);
  pnh.param<double>("drag_dz", dz_, 0.0);
  pnh.param<double>("attctrl_constant", attctrl_tau_, 0.1);
  pnh.param<double>("normalizedthrust_constant", norm_thrust_const_, 0.05);  // 1 / max acceleration
  pnh.param<double>("normalizedthrust_offset", norm_thrust_offset_, 0.1);    // 1 / max acceleration
  pnh.param<double>("Kp_x", Kpos_x_, 8.0);
  pnh.param<double>("Kp_y", Kpos_y_, 8.0);
  pnh.param<double>("Kp_z", Kpos_z_, 10.0);
  pnh.param<double>("Kv_x", Kvel_x_, 1.5);
  pnh.param<double>("Kv_y", Kvel_y_, 1.5);
  pnh.param<double>("Kv_z", Kvel_z_, 3.3);

  g_ << 0.0, 0.0, -9.8;
  Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;

  D_ << dx_, dy_, dz_;

  tau << tau_x, tau_y, tau_z;

  b_spline_.reset(new ewok::UniformBSpline3D<6, double>(dt));
  time_elapsed = 0;
  controller_cmd = POSControllerCMD::POS_CMD_NONE;
  controller_state = ControllerState::CONTROLLER_STATE_LAND;
  controller_ready = false;
  request_hold = true;
  mission_hold = true;
  flag_reset = false;
  bspline_allowed = false;
  mission_controller_state = POSControllerCMD::POS_CMD_MISSION_HOLD;

  p_targ = v_targ = a_targ = Eigen::Vector3d::Zero();
  p_targ = Eigen::Vector3d(0, 0, 0.2);
  v_targ = Eigen::Vector3d(0, 0, 0.001);
  targ_yaw = mav_msgs::yawFromQuaternion(mavAtt_);

  hold_stamp = ros::Time::now();
  pubRateCommands(Eigen::Vector4d(0,0,0,0.1));
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
  catch (const std::exception &e)
  {
    ROS_ERROR("Fail to load yaml file.");
  }

  controller_config = node["controllers"];
  lee_position_controller_.controller_parameters_.position_gain_ =
      controller_config["position_gain"].as<Eigen::Vector3d>();
  lee_position_controller_.controller_parameters_.velocity_gain_ =
      controller_config["velocity_gain"].as<Eigen::Vector3d>();
  lee_position_controller_.controller_parameters_.attitude_gain_ =
      controller_config["attitude_gain"].as<Eigen::Vector3d>();
  lee_position_controller_.controller_parameters_.angular_rate_gain_ =
      controller_config["angular_rate_gain"].as<Eigen::Vector3d>();

  global_cofing = node["global"];
}

inline Eigen::Vector3d MotionController::toEigen(const geometry_msgs::Point &p)
{
  Eigen::Vector3d ev3(p.x, p.y, p.z);
  return ev3;
}

bool MotionController::missionCommandCallback(px4_trajectory_replanning::MAV_MISSION_COMMAND::Request &req,
                                              px4_trajectory_replanning::MAV_MISSION_COMMAND::Response &res)
{
  if (req.header.frame_id == "UI Interface")
  {
    if (traj_type == TRAJ_MISSION)
    {
      mission_controller_state = req.mission_command;
      ROS_WARN_STREAM("Incoming Command : " << mission_controller_state);
    }
  }
}

bool MotionController::commandCallback(px4_trajectory_replanning::POS_CONTROLLER_COMMAND::Request &req,
                                       px4_trajectory_replanning::POS_CONTROLLER_COMMAND::Response &res)
{
  req_cmd_tol = true;
  controller_cmd = req.cmd_req;

  if (req.cmd_req == POSControllerCMD::POS_CMD_HOLD)
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

/*
 * Receiving Spline Setpoint to Reconstruct Bspline
 */
void MotionController::PointCallback(const geometry_msgs::PointConstPtr &point_msg)
{
  ROS_INFO("PointCallback: %d points in spline", b_spline_->size());

  Eigen::Vector3d p(point_msg->x, point_msg->y, point_msg->z);
  if (b_spline_->size() != 0)
  {
    ROS_WARN_STREAM("Received Point : " << p.transpose());
    b_spline_->push_back(p);
  }
}

/*
 * Receiving UAV Position Estimation from Pixhawk via MavRos
 */

void MotionController::mavPosCallback(const geometry_msgs::PoseStamped &msg)
{
  mavPos_ = toEigen(msg.pose.position);
  mavAtt_.w() = msg.pose.orientation.w;
  mavAtt_.x() = msg.pose.orientation.x;
  mavAtt_.y() = msg.pose.orientation.y;
  mavAtt_.z() = msg.pose.orientation.z;

  if (!request_hold)
  {
    lastPos_ = mavPos_;
    lastAtt_ = mavAtt_;
  }
}

/*
 * Receiving UAV Twist (Velocity) Estimation from Pixhawk via MavRos
 */

void MotionController::mavtwistCallback(const geometry_msgs::TwistStamped &msg)
{
  geometry_msgs::Vector3 v3 = msg.twist.linear;
  mavVel_ = Eigen::Vector3d(v3.x, v3.y, v3.z);
}

/*
 * Publish Calculated Rate to Pixhawk via MavRos
 */

void MotionController::pubRateCommands(const Eigen::Vector4d &cmd)
{
  mavros_msgs::AttitudeTarget msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.body_rate.x = cmd(0);
  msg.body_rate.y = cmd(1);
  msg.body_rate.z = cmd(2);
  msg.type_mask = 128;  // Ignore orientation messages
  msg.thrust = cmd(3);

  bodyRatePub_.publish(msg);
}

/*
 * Calculate Body Rate based on Differental Flatness
 */

void MotionController::computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd)
{
  const Eigen::Vector3d a_ref = a_targ;
  const Eigen::Vector4d q_ref = acc2quaternion(a_ref - g_, targ_yaw);
  const Eigen::Matrix3d R_ref = quat2RotMatrix(q_ref);

  const Eigen::Vector3d pos_error = mavPos_ - p_targ;
  const Eigen::Vector3d vel_error = mavVel_ - v_targ;

  Eigen::Vector3d a_fb =
      Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error;  // feedforward term for trajectory error
  if (a_fb.norm() > max_fb_acc_)
    a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;  // Clip acceleration if reference is too large

  const Eigen::Vector3d a_rd = R_ref * D_.asDiagonal() * R_ref.transpose() * v_targ;  // Rotor drag
  const Eigen::Vector3d a_des = a_fb + a_ref - a_rd - g_;

  q_des = acc2quaternion(a_des, targ_yaw);

  Eigen::Vector4d mavAtt_4d = Eigen::Vector4d(mavAtt_.w(), mavAtt_.x(), mavAtt_.y(), mavAtt_.z());
  bodyrate_cmd = geometric_attcontroller(q_des, a_des, mavAtt_4d);  // Calculate BodyRate

}

/*
 * Calculate Body Rate based
 */

Eigen::Vector4d MotionController::geometric_attcontroller(const Eigen::Vector4d &ref_att,
                                                          const Eigen::Vector3d &ref_acc, Eigen::Vector4d &curr_att)
{
  Eigen::Vector4d ratecmd;
  Eigen::Vector4d qe, q_inv, inverse;
  Eigen::Matrix3d rotmat;
  Eigen::Vector3d zb;

  inverse << 1.0, -1.0, -1.0, -1.0;
  q_inv = inverse.asDiagonal() * curr_att;
  qe = quatMultiplication(q_inv, ref_att);
  ratecmd(0) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(1);
  ratecmd(1) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(2);
  ratecmd(2) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(3);
  Eigen::Vector4d mavAtt_4d = Eigen::Vector4d(mavAtt_.w(), mavAtt_.x(), mavAtt_.y(), mavAtt_.z());
  rotmat = quat2RotMatrix(mavAtt_4d);
  zb = rotmat.col(2);
  ratecmd(3) =
      std::max(0.0, std::min(1.0, norm_thrust_const_ * ref_acc.dot(zb) + norm_thrust_offset_));  // Calculate thrust
  return ratecmd;
}

Eigen::Vector4d MotionController::acc2quaternion(const Eigen::Vector3d vector_acc, double yaw)
{
  Eigen::Vector4d quat;
  Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
  Eigen::Matrix3d rotmat;

  proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;

  zb_des = vector_acc / vector_acc.norm();
  yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
  xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

  rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
  quat = rot2Quaternion(rotmat);
  return quat;
}

Eigen::Vector4d MotionController::quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p)
{
  Eigen::Vector4d quat;
  quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3), p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
      p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1), p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
  return quat;
}

Eigen::Matrix3d MotionController::quat2RotMatrix(const Eigen::Vector4d q)
{
  Eigen::Matrix3d rotmat;
  rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3), 2 * q(1) * q(2) - 2 * q(0) * q(3),
      2 * q(0) * q(2) + 2 * q(1) * q(3),

      2 * q(0) * q(3) + 2 * q(1) * q(2), q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
      2 * q(2) * q(3) - 2 * q(0) * q(1),

      2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3),
      q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  return rotmat;
}

Eigen::Vector4d MotionController::rot2Quaternion(const Eigen::Matrix3d R)
{
  Eigen::Vector4d quat;
  double tr = R.trace();
  if (tr > 0.0)
  {
    double S = sqrt(tr + 1.0) * 2.0;  // S=4*qw
    quat(0) = 0.25 * S;
    quat(1) = (R(2, 1) - R(1, 2)) / S;
    quat(2) = (R(0, 2) - R(2, 0)) / S;
    quat(3) = (R(1, 0) - R(0, 1)) / S;
  }
  else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2)))
  {
    double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;  // S=4*qx
    quat(0) = (R(2, 1) - R(1, 2)) / S;
    quat(1) = 0.25 * S;
    quat(2) = (R(0, 1) + R(1, 0)) / S;
    quat(3) = (R(0, 2) + R(2, 0)) / S;
  }
  else if (R(1, 1) > R(2, 2))
  {
    double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;  // S=4*qy
    quat(0) = (R(0, 2) - R(2, 0)) / S;
    quat(1) = (R(0, 1) + R(1, 0)) / S;
    quat(2) = 0.25 * S;
    quat(3) = (R(1, 2) + R(2, 1)) / S;
  }
  else
  {
    double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;  // S=4*qz
    quat(0) = (R(1, 0) - R(0, 1)) / S;
    quat(1) = (R(0, 2) + R(2, 0)) / S;
    quat(2) = (R(1, 2) + R(2, 1)) / S;
    quat(3) = 0.25 * S;
  }
  return quat;
}

Eigen::Vector3d MotionController::matrix_hat_inv(const Eigen::Matrix3d &m)
{
  Eigen::Vector3d v;
  // TODO: Sanity checks if m is skew symmetric
  v << m(7), m(2), m(3);
  return v;
}

void MotionController::followTraj()
{
  p_targ = traj->evaluate(time_elapsed, 0);
  v_targ = traj->evaluate(time_elapsed, 1);
  a_targ = traj->evaluate(time_elapsed, 2);
  // targ_yaw = mav_msgs::yawFromQuaternion(mavAtt_);
  yaw_from_traj = false;

  Eigen::Vector3d target_pos = traj->evaluate(time_elapsed, 0);
  if (Eigen::Vector3d(target_pos - mavPos_).norm() < 0.2)
  {
    if (time_elapsed < traj->duration())
      time_elapsed += dt;
  }
}

void MotionController::generateTraj(Eigen::Vector3d start, Eigen::Vector3d tar, Eigen::Vector4d limits)
{
  Eigen::Vector3d mid_point = (start + tar) / 2;

  traj_tar = tar;

  ewok::Polynomial3DOptimization<10> po(limits);

  typename ewok::Polynomial3DOptimization<10>::Vector3Array vec;
  vec.push_back(start);
  vec.push_back(mid_point);
  vec.push_back(tar);

  traj = po.computeTrajectory(vec);
}

void MotionController::followBSpline()
{
  if (b_spline_->size() == 0 || flag_reset)
  {
    ROS_FATAL("RESET BSPLINE");
    b_spline_.reset(new ewok::UniformBSpline3D<6, double>(dt));
    for (int i = 0; i < 7; i++)
      b_spline_->push_back(mavPos_);
    init_time = ros::Time::now();
    traj_pt_counter = 0;
    last_yaw = mav_msgs::yawFromQuaternion(mavAtt_);
    flag_reset = false;
    time_elapsed = 0;
  }

  double local_t = (ros::Time::now() - init_time).toSec();
  // std::cout << "Time : " << local_t << std::endl;
  if (mission_hold || time_elapsed > b_spline_->maxValidTime())
  {
    if ((ros::Time::now() - hold_stamp).toSec() >= 0.5)
    {
      hold_stamp = ros::Time::now();
      b_spline_->push_back(b_spline_->getControlPoint(b_spline_->size() - 1));
      ROS_WARN("Adding last point once again!");
    }
  }

  p_targ = b_spline_->evaluate(time_elapsed, 0);
  v_targ = b_spline_->evaluate(time_elapsed, 1);
  a_targ = b_spline_->evaluate(time_elapsed, 2);

  static const double eps = 0.1;
  static const double delta = 0.02;

  Eigen::Vector3d d_t = b_spline_->evaluate(time_elapsed + eps, 0) - p_targ;

  yaw_from_traj = false;

  if (std::abs(d_t[0]) > delta || std::abs(d_t[1]) > delta)
  {
    double yaw = std::atan2(d_t[1], d_t[0]);
    yaw_from_traj = true;

    targ_yaw = yaw;

    Eigen::Vector3d d_t_e = b_spline_->evaluate(time_elapsed + 2 * eps, 0) - b_spline_->evaluate(time_elapsed + eps, 0);

    if (std::abs(d_t_e[0]) > delta || std::abs(d_t_e[1]) > delta)
    {
      double yaw_e = std::atan2(d_t_e[1], d_t_e[0]);
      double yaw_rate = (yaw_e - yaw) / eps;
      zrate_targ = yaw_rate;
    }
    // else {
    //   command_trajectory.setFromYawRate(0);
    // }
  }

  if (!yaw_from_traj)
  {
    targ_yaw = last_yaw;
    // command_trajectory.setFromYaw(last_yaw);
    // command_trajectory.setFromYawRate(0);
  }
  else
  {
    last_yaw = targ_yaw;
  }

  Eigen::Vector3d target_pos = b_spline_->evaluate(time_elapsed, 0);
  if (Eigen::Vector3d(target_pos - mavPos_).norm() < 0.5)
  {
    if (time_elapsed < b_spline_->maxValidTime())
    {
      time_elapsed += 0.01;
      traj_pt_counter = 0;
    }
  }
}

void MotionController::spin()
{
  ros::Rate rate(100);
  controller_ready = true;

  while (ros::ok())
  {
    switch (controller_cmd)
    {
      case POSControllerCMD::POS_CMD_LAND:
      {
        generateTraj(mavPos_, Eigen::Vector3d(mavPos_.x(), mavPos_.y(), -0.1), Eigen::Vector4d(0.2, 0.2, 0, 0));
        time_elapsed = 0;
        traj_type = TRAJ_LAND;
        controller_cmd = POSControllerCMD::POS_CMD_FOL_TRAJ;
        break;
      }

      case POSControllerCMD::POS_CMD_TAKEOFF:
      {
        generateTraj(mavPos_, Eigen::Vector3d(mavPos_.x(), mavPos_.y(), 2), Eigen::Vector4d(0.2, 0.2, 0, 0));
        time_elapsed = 0;
        traj_type = TRAJ_TAKEOFF;
        controller_cmd = POSControllerCMD::POS_CMD_FOL_TRAJ;
        break;
      }

      case POSControllerCMD::POS_CMD_FOL_TRAJ:
      {
        if (traj_type == TRAJ_MISSION)
        {
          switch (mission_controller_state)
          {
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

        else
        {
          followTraj();
          double dist = Eigen::Vector3d(traj_tar - mavPos_).norm();
          if (dist < 0.2)
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
        flag_reset = true;
        controller_cmd = POSControllerCMD::POS_CMD_FOL_TRAJ;
        controller_state = CONTROLLER_STATE_MISSION_FOLLOW;
        break;
      }

      case POSControllerCMD::POS_CMD_NONE:
      {
        break;
      }
    }

    computeBodyRateCmd(cmdBodyRate_);
    pubRateCommands(cmdBodyRate_);

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
