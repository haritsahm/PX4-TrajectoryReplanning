/**
 * This file is part of Ewok.
 *
 * Copyright 2017 Vladyslav Usenko, Technical University of Munich.
 * Developed by Vladyslav Usenko <vlad dot usenko at tum dot de>,
 * for more information see <http://vision.in.tum.de/research/robotvision/replanning>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * Ewok is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Ewok is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Ewok. If not, see <http://www.gnu.org/licenses/>.
 */

#include <px4_trajectory_replanning/px4_trajectory_controller.h>

TrajectoryController::TrajectoryController(ros::NodeHandle& nh)
  : nh_(nh)
  , running_allow(false)
  , running_hold(false)
  , initialized(false)
  , mission_state(POSControllerCMD::POS_CMD_MISSION_HOLD)
  , prev_mission_state(99)
  , core_debug(false)
  , process_debug(false)
  , gazebo_sim(true)
  , new_wp_initialized(false)
{
  // init();
}

void TrajectoryController::init()
{
  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  // {
  //   ros::console::notifyLoggerLevelsChanged();
  // }

  config_path = ros::package::getPath("px4_trajectory_replanning") + "/config/config.yaml";
  loadParam(config_path);

  occ_marker_pub = nh_.advertise<visualization_msgs::Marker>("ring_buffer/occupied", 50);
  free_marker_pub = nh_.advertise<visualization_msgs::Marker>("ring_buffer/free", 50);
  trajectory_pub = nh_.advertise<geometry_msgs::Point>("trajectory/command/point", 50);
  current_traj_pub = nh_.advertise<visualization_msgs::MarkerArray>("trajectory/optimal_trajectory", 50, true);

  rrt_tree_pub = nh_.advertise<visualization_msgs::Marker>("trajectory/rrt_tree", 1, true);
  rrt_solution_pub = nh_.advertise<visualization_msgs::Marker>("trajectory/rrt_solution", 1, true);
  rrt_property_pub = nh_.advertise<visualization_msgs::MarkerArray>("trajectory/rrt_property", 1, true);

  traj_marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("trajectory/global_trajectory", 50, true);
  traj_checker_pub = nh_.advertise<visualization_msgs::Marker>("trajectory/checker_trajectory", 50, true);

  image_transport::ImageTransport it(nh_);
  sub_image = it.subscribe("/camera/depth/image_raw", 100, &TrajectoryController::depthImageCallback, this);
  camera_info_sub_ = nh_.subscribe("/camera/depth/camera_info", 50, &TrajectoryController::cameraInfoCallback, this);
  robot_pos_subscriber = nh_.subscribe("/mavros/local_position/odom",10, &TrajectoryController::OdometryCallback, this);

  queue_thread = boost::thread(boost::bind(&TrajectoryController::queueThread, this));

  ros::NodeHandle pnh("~");
  std::string map_default = "cones";
  pnh.param("mission_type", mission_map, map_default);
  pnh.param("use_wp", mission_use_wp, true);
  pnh.param("gazebo_sim", gazebo_sim, true);

  traj_checker_marker.lifetime = ros::Duration(0);

  c_obs.r = 1.0;
  c_obs.g = 0.5;
  c_obs.b = 1.0;
  c_obs.a = 1.0;

  c_free.r = 1.0;
  c_free.g = 1.0;
  c_free.b = 0.0;
  c_free.a = 1.0;

  edrb.reset(new ewok::EuclideanDistanceRingBuffer<6, int16_t, double>(config.resolution, 1.0));

  path_planner.reset(
        new ewok::RRTStar3D<6, double>(config.step_size, config.rrt_factor, config.uav_radius, config.max_solve_t, dt));
  path_planner->setDistanceBuffer(edrb);

  // Starting thread
  ROS_INFO("Starting Thread");
}

void TrajectoryController::queueThread()
{
  ros::NodeHandle node_handle;
  ros::CallbackQueue callback_queue;
  node_handle.setCallbackQueue(&callback_queue);

  mission_command_server =
      node_handle.advertiseService("controllers/mission_command_param", &TrajectoryController::missionCommandParam, this);
  mission_controller_cmd_client =
      node_handle.serviceClient<px4_trajectory_replanning::POS_CONTROLLER_COMMAND>
      ("controllers/mission_controller_cmd");

  ros::WallDuration duration(0.01);
  while (node_handle.ok())
    callback_queue.callAvailable(duration);
}

bool TrajectoryController::missionCommandParam(px4_trajectory_replanning::MAV_MISSION_COMMAND::Request& req,
                                               px4_trajectory_replanning::MAV_MISSION_COMMAND::Response& res)
{
  if (req.header.frame_id == "UI Interface")
  {
    if (req.request_param == true)
    {
      res.response = true;
      res.config = config;
    }
    else if (req.set_param == true)
    {
      if (req.save == false)
        config = req.config;
      else
      {
        saveParam();
      }
    }

    if (req.reset_param)
      new_wp_initialized = true;

    mission_state = req.mission_command;
    ROS_DEBUG_STREAM_COND_NAMED(core_debug, "mission_planner", "Incoming Command for RRT : " << mission_state);
  }

  else if (req.header.frame_id == "Offboard Controller")
  {
    running_allow = req.allowed;
    if (running_allow)
    {
      new_wp_initialized = true;
      ROS_WARN("Mission allowed run");}
    else
      ROS_WARN("Mission disabled");
  }

  return true;
}

void TrajectoryController::saveParam()
{
  YAML::Node node;
  try
  {
    // load yaml
    node = YAML::LoadFile(config_path.c_str());
  }

  catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
  }

  YAML::Node node_pp = node["path_plnanning"];

  node_pp["max_velocity"] = config.max_velocity;
  node_pp["max_acceleration"] = config.max_acceleration;
  node_pp["resolution"] = config.resolution;
  node_pp["num_opt_points"] = config.num_opt_points;

  node_pp["step_size"] = config.step_size;
  node_pp["rrt_factor"] = config.rrt_factor;
  node_pp["max_solve_t"] = config.max_solve_t;
  node_pp["uav_radius"] = config.uav_radius;
  node_pp["num_iter"] = config.num_iter;

  node_pp["num_pt_window"] = config.num_pt_window;
  node_pp["clear_distance"] = config.clear_distance;
  node_pp["max_check_dist"] = config.max_check_dist;

  std::ofstream fout(config_path.c_str());
  fout << node_pp;
}

void TrajectoryController::loadParam(const std::string path)
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

  YAML::Node node_pp = node["path_planning"];

  config.max_velocity = node_pp["max_velocity"].as<float>();
  config.max_acceleration = node_pp["max_acceleration"].as<float>();
  config.resolution = node_pp["resolution"].as<float>();
  config.num_opt_points = node_pp["num_opt_points"].as<int>();

  config.step_size = node_pp["step_size"].as<float>();
  config.rrt_factor = node_pp["rrt_factor"].as<float>();
  config.max_solve_t = node_pp["max_solve_t"].as<float>();
  config.uav_radius = node_pp["uav_radius"].as<float>();
  config.num_iter = node_pp["num_iter"].as<int>();

  config.num_pt_window = node_pp["num_pt_window"].as<int>();
  config.clear_distance = node_pp["clear_distance"].as<float>();
  config.max_check_dist = node_pp["max_check_dist"].as<float>();

  YAML::Node node_g = node["global"];
  dt = node_g["dt"].as<double>();

  mission_config = node["mission"];
}

void TrajectoryController::cameraInfoCallback(const sensor_msgs::CameraInfo& msg)
{
  camera_info_msg_ = msg;
}

void TrajectoryController::depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  //    ROS_DEBUG_STREAM_COND_NAMED(core_debug, "mission_planner", "recieved depth image");
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  double fx = 554.254;
  double fy = 554.254;
  double cx = 320.5;
  double cy = 240.5;

  tf::StampedTransform transform;
  try
  {
    listener.lookupTransform("map", msg->header.frame_id, ros::Time(0), transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_DEBUG_STREAM_COND_NAMED(core_debug, "mission_planner", "Couldn't get transform");
    ROS_WARN("cam %s", ex.what());
    return;
  }

  Eigen::Affine3d dT_w_c;
  tf::transformTFToEigen(transform, dT_w_c);

  float* data = (float*)cv_ptr->image.data;

  typename ewok::EuclideanDistanceRingBuffer<6, int16_t, double>::PointCloud cloud1;

  for (int u = 0; u < cv_ptr->image.cols; u += 4)
  {
    for (int v = 0; v < cv_ptr->image.rows; v += 4)
    {
      float val = data[v * cv_ptr->image.cols + u];

      // ROS_INFO_STREAM(val);

      if (std::isfinite(val))
      {
        Eigen::Vector4d p;
        p[0] = val * (u - cx) / fx;
        p[1] = val * (v - cy) / fy;
        p[2] = val;
        p[3] = 1;

        p = dT_w_c * p;

        cloud1.push_back(p);
      }
    }
  }

  Eigen::Vector3d origin = dT_w_c.translation();//(dT_w_c * Eigen::Vector4d(0, 0, 0, 1)).head<3>();

  if (!initialized)
  {
    Eigen::Vector3i idx;
    edrb->getIdx(origin, idx);

    ROS_INFO_STREAM("Origin: " << origin.transpose() << " idx " << idx.transpose());

    edrb->setOffset(idx);

    initialized = true;
  }
  else
  {
    Eigen::Vector3i origin_idx, offset, diff;
    edrb->getIdx(origin, origin_idx);

    offset = edrb->getVolumeCenter();
    diff = origin_idx - offset;

    while (diff.array().any())
    {
      // ROS_DEBUG_STREAM_COND_NAMED(core_debug, "mission_planner", "Moving Volume");
      edrb->moveVolume(diff);

      offset = edrb->getVolumeCenter();
      diff = origin_idx - offset;
    }
  }

  mutex.lock();
  edrb->insertPointCloud(cloud1, origin);
  mutex.unlock();

  visualization_msgs::Marker m_occ, m_free;
  edrb->getMarkerOccupied(m_occ);
  edrb->getMarkerFree(m_free);


  m_occ.header.frame_id = "map";
  m_free.header.frame_id = "map";

  occ_marker_pub.publish(m_occ);
  free_marker_pub.publish(m_free);
  // ROS_INFO("Finished Publishing");
}

void TrajectoryController::missionWaypoint()
{
  Eigen::Vector4d limits(config.max_velocity, config.max_acceleration, 0, 0);

  Eigen::Vector3d start, middle, stop;
  start = mission_config[mission_map]["start"].as<Eigen::Vector3d>();
  middle = mission_config[mission_map]["middle"].as<Eigen::Vector3d>();
  stop = mission_config[mission_map]["stop"].as<Eigen::Vector3d>();

  ROS_INFO_STREAM("Started hovering with parameters: start " << start << " middle " << middle << " stop " << stop);

  /**
   * Generate Original Spline Trajectory
   */

  ewok::Polynomial3DOptimization<10, double> to(limits * 0.4);

  {
    typename ewok::Polynomial3DOptimization<10, double>::Vector3Array vec;

    if (mission_use_wp)
    {
      vec.push_back(start);
      vec.push_back(middle);
      vec.push_back(stop);
    }

    traj = to.computeTrajectory(vec);

    visualization_msgs::MarkerArray traj_marker;
    traj->getVisualizationMarkerArray(traj_marker, "gt", Eigen::Vector3d(1, 0, 1));
    for(auto marker: traj_marker.markers)
    {
      marker.lifetime = ros::Duration(0);
    }
    traj_marker_pub.publish(traj_marker);
  }

  path_planner->setPolynomialTrajectory(traj);
  path_planner->setLogPath(log_path+file_name, false); //save log

  for (int i = 0; i < config.num_opt_points; i++)
  {
    path_planner->addControlPoint(start);
  }

  path_planner->setHeight(start, true);

  geometry_msgs::Point p;
  p.x = start.x();
  p.y = start.y();
  p.z = start.z();

  trajectory_pub.publish(p);
}

void TrajectoryController::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{
  Eigen::Affine3d base_link;
  geometry_msgs::Pose pose = odometry_msg->pose.pose;
  tf::poseMsgToEigen(pose, base_link);
  path_planner->setRobotPos(base_link.translation());
  path_planner->setRobotPose(base_link);
}

void TrajectoryController::RRTPublisher(const ros::TimerEvent& event)
{
  if(path_planner->isRunning())
  {
    ROS_INFO_COND(core_debug, "Publish RRT Visualizer");
    visualization_msgs::Marker rrt_tree_marker, rrt_solution_marker;
    visualization_msgs::MarkerArray rrt_property_marker;
    path_planner->getTreeMarker(rrt_tree_marker, "rrt_tree_marker", 1);
    rrt_tree_marker.header.frame_id = "map";
    if(path_planner->RRTVisualize())
    {
      path_planner->getSolutionMarker(rrt_solution_marker, "rrt_solution_markers", 0, Eigen::Vector3f(0,0,1), 0.05);
      rrt_solution_marker.header.frame_id = "map";
      if(rrt_solution_marker.points.size() > 0)
        rrt_solution_pub.publish(rrt_solution_marker);
    }

    if (rrt_tree_marker.points.size() > 0)
      rrt_tree_pub.publish(rrt_tree_marker);

    rrt_property_marker.markers.resize(2);
    if(path_planner->solutionFound())
    {
      path_planner->getRRTProperty(rrt_property_marker.markers[0], "rrt_property_marker", 0, Eigen::Vector4f(1,1,1,0.4));
    }

    path_planner->getRRTProperty(rrt_property_marker.markers[1], "rrt_state_marker", 1, Eigen::Vector4f(1,0.5,0,0.6));
    rrt_property_marker.markers[0].header.frame_id = "map";
    rrt_property_marker.markers[1].header.frame_id = "map";
    rrt_property_pub.publish(rrt_property_marker);

  }
}

void TrajectoryController::RRTProcess(const ros::TimerEvent& event)
{
  if(new_wp_initialized)
  {
    missionWaypoint();
    new_wp_initialized = false;
  }

  if (mission_state != prev_mission_state)
  {
    switch (mission_state)
    {
    case POSControllerCMD::POS_CMD_MISSION_STOP:
    {
      ROS_DEBUG_STREAM_COND_NAMED(process_debug, "mission_planner", "Mission Command to STOP");
      running_allow = false;
      running_hold = true;
      break;
    }
    case POSControllerCMD::POS_CMD_MISSION_HOLD:
    {
      ROS_DEBUG_STREAM_COND_NAMED(process_debug, "mission_planner", "Mission Command to HOLD");
      running_hold = true;
      break;
    }
    case POSControllerCMD::POS_CMD_MISSION_START:
    {
      ROS_DEBUG_STREAM_COND_NAMED(process_debug, "mission_planner", "Mission Command to START");
      running_allow = true;
      running_hold = false;
      break;
    }
    }
  }
  prev_mission_state = mission_state;

  if (running_allow && !running_hold)
  {  // process rrt
    ROS_INFO_COND(core_debug, "Process Trajectory - RRT");
    path_planner->process();

    // get trajectory checker
    ROS_INFO_COND(core_debug, "Publish Trajectory Checker");
    path_planner->TrajectoryChecker(traj_checker_marker, "map");
    traj_checker_pub.publish(traj_checker_marker);

    ROS_INFO_COND(core_debug, "Publish Path Visualizer");
    visualization_msgs::MarkerArray sol_traj_marker;
    path_planner->getTrajectoryMarkers(sol_traj_marker, "spline_opitimization_markers", Eigen::Vector3d(0, 1, 0),
                                       Eigen::Vector3d(0, 1, 1));
    sol_traj_marker.markers[0].header.frame_id = "map";
    sol_traj_marker.markers[1].header.frame_id = "map";
    current_traj_pub.publish(sol_traj_marker);

    ROS_INFO_COND(core_debug, "Publish Command");

    Eigen::Vector3d pc;
    if(path_planner->getNextPt(pc))
    {
      ROS_INFO_COND(core_debug, "Publish Command");
      geometry_msgs::Point pp;
      pp.x = pc.x();
      pp.y = pc.y();
      pp.z = pc.z();
      trajectory_pub.publish(pp);
    }
  }
}

void TrajectoryController::spin()
{
  ROS_INFO("Running Thread");
  if (gazebo_sim)
  { /**
     *
     * Waiting Gazebo Running
     *
     */

    std_srvs::Empty srv;
    bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    unsigned int i = 0;

    // Trying to unpause Gazebo for 10 seconds.
    while (!unpaused)
    {
      ROS_DEBUG_STREAM_COND_NAMED(core_debug, "mission_planner",
                                  "Wait for 1 second before trying to unpause Gazebo again.");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      unpaused = ros::service::call("/gazebo/unpause_physics", srv);
      ++i;
    }

    ROS_INFO("Unpaused");

    if (!unpaused)
    {
      ROS_FATAL("Could not wake up Gazebo.");
      //   return -1;
    }
    else
    {
      ROS_DEBUG_STREAM_COND_NAMED(core_debug, "mission_planner", "Unpaused the Gazebo simulation.");
    }

    /**
     * Waiting Gazebo GUI
     *
     */

    // Wait for 5 seconds to let the Gazebo GUI show up.
    ros::Duration(5.0).sleep();
  }

  log_path = ros::package::getPath("px4_trajectory_replanning") + "/logs/";
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "log-%Y-%m-%d-%X");
  file_name = ss.str();
  ROS_INFO_STREAM("Writing log file " << log_path+file_name);

  ros::Timer rrt_process_timer = nh_.createTimer(ros::Duration(dt), &TrajectoryController::RRTProcess, this);
  ros::Timer rrt_visualizer_timer = nh_.createTimer(ros::Duration(0.002), &TrajectoryController::RRTPublisher, this);
  ros::spin();


  ROS_INFO_STREAM("Finished");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "px4_trajectory_controller_node");
  ros::NodeHandle nh;
  TrajectoryController mc(nh);
  mc.init();
  mc.spin();
}
