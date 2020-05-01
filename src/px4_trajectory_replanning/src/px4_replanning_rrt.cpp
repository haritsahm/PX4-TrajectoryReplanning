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

#include <px4_trajectory_replanning/px4_mission_controller.h>

MissionController::MissionController(ros::NodeHandle& nh)
  : nh_(nh)
  , running_allow(false)
  , running_hold(false)
  , initialized(false)
  , mission_state(MissionCMD::MISSION_HOLD)
  , prev_mission_state(99)
  , core_debug(true)
  , process_debug(true)
  , gazebo_sim(true)
  , new_wp_initialized(true)
{
  // init();
}

void MissionController::init()
{
  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  // {
  //   ros::console::notifyLoggerLevelsChanged();
  // }

  config_path = ros::package::getPath("px4_trajectory_replanning") + "/config/config.yaml";
  loadParam(config_path);

  occ_marker_pub = nh_.advertise<visualization_msgs::Marker>("ring_buffer/occupied", 5);
  free_marker_pub = nh_.advertise<visualization_msgs::Marker>("ring_buffer/free", 5);
  dist_marker_pub = nh_.advertise<visualization_msgs::Marker>("ring_buffer/distance", 5);
  trajectory_pub = nh_.advertise<geometry_msgs::Point>("trajectory/command/point", 50);
  current_traj_pub = nh_.advertise<visualization_msgs::MarkerArray>("trajectory/optimal_trajectory", 50, true);
  rrt_planner_pub = nh_.advertise<visualization_msgs::MarkerArray>("trajectory/rrt_trajectory", 50, true);

  traj_marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("trajectory/global_trajectory", 50, true);
  traj_checker_pub = nh_.advertise<visualization_msgs::Marker>("trajectory/checker_trajectory", 50, true);

  mission_command_server =
      nh_.advertiseService("controllers/mission_command_param", &MissionController::missionCommandParam, this);
  mission_controller_cmd_client =
  nh_.serviceClient<px4_trajectory_replanning::POS_CONTROLLER_COMMAND>("controllers/mission_controller_cmd");

  camera_info_sub_ = nh_.subscribe("/camera/depth/camera_info", 50, &MissionController::cameraInfoCallback, this);
  //   message_filters::Subscriber<sensor_msgs::Image> depth_image_sub_;

  depth_image_sub_.subscribe(nh_, "/camera/depth/image_raw", 50);
  tf_filter_ = new tf::MessageFilter<sensor_msgs::Image>(depth_image_sub_, listener, "map", 50);
  tf_filter_->registerCallback(boost::bind(&MissionController::depthImageCallback, this, _1));

  // queue_thread = boost::thread(boost::bind(&MissionController::queueThread, this));

  ros::NodeHandle pnh("~");
  std::string map_default = "cones";
  pnh.param("mission_type", mission_map, map_default);
  pnh.param("use_wp", mission_use_wp, true);
  pnh.param("gazebo_sim", gazebo_sim, true);

  traj_checker_marker.header.frame_id = "map";
  traj_checker_marker.ns = "trajectory_checker";
  traj_checker_marker.id = 0;
  traj_checker_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  traj_checker_marker.action = visualization_msgs::Marker::MODIFY;
  traj_checker_marker.scale.x = 0.1;
  traj_checker_marker.scale.y = 0.1;
  traj_checker_marker.scale.z = 0.1;
  traj_checker_marker.color.a = 1.0;

  traj_checker_marker.lifetime = ros::Duration(0);

  c_obs.r = 1.0;
  c_obs.g = 0.5;
  c_obs.b = 1.0;
  c_obs.a = 1.0;

  c_free.r = 1.0;
  c_free.g = 1.0;
  c_free.b = 0.0;
  c_free.a = 1.0;

  // Starting thread
  ROS_INFO("Starting Thread");
}

void MissionController::queueThread()
{
  ros::NodeHandle node_handle;
  ros::CallbackQueue callback_queue;
  node_handle.setCallbackQueue(&callback_queue);

  mission_command_server =
      node_handle.advertiseService("controllers/mission_command_param", &MissionController::missionCommandParam, this);
  mission_controller_cmd_client = node_handle.serviceClient<px4_trajectory_replanning::POS_CONTROLLER_COMMAND>("control"
                                                                                                               "lers/"
                                                                                                               "mission"
                                                                                                               "_contro"
                                                                                                               "ller_"
                                                                                                               "cmd");

  ros::WallDuration duration(0.001);
  while (node_handle.ok())
    callback_queue.callAvailable(duration);
}

bool MissionController::missionCommandParam(px4_trajectory_replanning::MAV_MISSION_COMMAND::Request& req,
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
    ROS_DEBUG_STREAM_COND_NAMED(core_debug, "mission_planner", "Incoming Command : " << mission_state);
  }

  else if (req.header.frame_id == "Offboard Controller")
  {
    running_allow = req.allowed;
    if (running_allow)
      ROS_WARN("Mission allowed run");
    else
      ROS_WARN("Mission disabled");
  }

  return true;
}

void MissionController::saveParam()
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

void MissionController::loadParam(const std::string path)
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

void MissionController::cameraInfoCallback(const sensor_msgs::CameraInfo& msg)
{
  camera_info_msg_ = msg;
}

void MissionController::depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
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

  float fx = camera_info_msg_.K[0];
  float fy = camera_info_msg_.K[4];
  float cx = camera_info_msg_.K[2];
  float cy = camera_info_msg_.K[5];

  tf::StampedTransform transform;

  try
  {
    listener.lookupTransform("map", msg->header.frame_id, msg->header.stamp, transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_DEBUG_STREAM_COND_NAMED(core_debug, "mission_planner", "Couldn't get transform");
    ROS_WARN("%s", ex.what());
    return;
  }

  Eigen::Affine3d dT_w_c;
  tf::transformTFToEigen(transform, dT_w_c);

  Eigen::Affine3f T_w_c = dT_w_c.cast<float>();

  float* data = (float*)cv_ptr->image.data;

  auto t1 = std::chrono::high_resolution_clock::now();

  ewok::EuclideanDistanceRingBuffer<6>::PointCloud cloud1;

  for (int u = 0; u < cv_ptr->image.cols; u += 4)
  {
    for (int v = 0; v < cv_ptr->image.rows; v += 4)
    {
      float val = data[v * cv_ptr->image.cols + u];

      // ROS_INFO_STREAM(val);

      if (std::isfinite(val))
      {
        Eigen::Vector4f p;
        p[0] = val * (u - cx) / fx;
        p[1] = val * (v - cy) / fy;
        p[2] = val;
        p[3] = 1;

        p = T_w_c * p;

        cloud1.push_back(p);
      }
    }
  }

  Eigen::Vector3f origin = (T_w_c * Eigen::Vector4f(0, 0, 0, 1)).head<3>();

  auto t2 = std::chrono::high_resolution_clock::now();

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

  edrb->insertPointCloud(cloud1, origin);

  visualization_msgs::Marker m_occ, m_free;
  edrb->getMarkerOccupied(m_occ);
  edrb->getMarkerFree(m_free);

  m_occ.header.frame_id = "map";
  m_free.header.frame_id = "map";

  occ_marker_pub.publish(m_occ);
  free_marker_pub.publish(m_free);
}

void MissionController::missionWaypoint()
{
  Eigen::Vector4d limits(config.max_velocity, config.max_acceleration, 0, 0);

  Eigen::Vector3d start, middle, stop;
  start = mission_config[mission_map]["start"].as<Eigen::Vector3d>();
  middle = mission_config[mission_map]["middle"].as<Eigen::Vector3d>();
  stop = mission_config[mission_map]["stop"].as<Eigen::Vector3d>();
  // double start_x, start_y, start_z, start_yaw;
  // pnh.param("start_x", start_x, 0.0);
  // pnh.param("start_y", start_y, 0.0);
  // pnh.param("start_z", start_z, 0.0);
  // pnh.param("start_yaw", start_yaw, 0.0);

  // double middle_x, middle_y, middle_z, middle_yaw;
  // pnh.param("middle_x", middle_x, 0.0);
  // pnh.param("middle_y", middle_y, 0.0);
  // pnh.param("middle_z", middle_z, 0.0);
  // pnh.param("middle_yaw", middle_yaw, 0.0);

  // double stop_x, stop_y, stop_z, stop_yaw;
  // pnh.param("stop_x", stop_x, 0.0);
  // pnh.param("stop_y", stop_y, 0.0);
  // pnh.param("stop_z", stop_z, 0.0);
  // pnh.param("stop_yaw", stop_yaw, 0.0);

  ROS_INFO_STREAM("Started hovering with parameters: start " << start << " middle " << middle << " stop " << stop);

  ROS_INFO("dt: %f, num_opt_points: %d", dt, config.num_opt_points);

  /**
   * Generate Original Spline Trajectory
   */

  ewok::Polynomial3DOptimization<10> to(limits * 0.4);

  {
    typename ewok::Polynomial3DOptimization<10>::Vector3Array vec;

    if (mission_use_wp)
    {
      vec.push_back(start);
      vec.push_back(middle);
      vec.push_back(stop);
    }

    traj = to.computeTrajectory(vec);

    visualization_msgs::MarkerArray traj_marker;
    traj->getVisualizationMarkerArray(traj_marker, "gt", Eigen::Vector3d(1, 0, 1));
    traj_marker_pub.publish(traj_marker);
  }

  edrb.reset(new ewok::EuclideanDistanceRingBuffer<POW>(config.resolution, 1.0));
  ewok::UniformBSpline3D<6> spline_(dt);

  path_planner.reset(
      new ewok::RRTStar3D<POW>(traj, config.step_size, config.rrt_factor, config.uav_radius, config.max_solve_t, dt));
  path_planner->setDistanceBuffer(edrb);

  for (int i = 0; i < config.num_opt_points; i++)
  {
    path_planner->addControlPoint(start.cast<float>());
  }

  path_planner->setNumControlPointsOptimized(config.num_opt_points);
  path_planner->setHeight(start.cast<float>());

  geometry_msgs::Point p;
  p.x = start.x();
  p.y = start.y();
  p.z = start.z();

  trajectory_pub.publish(p);
}

void MissionController::spin()
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

  ROS_INFO("Ready to Run");

  double current_time = 0;
  bool start_pt_found = false, finish_pt_found = false;
  bool flag_holdSpline = false;
  bool flag_useImmidiate = false;

  int hold_counter = 0;

  double time_start = 0, time_stop = 0;
  Eigen::Vector3f start_point, stop_point;

  std::list<Eigen::Vector3d> ctrl_points_;
  std::vector<Eigen::Vector3f> obs_points;
  std::vector<bool> prev_ctrl_pts_bool;
  ros::Rate r(1 / dt);

  Eigen::Vector3f pt_hold, point_prev, point_next, last_point;

  while (ros::ok())
  {
    if (new_wp_initialized)
    {
      hold_counter = time_start = time_stop = current_time = 0;
      flag_holdSpline = false;
      flag_useImmidiate = false;
      start_pt_found = false;
      finish_pt_found = false;
      new_wp_initialized = false;
      missionWaypoint();
    }

    tf::StampedTransform transform;

    try
    {
      listener.lookupTransform("map", "base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException& ex)
    {
      ROS_DEBUG_STREAM_COND_NAMED(core_debug, "mission_planner", "Couldn't get transform");
      ROS_WARN("%s", ex.what());
    }

    std::cout << "RUNNING " << std::endl;

    Eigen::Affine3d base_link;
    tf::transformTFToEigen(transform, base_link);

    path_planner->setRobotPos(base_link.translation().cast<float>());

    if (mission_state != prev_mission_state)
    {
      px4_trajectory_replanning::POS_CONTROLLER_COMMAND pos_controller_srv;

      switch (mission_state)
      {
        case MissionCMD::MISSION_STOP:
        {
          ROS_DEBUG_STREAM_COND_NAMED(process_debug, "mission_planner", "Mission Command to STOP");
          pos_controller_srv.request.cmd_req = POS_CMD_MISSION_STOP;
          // mission_controller_cmd_client.call(pos_controller_srv);
          running_allow = false;
          running_hold = true;
          mission_state = MissionCMD::MISSION_PUB;
          break;
        }
        case MissionCMD::MISSION_HOLD:
        {
          ROS_DEBUG_STREAM_COND_NAMED(process_debug, "mission_planner", "Mission Command to HOLD");
          pos_controller_srv.request.cmd_req = POS_CMD_MISSION_HOLD;
          // mission_controller_cmd_client.call(pos_controller_srv);
          running_hold = true;
          mission_state = MissionCMD::MISSION_PUB;
          break;
        }
        case MissionCMD::MISSION_START:
        {
          ROS_DEBUG_STREAM_COND_NAMED(process_debug, "mission_planner", "Mission Command to START");
          pos_controller_srv.request.cmd_req = POS_CMD_MISSION_START;
          // mission_controller_cmd_client.call(pos_controller_srv);
          running_allow = true;
          running_hold = false;
          mission_state = MissionCMD::MISSION_PUB;
          break;
        }

        case MissionCMD::MISSION_PUB:
        {
          ROS_DEBUG_STREAM_COND_NAMED(process_debug, "mission_planner", "Publishing Command");
          if (mission_controller_cmd_client.exists())
            mission_controller_cmd_client.call(pos_controller_srv);
          break;
        }
      }
    }
    prev_mission_state = mission_state;

    std::cout << "calculating" << std::endl;
    
    if (current_time < traj->duration() && running_allow)
    {
      ROS_INFO("Processing");
      std::vector<Eigen::Vector3d> ctrl_points = traj->evaluates(current_time, dt, config.num_pt_window, 0);
      Eigen::Vector3d end_segment_point = traj->evaluateEndSegment(current_time, 0);
      std::vector<Eigen::Vector3f> ctrl_points_f;
      for (Eigen::Vector3d pt : ctrl_points)
      {
        ctrl_points_f.push_back(pt.cast<float>());
      }

      if (Eigen::Vector3f(base_link.translation().cast<float>() - ctrl_points_f[0]).norm() > config.max_check_dist)
        flag_holdSpline = true;
      else
        flag_holdSpline = false;

      std::vector<Eigen::Vector3f> ctrl_pts_near;

      if (edrb->insideVolume(ctrl_points_f))
      {
        std::vector<bool> ctrl_pts_bool = edrb->isNearObstacle(ctrl_points_f, config.clear_distance);

        for (int i = 0; i < ctrl_pts_bool.size() - 1; i++)
        {
          bool prev_bool = ctrl_pts_bool[i];
          bool next_bool = ctrl_pts_bool[i + 1];
          point_prev = ctrl_points_f[i];
          point_next = ctrl_points_f[i + 1];

          if (!prev_bool && next_bool && start_point != point_prev)
          {
            if (!start_pt_found && !path_planner->isRunnning())
            {
              time_start = current_time + (dt * i);
              obs_points.push_back(point_next);
              start_pt_found = true;
              ROS_WARN_STREAM_COND_NAMED(core_debug, "mission_planner", "FOUND START");
              std::cout << "Source : \n";
              std::cout << point_prev << std::endl;
              pt_hold = start_point = point_prev;
              path_planner->setStartPoint(point_prev);
              path_planner->setTargetPoint(end_segment_point.cast<float>(), false);
              path_planner->findPath(config.num_iter);
            }
            else if (start_pt_found && path_planner->isRunnning())
            {
              time_start = current_time + (dt * i);
              pt_hold = start_point = point_prev;
              path_planner->setStartPoint(point_prev);
            }
          }

          if (prev_bool && !next_bool && stop_point != point_next)
          {
            if (start_pt_found && (obs_points.size() == 1))
            {
              start_pt_found = false;
              obs_points.clear();
              path_planner->reset();
            }

            else if (start_pt_found && path_planner->isRunnning())
            {
              ROS_WARN_STREAM_COND_NAMED(core_debug, "mission_planner", "FOUND TARGET");
              flag_holdSpline = true;
              finish_pt_found = true;
              time_stop = current_time + (dt * (i + 1));
              path_planner->setTargetPoint(point_next, true);
            }
          }

          if (!prev_bool && !next_bool)
          {
            if (std::find(obs_points.begin(), obs_points.end(), point_prev) == obs_points.end())
            {
              obs_points.push_back(point_prev);
            }
            if (std::find(obs_points.begin(), obs_points.end(), point_next) == obs_points.end())
            {
              obs_points.push_back(point_next);
            }
          }
        }

        if (start_pt_found && path_planner->isSolved())
        {
          ROS_DEBUG_STREAM_COND_NAMED(core_debug, "mission_planner", "Getting Data from RRT");
          finish_pt_found = false;
          hold_counter = 0;
          std::list<Eigen::Vector3f> path_points = path_planner->getPathPoints();
          for (Eigen::Vector3f pt : path_points)
          {
            path_planner->addControlPoint(pt);
          }
          start_pt_found = false;
          flag_holdSpline = false;
          current_time = time_stop;
        }

        else if (start_pt_found && !path_planner->isSolved() && path_planner->immidiatePath())
        {
          ROS_DEBUG_STREAM_COND_NAMED(core_debug, "mission_planner", "Using Immidiate Path");
          hold_counter = 0;
          flag_useImmidiate = true;
          std::list<Eigen::Vector3f> temp_path_points = path_planner->getImmidiatePath();
          pt_hold = temp_path_points.back();
          for (Eigen::Vector3f pt : temp_path_points)
          {
            path_planner->addControlPoint(pt);
          }

          ROS_DEBUG_STREAM_COND_NAMED(core_debug, "mission_planner", "Path from Immidiate Ready");
          flag_holdSpline = false;
        }

        // If RRT Not Solved use starting point
        else if (start_pt_found && !path_planner->isSolved() && !path_planner->immidiatePath())
        {
          ROS_DEBUG_STREAM_COND_NAMED(core_debug, "mission_planner", "Still Solving RRT");
          if (current_time < time_start)
          {
            if (last_point != ctrl_points_f[0])
            {
              hold_counter = 0;
              ROS_DEBUG_STREAM_COND_NAMED(core_debug, "mission_planner", "Using SPline");
              path_planner->addControlPoint(ctrl_points_f[0]);
            }
          }
          else
          {
            if (hold_counter <= 5)
            {
              path_planner->addControlPoint(pt_hold);
              hold_counter++;
            }
          }
        }

        // Using Spline
        else if (!start_pt_found && !path_planner->isRunnning())
        {
          if (!ctrl_pts_bool[0])
            //                        if(std::find(ctrl_points_.begin(), ctrl_points_.end(),
            //                        ctrl_points_f[0].cast<double>()) == ctrl_points_.end())
            if (last_point != ctrl_points_f[0])
            {
              ROS_DEBUG_STREAM_COND_NAMED(core_debug, "mission_planner", "Using SPline");
              path_planner->addControlPoint(ctrl_points_f[0]);
            }
        }

        for (int i = 0; i < ctrl_pts_bool.size(); i++)
        {
          bool obs = ctrl_pts_bool[i];
          Eigen::Vector3f point = ctrl_points_f[i];
          geometry_msgs::Point p;
          p.x = point(0);
          p.y = point(1);
          p.z = point(2);

          if (obs)
          {
            traj_checker_marker.colors.push_back(c_obs);
            traj_checker_marker.points.push_back(p);
          }

          else
          {
            traj_checker_marker.colors.push_back(c_free);
            traj_checker_marker.points.push_back(p);
          }
        }

        prev_ctrl_pts_bool = ctrl_pts_bool;
        last_point = ctrl_points_f[0];
        if (!flag_holdSpline && !running_hold)
          current_time += dt;
      }

      traj_checker_pub.publish(traj_checker_marker);
    }

    

    if (running_allow && !running_hold &&
        (path_planner->isSolved() || (path_planner->immidiatePath() && flag_useImmidiate)))
    {
      ROS_WARN_STREAM_COND_NAMED(core_debug, "mission_planner", "FOUND SOLUTION");
      flag_useImmidiate = false;
      visualization_msgs::MarkerArray rrt_marker;
      rrt_marker.markers.resize(2);
      path_planner->getSolutionMarker(rrt_marker.markers[0], "rrt_trajectory_markers", 0);
      path_planner->getTreeMarker(rrt_marker.markers[1], "rrt_trajectory_markers", 1);

      rrt_marker.markers[0].header.frame_id = "map";
      rrt_marker.markers[1].header.frame_id = "map";

      if (rrt_marker.markers[0].points.size() > 0)
        rrt_planner_pub.publish(rrt_marker);

      path_planner->resetImmidiatePath();

      if (path_planner->isSolved())
        path_planner->reset();
    }

    if (current_time > ros::Duration(5).toSec() && running_allow && !running_hold)
    {
      ROS_INFO("PUBLISHING");
      visualization_msgs::MarkerArray sol_traj_marker;
      path_planner->getTrajectoryMarkers(sol_traj_marker);

      sol_traj_marker.markers[0].header.frame_id = "map";
      sol_traj_marker.markers[1].header.frame_id = "map";

      current_traj_pub.publish(sol_traj_marker);

      Eigen::Vector3f pc = path_planner->getFirstTrajPoint();

      geometry_msgs::Point pp;
      pp.x = pc[0];
      pp.y = pc[1];
      pp.z = pc[2];
      trajectory_pub.publish(pp);

      path_planner->addLastControlPoint();
    }

    r.sleep();
    ros::spinOnce();
  }

  ROS_INFO_STREAM("Finished");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "px4_replanning_rrt_node");
  ros::NodeHandle nh;
  MissionController mc(nh);
  mc.init();
  mc.spin();
}
