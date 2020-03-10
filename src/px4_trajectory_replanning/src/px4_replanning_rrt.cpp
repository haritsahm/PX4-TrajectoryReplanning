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



#include <thread>
#include <chrono>
#include <map>
#include <algorithm>
#include <math.h>

#include <Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <ewok/polynomial_3d_optimization.h>
#include <ewok/uniform_bspline_3d_optimization.h>
#include <ewok/uniform_bspline_3d.h>
#include <ewok/rrtstar3d.h>

#define bool2int (x ? 1 : 0)


const int POW = 6;

double dt ;
int num_opt_points;

bool initialized = false;

std::ofstream f_time, opt_time;


ewok::PolynomialTrajectory3D<10>::Ptr traj;
ewok::EuclideanDistanceRingBuffer<POW>::Ptr edrb;
ewok::RRTStar3D<POW>::Ptr path_planner;

ros::Publisher rrt_planner_pub, occ_marker_pub, free_marker_pub, dist_marker_pub, trajectory_pub, current_traj_pub, command_pt_pub, command_pt_viz_pub;
tf::TransformListener * listener;


void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    //    ROS_INFO("recieved depth image");

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

    const float fx = 554.254691191187;
    const float fy = 554.254691191187;
    const float cx = 320.5;
    const float cy = 240.5;

    tf::StampedTransform transform;


    try{

        listener->lookupTransform("world", msg->header.frame_id,
                                  msg->header.stamp, transform);
    }
    catch (tf::TransformException &ex) {
        ROS_INFO("Couldn't get transform");
        ROS_WARN("%s",ex.what());
        return;
    }

    Eigen::Affine3d dT_w_c;
    tf::transformTFToEigen(transform, dT_w_c);

    Eigen::Affine3f T_w_c = dT_w_c.cast<float>();

    float * data = (float *) cv_ptr->image.data;


    auto t1 = std::chrono::high_resolution_clock::now();

    ewok::EuclideanDistanceRingBuffer<POW>::PointCloud cloud1;

    for(int u=0; u < cv_ptr->image.cols; u+=4) {
        for(int v=0; v < cv_ptr->image.rows; v+=4) {
            float val = data[v*cv_ptr->image.cols + u];

            //ROS_INFO_STREAM(val);

            if(std::isfinite(val)) {
                Eigen::Vector4f p;
                p[0] = val*(u - cx)/fx;
                p[1] = val*(v - cy)/fy;
                p[2] = val;
                p[3] = 1;

                p = T_w_c * p;

                cloud1.push_back(p);
            }
        }
    }

    Eigen::Vector3f origin = (T_w_c * Eigen::Vector4f(0,0,0,1)).head<3>();

    auto t2 = std::chrono::high_resolution_clock::now();

    if(!initialized) {
        Eigen::Vector3i idx;
        edrb->getIdx(origin, idx);

        ROS_INFO_STREAM("Origin: " << origin.transpose() << " idx " << idx.transpose());

        edrb->setOffset(idx);

        initialized = true;
    } else {
        Eigen::Vector3i origin_idx, offset, diff;
        edrb->getIdx(origin, origin_idx);

        offset = edrb->getVolumeCenter();
        diff = origin_idx - offset;

        while(diff.array().any()) {
            //ROS_INFO("Moving Volume");
            edrb->moveVolume(diff);

            offset = edrb->getVolumeCenter();
            diff = origin_idx - offset;
        }


    }

    edrb->insertPointCloud(cloud1, origin);

    visualization_msgs::Marker m_occ, m_free;
    edrb->getMarkerOccupied(m_occ);
    edrb->getMarkerFree(m_free);


    occ_marker_pub.publish(m_occ);
    free_marker_pub.publish(m_free);

}

int main(int argc, char** argv){
//    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
//    {
//        ros::console::notifyLoggerLevelsChanged();
//    }
    ros::init(argc, argv, "px4_replanning_rrt_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string path = ros::package::getPath("ewok_simulation") + "/benchmarking/";

    ROS_INFO_STREAM("path: " << path);

    listener = new tf::TransformListener;

    occ_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/occupied", 5);
    free_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/free", 5);
    dist_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/distance", 5);
    command_pt_viz_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/command_pt", 5);
    trajectory_pub = nh.advertise<geometry_msgs::Point>("command/point", 10);
    current_traj_pub = nh.advertise<visualization_msgs::MarkerArray>("optimal_trajectory", 1, true);
    rrt_planner_pub = nh.advertise<visualization_msgs::MarkerArray>("rrt_trajectory", 1, true);

    ros::Publisher traj_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("global_trajectory", 1, true);
    ros::Publisher traj_checker_pub = nh.advertise<visualization_msgs::Marker>("checker_trajectory", 1, true);


    message_filters::Subscriber<sensor_msgs::Image> depth_image_sub_ ;
    depth_image_sub_.subscribe(nh, "camera/depth/image_raw", 5);

    tf::MessageFilter<sensor_msgs::Image> tf_filter_(depth_image_sub_, *listener, "world", 5);
    tf_filter_.registerCallback(depthImageCallback);


    double max_velocity, max_acceleration;
    pnh.param("max_velocity", max_velocity, 1.0);
    pnh.param("max_acceleration", max_acceleration, 2.0);

    Eigen::Vector4d limits(max_velocity, max_acceleration, 0, 0);

    double start_x, start_y, start_z, start_yaw;
    pnh.param("start_x", start_x, 0.0);
    pnh.param("start_y", start_y, 0.0);
    pnh.param("start_z", start_z, 0.0);
    pnh.param("start_yaw", start_yaw, 0.0);

    double middle_x, middle_y, middle_z, middle_yaw;
    pnh.param("middle_x", middle_x, 0.0);
    pnh.param("middle_y", middle_y, 0.0);
    pnh.param("middle_z", middle_z, 0.0);
    pnh.param("middle_yaw", middle_yaw, 0.0);

    double stop_x, stop_y, stop_z, stop_yaw;
    pnh.param("stop_x", stop_x, 0.0);
    pnh.param("stop_y", stop_y, 0.0);
    pnh.param("stop_z", stop_z, 0.0);
    pnh.param("stop_yaw", stop_yaw, 0.0);


    pnh.param("dt", dt, 0.5);
    pnh.param("num_opt_points", num_opt_points, 7);

    ROS_INFO("Started hovering example with parameters: start - %f %f %f %f, middle - %f %f %f %f, stop - %f %f %f %f",
             start_x, start_y, start_z, start_yaw,
             middle_x, middle_y, middle_z, middle_yaw,
             stop_x, stop_y, stop_z, stop_yaw);

    ROS_INFO("dt: %f, num_opt_points: %d", dt, num_opt_points);

    visualization_msgs::Marker ctrl_pts_marker;

    ctrl_pts_marker.header.frame_id = "world";
    ctrl_pts_marker.ns = "spline loop test";
    ctrl_pts_marker.id = 0;
    ctrl_pts_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    ctrl_pts_marker.action = visualization_msgs::Marker::MODIFY;
    ctrl_pts_marker.scale.x = 0.1;
    ctrl_pts_marker.scale.y = 0.1;
    ctrl_pts_marker.scale.z = 0.1;
    ctrl_pts_marker.color.a = 1.0;

    ctrl_pts_marker.lifetime = ros::Duration(0);

    std_msgs::ColorRGBA c_obs, c_free;

    c_obs.r = 1.0;
    c_obs.g = 0.5;
    c_obs.b = 1.0;
    c_obs.a = 1.0;

    c_free.r = 1.0;
    c_free.g = 1.0;
    c_free.b = 0.0;
    c_free.a = 1.0;

    /**
     * Generate Original Spline Trajectory
     */

    ewok::Polynomial3DOptimization<10> to(limits*0.4);

    {
    typename ewok::Polynomial3DOptimization<10>::Vector3Array vec;

    vec.push_back(Eigen::Vector3d(start_x, start_y, start_z));
    vec.push_back(Eigen::Vector3d(middle_x, middle_y, middle_z));
    vec.push_back(Eigen::Vector3d(stop_x, stop_y, stop_z));

    traj = to.computeTrajectory(vec);

    visualization_msgs::MarkerArray traj_marker;
    traj->getVisualizationMarkerArray(traj_marker, "gt", Eigen::Vector3d(1,0,1));
    traj_marker_pub.publish(traj_marker);
    }


    double resolution;
    pnh.param("resolution", resolution, 0.15);

    edrb.reset(new ewok::EuclideanDistanceRingBuffer<POW>(resolution, 1.0));
    ewok::UniformBSpline3D<6> spline_(dt);

    path_planner.reset(new ewok::RRTStar3D<POW>(traj, 0.25, 1.65, 1, 6, dt));
    path_planner->setDistanceBuffer(edrb);

    for (int i = 0; i < num_opt_points; i++) {
        path_planner->addControlPoint(Eigen::Vector3f(start_x, start_y, start_z));
    }

    path_planner->setNumControlPointsOptimized(num_opt_points);
    path_planner->setHeight(Eigen::Vector3f(start_x, start_y, start_z));

    /**
     *
     * Waiting Gazebo Running
     *
     */

    std_srvs::Empty srv;
    bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    unsigned int i = 0;

    // Trying to unpause Gazebo for 10 seconds.
    while (i <= 10 && !unpaused) {
        ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        unpaused = ros::service::call("/gazebo/unpause_physics", srv);
        ++i;
    }

    if (!unpaused) {
        ROS_FATAL("Could not wake up Gazebo.");
        return -1;
    }
    else {
        ROS_INFO("Unpaused the Gazebo simulation.");
    }

    /**
      * Waiting Gazebo GUI
      *
      */

    // Wait for 5 seconds to let the Gazebo GUI show up.
    ros::Duration(5.0).sleep();


    geometry_msgs::Point p;
    p.x = start_x;
    p.y = start_y;
    p.z = start_z;

    ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f, %f].",
             nh.getNamespace().c_str(),
             start_x,
             start_y,
             start_z, start_yaw);

    trajectory_pub.publish(p);

    ros::Duration(5.0).sleep();

    double current_time=0;
    bool start_pt_found = false, finish_pt_found = false;
    bool flag_holdSpline = false;
    bool flag_useImmidiate = false;

    int hold_counter = 0;

    double time_start = 0, time_stop = 0;
    Eigen::Vector3f start_point, stop_point;

    std::list<Eigen::Vector3d> ctrl_points_;
    std::vector<Eigen::Vector3f> obs_points;
    std::vector<bool> prev_ctrl_pts_bool;
    ros::Rate r(1/dt);

    Eigen::Vector3f pt_hold, point_prev, point_next, last_point;

    while(ros::ok())
    {
        r.sleep();

        tf::StampedTransform transform;


        try{

            listener->lookupTransform("world", "firefly/base_link",
                                      ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_INFO("Couldn't get transform");
            ROS_WARN("%s",ex.what());
        }

        Eigen::Affine3d base_link;
        tf::transformTFToEigen(transform, base_link);

        path_planner->setRobotPos(base_link.translation().cast<float>());

        if(current_time < traj->duration())
        {
            std::vector<Eigen::Vector3d> ctrl_points = traj->evaluates(current_time, dt, 4, 0);
            Eigen::Vector3d end_segment_point = traj->evaluateEndSegment(current_time, 0);
            std::vector<Eigen::Vector3f> ctrl_points_f;
            for(Eigen::Vector3d pt: ctrl_points)
            {
                ctrl_points_f.push_back(pt.cast<float>());
            }

            if(Eigen::Vector3f(base_link.translation().cast<float>() - ctrl_points_f[0]).norm() > 2.5)
                flag_holdSpline = true;
            else
                flag_holdSpline = false;

            std::vector<Eigen::Vector3f> ctrl_pts_near;

            if(edrb->insideVolume(ctrl_points_f))
            {
                std::vector<bool> ctrl_pts_bool = edrb->isNearObstacle(ctrl_points_f, 1.2);

                for(int i =0; i < ctrl_pts_bool.size()-1; i++)
                {
                    bool prev_bool = ctrl_pts_bool[i];
                    bool next_bool = ctrl_pts_bool[i+1];
                    point_prev = ctrl_points_f[i];
                    point_next = ctrl_points_f[i+1];

                    if(!prev_bool && next_bool && start_point != point_prev)
                    {
                        if(!start_pt_found && !path_planner->isRunnning())
                        {
                            time_start = current_time+(dt*i);
                            obs_points.push_back(point_next);
                            start_pt_found = true;
                            ROS_WARN("FOUND START");
                            std::cout << "Source : \n";
                            std::cout << point_prev << std::endl;
                            pt_hold = start_point = point_prev;
                            path_planner->setStartPoint(point_prev);
                            path_planner->setTargetPoint(end_segment_point.cast<float>(), false);
                            path_planner->findPath(5000);

                        }
                        else if(start_pt_found && path_planner->isRunnning())
                        {
                            time_start = current_time+(dt*i);
                            pt_hold = start_point = point_prev;
                            path_planner->setStartPoint(point_prev);
                        }
                    }

                    if(prev_bool && !next_bool && stop_point != point_next)
                    {
                        if(start_pt_found && (obs_points.size()==1))
                        {
                            start_pt_found = false;
                            obs_points.clear();
                            path_planner->reset();
                        }

                        else if (start_pt_found && path_planner->isRunnning())
                        {
                            ROS_WARN("FOUND TARGET");
                            flag_holdSpline = true;
                            finish_pt_found = true;
                            time_stop = current_time+(dt*(i+1));
                            path_planner->setTargetPoint(point_next, true);
                        }

                    }

                    if(!prev_bool && !next_bool)
                    {
                        if(std::find(obs_points.begin(), obs_points.end(), point_prev) == obs_points.end())
                        {
                            obs_points.push_back(point_prev);
                        }
                        if(std::find(obs_points.begin(), obs_points.end(), point_next) == obs_points.end())
                        {
                            obs_points.push_back(point_next);
                        }
                    }
                }


                if(start_pt_found && path_planner->isSolved())
                {
                    ROS_INFO("Getting Data from RRT");
                    finish_pt_found = false;
                    hold_counter = 0;
                    std::list<Eigen::Vector3f> path_points = path_planner->getPathPoints();
                    for(Eigen::Vector3f pt: path_points)
                    {
                        path_planner->addControlPoint(pt);
                    }
                    start_pt_found = false;
                    flag_holdSpline = false;
                    current_time = time_stop;
                }

                else if(start_pt_found && !path_planner->isSolved() && path_planner->immidiatePath())
                {
                    ROS_INFO("Using Immidiate Path");
                    hold_counter = 0;
                    flag_useImmidiate = true;
                    std::list<Eigen::Vector3f> temp_path_points = path_planner->getImmidiatePath();
                    pt_hold = temp_path_points.back();
                    for(Eigen::Vector3f pt: temp_path_points)
                    {
                        path_planner->addControlPoint(pt);
                    }

                    ROS_INFO("Path from Immidiate Ready");
                    flag_holdSpline = false;
                }

                // If RRT Not Solved use starting point
                else if(start_pt_found && !path_planner->isSolved() && !path_planner->immidiatePath())
                {
                    ROS_INFO("Still Solving RRT");
                    if(current_time < time_start)
                    {

                        if(last_point != ctrl_points_f[0])
                        {
                            hold_counter = 0;
                            ROS_INFO("Using SPline");
                            path_planner->addControlPoint(ctrl_points_f[0]);
                        }
                    }
                    else {
                        if(hold_counter <=5)
                        {
                            path_planner->addControlPoint(pt_hold);
                            hold_counter++;
                        }
                    }

                }

                // Using Spline
                else if(!start_pt_found && !path_planner->isRunnning())
                {
                    if(!ctrl_pts_bool[0])
//                        if(std::find(ctrl_points_.begin(), ctrl_points_.end(), ctrl_points_f[0].cast<double>()) == ctrl_points_.end())
                        if(last_point != ctrl_points_f[0])
                        {
                            ROS_INFO("Using SPline");
                            path_planner->addControlPoint(ctrl_points_f[0]);
                        }
                }

                for(int i =0; i < ctrl_pts_bool.size(); i++)
                {
                    bool obs = ctrl_pts_bool[i];
                    Eigen::Vector3f point = ctrl_points_f[i];
                    geometry_msgs::Point p;
                    p.x = point(0);
                    p.y = point(1);
                    p.z = point(2);

                    if(obs)
                    {
                        ctrl_pts_marker.colors.push_back(c_obs);
                        ctrl_pts_marker.points.push_back(p);
                    }

                    else {
                        ctrl_pts_marker.colors.push_back(c_free);
                        ctrl_pts_marker.points.push_back(p);
                    }
                }

                prev_ctrl_pts_bool = ctrl_pts_bool;
                last_point = ctrl_points_f[0];
                if(!flag_holdSpline)
                    current_time += dt;
            }


            traj_checker_pub.publish(ctrl_pts_marker);
        }

        if(path_planner->isSolved() || (path_planner->immidiatePath() && flag_useImmidiate))
        {
            ROS_WARN("FOUND SOLUTION");
            flag_useImmidiate = false;
            visualization_msgs::MarkerArray rrt_marker;
            rrt_marker.markers.resize(2);
            ROS_WARN("HERE");
            path_planner->getSolutionMarker(rrt_marker.markers[0], "rrt_trajectory_markers", 0);
            ROS_WARN("MIGHT BE HERE");
            path_planner->getTreeMarker(rrt_marker.markers[1], "rrt_trajectory_markers", 1);
            if(rrt_marker.markers[0].points.size() > 0)
                rrt_planner_pub.publish(rrt_marker);

            path_planner->resetImmidiatePath();

            if(path_planner->isSolved())
                path_planner->reset();
        }

        if(current_time > ros::Duration(5).toSec())
        {
            visualization_msgs::MarkerArray sol_traj_marker;
            path_planner->getTrajectoryMarkers(sol_traj_marker);

            current_traj_pub.publish(sol_traj_marker);

            Eigen::Vector3f pc = path_planner->getFirstTrajPoint();

            geometry_msgs::Point pp;
            pp.x = pc[0];
            pp.y = pc[1];
            pp.z = pc[2];
            trajectory_pub.publish(pp);

            path_planner->addLastControlPoint();
        }

        ros::spinOnce();

    }

    ROS_INFO_STREAM("Finished ");

    return 0;
}
