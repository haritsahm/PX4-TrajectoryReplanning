#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/conversions.h>

#include <ewok/uniform_bspline_3d.h>
#include <px4_trajectory_replanning/common.h>
#include <boost/chrono.hpp>
#include <boost/bind.hpp>
#include <eigen3/Eigen/Eigen>
#include <stdio.h>
#include <algorithm>
#include <math.h>

using namespace ewok;
using namespace tf;

class LeePositionController
{
public:
    LeePositionController(ros::NodeHandle &nh):
        nh_(nh)
    {
        initializeParam();


        cmd_poly_sub_ = nh.subscribe(
            "command/point", 10,
            &LeePositionController::PointCallback, this);


        odometry_sub_ = nh.subscribe("/mavros/local_position/odom", 1,
                                     &LeePositionController::OdometryCallback, this);

        ros::NodeHandle pnh("~");
        pnh.param("dt", dt, 0.5);

        b_spline_.reset(new ewok::UniformBSpline3D<6, double>(dt));

    }

    void initializeParam()
    {

    }

    void PointCallback(const geometry_msgs::PointConstPtr & point_msg)
    {
        ROS_INFO("PointCallback: %d points in spline", b_spline_->size());

        Eigen::Vector3d p(point_msg->x, point_msg->y, point_msg->z);

        if(b_spline_->size() != 0) b_spline_->push_back(p);
    }

    void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
    {
        odom_= *odometry_msg;
        eigenOdometryFromMsg(odometry_msg, &odometry);

    }

    void spin()
    {
        ros::Rate rate(20);

        double time_elapsed = 0;

        while(ros::ok())
        {
            if(b_spline_->size() == 0) {
              for(int i=0; i<6; i++) b_spline_->push_back(odometry.position);
              init_time = ros::Time::now();
              last_yaw = mav_msgs::yawFromQuaternion(odometry.orientation);
            }

            if(time_elapsed > b_spline_->maxValidTime()) {
              b_spline_->push_back(b_spline_->getControlPoint(b_spline_->size()-1));
              ROS_WARN("Adding last point once again!");
            }



            ros::spinOnce();
            rate.sleep();
        }
    }

private:

    std::string namespace_;

    //subscribers
    ros::Subscriber cmd_poly_sub_;
    ros::Subscriber odometry_sub_;

    ros::NodeHandle nh_;

    double dt;

    ewok::UniformBSpline3D<6, double>::Ptr b_spline_;
    ros::Time init_time;
    double last_yaw;
    bool arrived;

    nav_msgs::Odometry odom_;
    Eigen::Affine3d pose_;
    EigenOdometry odometry;

//    void getTrajectoryPoint(double t,
//              mav_msgs::EigenTrajectoryPoint& command_trajectory, bool & yaw_from_traj);


};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_position_controller_node");
    ros::NodeHandle nh;

    ROS_INFO("Hello world!");
}
