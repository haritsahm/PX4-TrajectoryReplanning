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


        position_sub_ = nh.subscribe("/mavros/local_position/pose", 1,
                                     &LeePositionController::mavPosCallback, this);

        ros::NodeHandle pnh("~");
        pnh.param("dt", dt, 0.5);

        b_spline_.reset(new ewok::UniformBSpline3D<6, double>(dt));

    }

    void initializeParam()
    {

    }

    inline Eigen::Vector3d toEigen(const geometry_msgs::Point& p) {
      Eigen::Vector3d ev3(p.x, p.y, p.z);
      return ev3;
    }

    void PointCallback(const geometry_msgs::PointConstPtr & point_msg)
    {
        ROS_INFO("PointCallback: %d points in spline", b_spline_->size());

        Eigen::Vector3d p(point_msg->x, point_msg->y, point_msg->z);

        if(b_spline_->size() != 0) b_spline_->push_back(p);
    }

    void mavPosCallback(const geometry_msgs::PoseStamped& msg)
    {
      mavPos_ = toEigen(msg.pose.position);
      mavAtt_.w() = msg.pose.orientation.w;
      mavAtt_.x() = msg.pose.orientation.x;
      mavAtt_.y() = msg.pose.orientation.y;
      mavAtt_.z() = msg.pose.orientation.z;

    }

    void getTrajectoryPoint(double t,
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

    void spin()
    {
        ros::Rate rate(20);

        double time_elapsed = 0;

        while(ros::ok())
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

            ros::spinOnce();
            rate.sleep();
        }
    }

private:

    std::string namespace_;

    //subscribers
    ros::Subscriber cmd_poly_sub_;
    ros::Subscriber position_sub_;

    ros::NodeHandle nh_;

    double dt;

    ewok::UniformBSpline3D<6, double>::Ptr b_spline_;
    ros::Time init_time;
    double last_yaw;
    bool arrived;

    nav_msgs::Odometry odom_;
    Eigen::Vector3d mavPos_, mavVel_, mavRate_;
    Eigen::Quaterniond mavAtt_;

    Eigen::Affine3d pose_;
    EigenOdometry odometry;
    Eigen::Vector3f offset_;

//    void getTrajectoryPoint(double t,
//              mav_msgs::EigenTrajectoryPoint& command_trajectory, bool & yaw_from_traj);


};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_position_controller_node");
    ros::NodeHandle nh;

    ROS_INFO("Hello world!");
}
