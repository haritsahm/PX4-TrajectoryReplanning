#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <eigen3/Eigen/Eigen>
#include <vector>
#include <algorithm>
#include <iostream>


class TrajectoryGenerator
{
public:
    TrajectoryGenerator();

    void controlPointsCallback(const trajectory_msgs::MultiDOFJointTrajectory &msg);

    double computeBasisFunction(int i, double t, int k);
    void computeTrajectory();
    void computeReplanning();

private:
    int n_control_points;
    std::vector<Eigen::Vector3d> control_points;
    std::vector<double> time_knots;

};

#endif