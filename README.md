# rgbd_local_map
This is part of my capstone project. A local map for UAV based system

run command 
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_ROS_INTERFACE=ON

if px4/sitl_gazebo/realsense_ros empty
try
git submodule update --init --recursive

or
 git rm src/px4
 git rm src/sitl_gazebo
 git rm src/realsense-ros

git submodule add --recursive  https://github.com/PX4/sitl_gazebo.git src/sitl_gazebo 
git submodule add --recursive https://github.com/PX4/Firmware.git src/px4
git submodule add --recursive https://github.com/IntelRealSense/realsense-ros.git src/relsense-ros

git submodule update --init --recursive
