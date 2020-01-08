# RGBD Local Mapping UAV SITL

This repo is for exploring PX4 SITL & HITL Mode and Circular Buffer Mapping using Octomap 

---
## 1. Download Repository and all of its submodules

Pull all of the repo and its submodule using 
```bash=
git pull --branch melodic-dev --recurse-submodules https://github.com/haritsahm/rgbd_local_map.git
```
check px4, sitl_gazebo, and realsense-ros, if its empty then
```bash=
git submodule update --init --recursive
```
or
```bash=
git rm src/px4
git rm src/sitl_gazebo
git rm src/realsense-ros

git submodule add --recursive  https://github.com/PX4/sitl_gazebo.git src/sitl_gazebo 
git submodule add --recursive https://github.com/PX4/Firmware.git src/px4
git submodule add --recursive https://github.com/IntelRealSense/realsense-ros.git src/relsense-ros
```

## 2. Installation Python3 Interpreter and PX4 
Install the Python3 Interpreter and all of PX4 and sitl_gazebo requirements such as mavros, FastRTPS, geographiclib, etc.
```bash=
chmod +x setup_python3_ros.sh
./setup python3_ros.sh
```

## 3. Build workspace using python3
Build workspace using python3 interpreter
```bash=
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_ROS_INTERFACE=ON
```
