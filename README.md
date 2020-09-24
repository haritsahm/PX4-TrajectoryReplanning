# Trajectory Replanning Using RRT* and Circular Buffer

This repo is for exploring PX4 SITL & HITL Mode and UAV trajectory replanning using RRT* in [Ewok-Replanning RRT*](https://github.com/haritsahm/ewok-replanning-rrt)

---

## 1. Download Repository and all of its submodules

Pull all of the repo and its submodule using 
```bash=
git clone --branch melodic-dev --recurse-submodules https://github.com/haritsahm/PX4-TrajectoryReplanning.git
```
check if these folder is inside src: **ewok**, **mavros_controllers**, **px4**, and **px4_trajectory_replanning**, **px4_simulation**

After that run this command to download the submodules
```bash=
git submodule update --init --recursive
```

After everything is finished, we need to create a symlink from src to the mavlink_sitl_gazebo inside px4/Tools if its not available.
```bash=
cd src/ # move to src directory
ln -s px4/Tools/sitl_gazebo/ mavlink_sitl_gazebo
```

---
## 2. Build workspace using python3
Because several packages generate the same target directory, we use **catkin build** to avoid these errors. Suggested by [Effective Robotics Programming with ROS - Third Edition#1](https://github.com/rosbook/effective_robotics_programming_with_ros/issues/1#issuecomment-299646511)

#### A. From terminal on workspace directory
Build workspace using python3 interpreter from terminal
```bash=
catkin build --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_ROS_INTERFACE=ON --make-args -j4
```
#### B. Using Qt Creator
Similar to [Setup Qt Creator for ROS by Levi Armstrong](https://github.com/Levi-Armstrong/ros_qtc_plugins/wiki/Setup-Qt-Creator-for-ROS#16-setup-build-settings-for-catkin-workspace)

1. First open the **project**, then navigate to the **Projects** > **Build Settings**
2. Remove existing "**Build Steps**".
3. Add a custom build step **Add Build Step** > **Custom Process Step** as shown below.
    * **Command**: /usr/local/bin/catkin
    * **Arguments**: build --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_ROS_INTERFACE=ON --make-args -j2
    * **Working directory**: %{CurrentProject:Path}
4. Remove existing "**Clean Steps**".
5. Add a custom clean step **Add Clean Step** > **Custom Process Step** as shown below.
    * **Command**: /usr/local/bin/catkin
    * **Arguments**: clean
    * **Working directory**: %{CurrentProject:Path}

---
## 3. Running Simulation

### A. Plain RRT Simulation
Use rviz to vizualize the simulation
```bash=
rviz -d src/ewok/ewok_simulation/rviz/simulation_rrt.rviz 
```
#### 1. run this to command to simulate in cones
```bash=
roslaunch ewok_simulation trajectory_replanning_simulation_rrt.launch 
```
[![Trajectory Replanning Using RRT* - Test Cones](http://img.youtube.com/vi/2R5ByHK022A/0.jpg)](http://www.youtube.com/watch?v=2R5ByHK022A)

#### 2. run this to command to simulate in Forest
```bash=
roslaunch ewok_simulation trajectory_replanning_forest_rrt.launch 
```
[![Trajectory Replanning Using RRT* - Test Forest](http://img.youtube.com/vi/aaczZhq_mr0/0.jpg)](http://www.youtube.com/watch?v=aaczZhq_mr0)

---
### B. Using PX4 SITL Mode
#### 1. Run the Gazebo PX4 SITL mode 
Please run this everytime you want to connect it with offboard controller or after editing the offboard controller
```bash=
roslaunch px4_trajectory_replanning px4_gazebo_cones.launch 
```
#### 2. Run Rviz 
```bash=
roslaunch px4_trajectory_replanning px4_rviz.launch 
```

#### 3. Run the Offboard Controller
This is the main offboard controller
```
roslaunch px4_trajectory_replanning px4_offboard_controller.launch 
```

#### 4. Run the Offboard Interface
```bash
roslaunch px4_trajectory_replanning px4_offboard_interface.launch 
```

![](https://i.imgur.com/VYNEfFt.png)

Wait for a few seconds until the mode status is filled. 
* Press the offboard button to change the pixhawk mode to offboard control
* wait for 3 seconds and then send any desired tasks
* Do not start the mission instruction before the replanning system is running
#### 5. Run the Replanning System
```bash
roslaunch px4_trajectory_replanning trajectory_replanning_rrt.launch 
```
##License

This project is licensed under the GNU Lesser General Public License Version 3 (LGPLv3). 
The interface was developed using Qt Designer and Qt Creator. They are licensed under GNU General Public License Version 3 (GPLv3). 
The interface is using Qt Core, Qt Gui, and Qt Widgets Framework APIs licensed udner the GNU Lesser General Public License Version 3 (LGPLv3).
Ewok is licensed under the GNU Lesser General Public License Version 3 (LGPLv3). 
Pixhawk Firmware and Middleware is licensed under BSD-3-Clause License.
For full license details, refer to the license file in the library's directory.





