#!/bin/sh

#update apt
sudo apt update

#install python3 peripherals
echo "INSTALL PYTHON3 PERIPHERALS"
sudo apt install -y python3 python3-dev python3-pip build-essential python-jinja2 python3-jinja python-empy python-toml python-numpy python-yaml python3-empy python3-toml python3-numpy python3-yaml

sudo apt-get install libprotobuf-dev libprotoc-dev protobuf-compiler libeigen3-dev libxml2-utils

#install ros python3 peripherals
echo "INSTALLING ROS PYTHON3 PERIPHERALS"
sudo -H pip3 install rosdep rospkg rosinstall_generator rosinstall wstool vcstools catkin_tools  catkin_pkg jinja2

#remove old rosdep
echo"--- REMOVING OLD ROSDEP ---"
sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
sudo rosdep init
rosdep update

#add melodic library incase of error missing library
echo "--- ADD MELODIC LIBRARY DIR TO BASHRC ---"
echo 'export LD_LIBRARY_PATH=/opt/ros/melodic/lib/:$LD_LIBRARY_PATH' >> ~/.bashrc 
source ~/.bashrc

echo "installing mavros packages"
sudo apt install ros-melodic-mavros ros-melodic-mavros-extras ros-melodic-mavros-msgs ros-melodic-ddynamic-reconfigure

#installing intel realsense sdk
echo "--- INSTALLING Intel RealSense SDK ---"
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
sudo apt update
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev

#installing gstreamer
echo "--- INSTALLING GSTREAMER---"
sudo apt install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio

echo "--- INSTALLING GEOGRAPHICS LIB IN HOME"
cd ~
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x ./install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh 
rm install_geographiclib_datasets.sh

#installing FastRTPS https://dev.px4.io/v1.9.0/en/setup/dev_env_linux_ubuntu.html
echo "installing FastRTPS on home/libraries"
DIR="/home/$USER/libraries/"
if [ -d "$DIR" ]; then
  echo "Dir Libraries Found"
  cd libraries
else
  echo "Dir Libraries Not Found"
  mkdir libraries && cd libraries
fi

wget https://www.eprosima.com/index.php/component/ars/repository/eprosima-fast-rtps/eprosima-fast-rtps-1-7-1/eprosima_fastrtps-1-7-1-linux-tar-gz -O eprosima_fastrtps-1-7-1-linux.tar.gz
tar -xzf eprosima_fastrtps-1-7-1-linux.tar.gz eProsima_FastRTPS-1.7.1-Linux/
tar -xzf eprosima_fastrtps-1-7-1-linux.tar.gz requiredcomponents
tar -xzf requiredcomponents/eProsima_FastCDR-1.0.8-Linux.tar.gz

cd eProsima_FastCDR-1.0.8-Linux && ./configure --libdir=/usr/lib && make -j2 && sudo make install
cd ../eProsima_FastRTPS-1.7.1-Linux && ./configure --libdir=/usr/lib && make -j2 && sudo make install
rm -rf requiredcomponents eprosima_fastrtps-1-7-1-linux.tar.gz

#building catkin with python3 interpreter
echo "If you want to build catkin workspace with ptyhon3 interpreter use command:"
echo "catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3"

echo "For this repo, enable SITL ROS Interface"
echo "catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_ROS_INTERFACE=ON"


