#! /bin/bash
# Script to set up simulation environment
# by installing ROS2, MicroROS, Gazebo, and ArduPilot
# Author: Toan Do
# Created: December 5, 2025
# Updated: December 5, 2025
# Version: 1.0

# Software versions
# Ubuntu: 22.04 LTS
# ROS2: Humble Hawksbill
# Gazebo: Harmonic
# ArduPilot: 4.6

sudo apt update && sudo apt upgrade -y
sudo apt install git curl lsb-release gnupg -y

# Install ROS2 Humble Hawksbill
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update && sudo apt upgrade -y

sudo apt install ros-humble-desktop -y

source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Install MicroROS

# Create a workspace and download the micro-ROS tools
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Install pip
sudo apt-get install python3-pip

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash
echo source ~/microros_ws/install/local_setup.bash >> ~/.bashrc

# # Create firmware step
# ros2 run micro_ros_setup create_firmware_ws.sh host

# # Build step
# ros2 run micro_ros_setup build_firmware.sh
cd ~/

# Install ArduPilot
git clone --recurse-submodules https://github.com/your-github-userid/ardupilot
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

# run ./waf configure
cd ~/

# Install ROS2-ArduPilot bridge
mkdir -p ~/ardu_ws/src
cd ~/ardu_ws
vcs import --recursive --input  https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src

cd ~/ardu_ws
sudo apt update
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# Installing the MicroXRCEDDSGen build dependency:
sudo apt install default-jre
cd ~/ardu_ws
git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
cd Micro-XRCE-DDS-Gen
./gradlew assemble
echo "export PATH=\$PATH:$PWD/scripts" >> ~/.bashrc

cd ~/ardu_ws
colcon build --packages-up-to ardupilot_dds_tests

# Build ArduPilot SITL ROS2 package
cd ~/ardu_ws/
colcon build --packages-up-to ardupilot_sitl

# Install Gazebo Harmonic
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic -y

# Installing ArduPilot to Gazebo bridge
cd ~/ardu_ws
vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src

export GZ_VERSION=harmonic

sudo apt install wget
wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update

sudo wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list
rosdep update

cd ~/ardu_ws
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -y


# Build Test
cd ~/ardu_ws
colcon build --packages-up-to ardupilot_gz_bringup

echo "Setup completed! Please run 'ros2 launch ardupilot_gz_bringup iris_runway.launch.py' to start the simulation."