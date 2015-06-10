#!/bin/bash
# Author: Jay Mills 19/02/2015
# Edited: Jeremie Bannwarth 28/05/2015
# Install ROS environment, as well as the quadcopter control program and dependencies
# Place in folder you want the control software to be placed

### Exit bash script if something fails
set -e

### Install ROS
echo "installing ROS Indigo"

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-indigo-desktop-full

sudo rosdep init
rosdep update

echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get install python-rosinstall

### Download quadcopter control folder from github
sudo apt-get install git
git clone https://github.com/JBannwarth/TrackingQuadControl.git

cd TrackingQuadControl
cd src

sudo rm -rf geometry
sudo rm -rf roscopter
sudo rm -rf ros_vrpn_client

# Get customised packages from onedrive
wget https://onedrive.live.com/download?resid=8f8e9b2f733222e5%2113522 -O geometry.tar.gz
wget https://onedrive.live.com/download?resid=8f8e9b2f733222e5%2113525 -O roscopter.tar.gz
wget https://onedrive.live.com/download?resid=8f8e9b2f733222e5%2113526 -O ros_vrpn_client.tar.gz

tar -zxvf geometry.tar.gz
tar -zxvf roscopter.tar.gz
tar -zxvf ros_vrpn_client.tar.gz

rm geometry.tar.gz
rm -zxvf roscopter.tar.gz
rm ros_vrpn_client.tar.gz

# Install pymavlink
cd roscopter/mavlink/pymavlink
python setup.py install --user

echo "export PYTHONPATH="$HOME/.local/lib/python2.6/site-packages/:$PYTHONPATH"" >> ~/.bashrc
echo "export PATH="$HOME/.local/lib/python2.6/bin/:$PATH"" >> ~/.bashrc
source ~/.bashrc

cd ../../..

### Install dependencies
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-hydro-sensor-msgs python-serial python-tz

export ROS_PACKAGE_PATH=$(pwd)/roscopter:$ROS_PACKAGE_PATH
rosdep update
rosdep install roscopter

### Build quadcopter control software
cd ..
source /opt/ros/indigo/setup.bash
catkin_make
source devel/setup.bash

### Download and install UI design environment
sudo apt-get install pyqt4-dev-tools qt4-dev-tools qt4-designer
