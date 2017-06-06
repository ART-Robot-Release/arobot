#!/bin/bash

# Varibles
rosversion="kinetic"
# Install the ros

if [ `id -u` == 0 ]; then
	echo "Don't running this use root(sudo)."
	exit 0
fi

echo "Start to install the ros, http://wiki.ros.org/$rosversion/Installation/Ubuntu"
echo "Update the software list"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get update

echo "Install the ros from apt" 
sudo apt-get install ros-$rosversion-desktop-full -y
sudo rosdep init
rosdep update

echo "Setup the ROS environment variables"
echo -e "if [ -f /opt/ros/kinetic/setup.bash ]; then\n\tsource /opt/ros/kinetic/setup.bash\nfi" >> ~/.bashrc
source ~/.bashrc

echo "Install the rosinstall"
sudo apt-get install python-rosinstall -y

# Install the dependecies for the project 
echo "Start to config for the project"

echo "Install the python dependecies"
sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas python-sympy python-nose -y

echo "Install the eigen3"
sudo apt install libeigen3-dev -y

echo "Install the nlopt"
sudo apt install libnlopt* -y

echo "Install the controller for ros"
sudo apt install ros-kinetic-controller-manager -y
sudo apt install ros-kinetic-rqt-controller-manager -y
sudo apt install ros-kinetic-effort-controllers -y
sudo apt install ros-kinetic-position-controllers -y
sudo apt install ros-kinetic-velocity-controllers -y
sudo apt install ros-kinetic-joint-state-controller -y
sudo apt install ros-kinetic-transmission-interface -y
sudo apt install ros-kinetic-joint-trajectory-controller -y
sudo apt install ros-kinetic-joint-trajectory-controller -y
sudo apt install ros-kinetic-joint-limits-interface -y

echo "Install the controller for gazebo"
# sudo apt install ros-kinetic-gazebo-ros-pkgs
# sudo apt install ros-kinetic-gazebo-ros-control
cd src/gazebo_ros_pkgs
rosdep install --from-paths . -i -y

echo "Install the redis"
sudo apt install redis-* -y
sudo apt install libhiredis-dev -y

echo "Copy the Media file to .gazebo/Media"
cp -r ./src/robot_gazebo/Media/models/slope ~/.gazebo/models/


echo "--Finish"
