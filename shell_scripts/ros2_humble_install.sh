#!/bin/bash

locale  # check for UTF-8

sudo apt update
wait

sudo apt install locales -y
wait

sudo locale-gen en_US en_US.UTF-8
wait

sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
wait

export LANG=en_US.UTF-8
wait

locale  # verify settings
wait

sudo apt install software-properties-common -y
wait

sudo add-apt-repository universe
wait

sudo apt update
wait

sudo apt install curl -y
wait

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
wait

curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
wait

sudo dpkg -i /tmp/ros2-apt-source.deb
wait

sudo apt update
wait

sudo apt upgrade -y
wait

sudo apt install ros-humble-desktop -y
wait

source /opt/ros/humble/setup.bash
wait

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
wait

sudo apt install python3-colcon-common-extensions -y
sudo apt-get install ros-humble-rviz2 ros-humble-turtle-tf2-py ros-humble-tf2-ros ros-humble-tf2-tools -y
sudo apt-get install ros-humble-usb-cam -y
wait