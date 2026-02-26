#!/bin/bash

#this shell file is an installer for all dependencies of the Thermal Vision Navigation package and research environment. 
#It is intended to be run a clean installation of Ubuntu 22.04 LTS, and will install Ceres Solver 2.2.0, Pangolin, OpenCV, and Kindr, as well as all necessary dependencies for these packages.
#This will allow for easy installation and benchmarking of SOTA visual inertial navigation algorithms such as VINS-Multi, ORB-SLAM3 and ROVTIO in a ROS2 environment.

cd ~
wait

sudo apt update
wait

sudo apt upgrade -y
wait

sudo apt install python3-pip -y
wait

pip3 install setuptools wheel catkin_pkg

sudo apt-get install build-essential

sudo apt-get install libgflags-dev -y

sudo apt install libeigen3-dev -y
sudo apt-get install ament-cmake -y

sudo apt-get install libgtk2.0-dev -y

sudo apt-get install libgtk-3-dev -y

sudo apt-get install libavcodec-dev \
                     libavformat-dev \
                     libavutil-dev \
                     libswscale-dev\
                     libavdevice-dev -y

sudo apt-get install libblas-dev liblapack-dev -y

sudo apt-get install libatlas-base-dev libsuitesparse-dev -y

sudo apt-get install freeglut3-dev libglew-dev -y

sudo apt-get install libmetis-dev -y

sudo apt install unzip -y

mkdir thirdparty
wait


#install ceres solver
cd ~/thirdparty/
wait

git clone --recursive https://github.com/ceres-solver/ceres-solver.git
wait

cd ceres-solver
wait

mkdir build
wait

cd build
wait

cmake ..

wait

make -j $(nproc)
wait

make test
wait

sudo make install -j $(nproc)
wait

#install kindr
cd ~/thirdparty

git clone https://github.com/ethz-asl/kindr.git

cd kindr/

mkdir build

cd build

cmake ..

sudo make install -j $(nproc)

# install pangolin
cd ~/thirdparty/

git clone --recursive https://github.com/stevenlovegrove/Pangolin.git

cd Pangolin/

sudo ./scripts/install_prerequisites.sh all

cmake -B build

cmake --build build

cd build

make -j8

sudo make install

cd ~

sudo ldconfig