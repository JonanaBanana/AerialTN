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

sudo apt install python3-venv -y
wait

python3 -m venv venv-tvnav
wait

pip3 install setuptools wheel catkin_pkg
pip3 install opencv-python
pip3 install opencv-contrib-python

sudo apt-get install build-essential

sudo apt-get install libgflags-dev -y
sudo apt install libgoogle-glog-dev -y
sudo apt-get install protobuf-compiler libprotobuf-dev -y

sudo apt install libeigen3-dev -y

sudo apt install cmake -y
sudo apt-get install ament-cmake -y


sudo apt-get install libgtk-3-dev \
                     libgtk-2-dev -y


sudo apt-get install libavcodec-dev \
                     libavformat-dev \
                     libavutil-dev \
                     libswscale-dev\
                     libavdevice-dev -y


sudo apt-get install libgstreamer1.0-dev \
                libgstreamer-plugins-base1.0-dev \
                libgstreamer-plugins-bad1.0-dev \
                gstreamer1.0-plugins-base \
                gstreamer1.0-plugins-good \
                gstreamer1.0-plugins-bad \
                gstreamer1.0-plugins-ugly \
                gstreamer1.0-libav \
                gstreamer1.0-tools \
                gstreamer1.0-x \
                gstreamer1.0-alsa \
                gstreamer1.0-gl \
                gstreamer1.0-gtk3 \
                gstreamer1.0-qt5 \
                gstreamer1.0-pulseaudio -y

sudo apt-get install libdc1394-2-dev -y

sudo apt-get install libblas-dev liblapack-dev -y

sudo apt install libogre-1.12-dev -y

sudo apt-get install libatlas-base-dev libsuitesparse-dev -y

sudo apt-get install freeglut3-dev libglew-dev -y

sudo apt-get install libmetis-dev -y

sudo apt install unzip -y

sudo apt install ros-humble-vision-opencv -y

sudo apt install ros-humble-message-filters -y



mkdir thirdparty
wait


#install ceres solver
cd ~/thirdparty/
wait

wget http://ceres-solver.org/ceres-solver-2.2.0.tar.gz
wait

tar zxf ceres-solver-2.2.0.tar.gz
wait

cd ceres-solver-2.2.0
wait

mkdir build
wait

cd build
wait

cmake -DEXPORT_BUILD_DIR=ON \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        ../
wait

make -j $(nproc)
wait

make test -j $(nproc)
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

## install OpenCV with OpenCV_contrib
cd ~/thirdparty/
wait

git clone https://github.com/opencv/opencv
wait

git -C opencv checkout 4.13.0
wait

git clone https://github.com/opencv/opencv_contrib
wait

git -C opencv_contrib checkout 4.13.0
wait

cd opencv/
wait

mkdir build
wait

cd build
wait

cmake -D CMAKE_BUILD_TYPE=RELEASE \
          -D CMAKE_INSTALL_PREFIX=/usr/local \
          -D INSTALL_C_EXAMPLES=OFF \
          -D INSTALL_PYTHON_EXAMPLES=OFF \
          -D ENABLE_FAST_MATH=ON \
          -D BUILD_opencv_java=OFF \
          -D BUILD_ZLIB=ON \
          -D BUILD_TIFF=ON \
          -D WITH_GTK=ON \
          -D WITH_FFMPEG=ON \
          -D WITH_1394=ON \
          -D OPENCV_GENERATE_PKGCONFIG=ON \
          -D OPENCV_PC_FILE_NAME=opencv4.pc \
          -D OPENCV_ENABLE_NONFREE=ON \
          -D WITH_GSTREAMER=ON \
          -D WITH_V4L=ON \
          -D WITH_QT=ON \
          -D WITH_OPENGL=ON \
          -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
          -D BUILD_EXAMPLES=ON ..
wait

make -j $(nproc)

wait

sudo make install -j $(nproc)

cd ~