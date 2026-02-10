#!/bin/bash
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

#sudo apt install libeigen3-dev -y

sudo apt install cmake -y

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


mkdir thirdparty
wait

#install eigen 3.3.9 - IT SHOULD ALREADY HAVE BEEN INSTALLED
#cd ~/thirdparty/

#wget -O eigen-3.3.9.zip https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.zip

#unzip eigen-3.3.9.zip

#cd eigen-3.3.9/

#mkdir build

#cd build

#cmake ../ && sudo make install -j $(nproc)


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



