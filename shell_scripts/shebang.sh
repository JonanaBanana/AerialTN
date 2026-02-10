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

#cd build

#make -j16

#sudo make install

#sudo ldconfig

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
wait