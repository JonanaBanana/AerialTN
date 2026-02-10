#!/bin/bash

## install OpenCV with OpenCV_contrib
cd ~
wait

mkdir thirdparty
wait

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