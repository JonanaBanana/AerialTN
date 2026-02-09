#!/bin/bash
source venv-orbslam3/bin/activate # uncomment if not using virtual environment
## install OpenCV with OpenCV_contrib
cd ~
mkdir thirdparty
cd ~/thirdparty/
git clone https://github.com/opencv/opencv
git -C opencv checkout 4.12.0
git clone https://github.com/opencv/opencv_contrib
git -C opencv_contrib checkout 4.12.0
mkdir build
cd build
cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules ../opencv
make -j $(nproc)
sudo make install -j $(nproc)