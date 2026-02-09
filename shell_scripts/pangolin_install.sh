#!/bin/bash

## install pangolin
cd ~
mkdir thirdparty
cd thirdparty/
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin/
sudo ./scripts/install_prerequisites.sh recommended
cmake -B build
cmake --build build
cd build
make -j16
sudo make install
