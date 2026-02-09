#!/bin/bash

source venv-orbslam3/bin/activate # uncomment if not using virtual environment
wait

## install pangolin

cd ~
wait

mkdir thirdparty
wait

cd thirdparty/
wait

git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
wait

cd Pangolin/
wait

sudo ./scripts/install_prerequisites.sh recommended
wait

cmake -B build
wait

cmake --build build
wait

cd build
wait

make -j16
wait

sudo make install
wait

sudo ldconfig