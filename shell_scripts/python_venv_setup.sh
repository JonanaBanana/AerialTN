#!/bin/bash

# If you are using WSL this is for installing python packages
## Setup virtual environment for python

sudo apt install python3-pip
wait

sudo apt install python3-venv
wait

python3 -m venv venv-orbslam3
wait

source venv-orbslam3/bin/activate
wait

pip install setuptools wheel catkin_pkg