#!/bin/bash

# If you are using WSL this is for installing python packages
## Setup virtual environment for python

sudo apt install python3-pip
sudo apt install python3-venv
python3 -m venv venv-orbslam3
source venv-orbslam3/bin/activate
pip install setuptools wheel catkin_pkg