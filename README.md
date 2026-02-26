# AerialTN

Thermal Vision Navigation using Remote Sensing in GPS Denied Environments.

This research projects aims to implement remote sensing thermal vision navigation in gps denied environments using drones.

This package is developed for Ubuntu 22.04 with ROS2 Humble. For any other versions of Ubuntu and ROS2, note that these shell scripts will need to be modified accordingly and might not work at all.

## Installation

To allow execution of the shell scripts run:

```bash
# Enable execution of the shell scripts
cd /path/to/ros2_ws/src/AerialTN/shell_scripts
chmod +x shebang.sh ros2_humble_install.sh orbslam3_install.sh vins_multi_install.sh rovtio_ros2_install.sh
```

On a clean system run the scripts in the following order:

First, setup your installation of ros2 humble (for Ubuntu 22.04). Run _ros2_humble_install.sh_. This also configures the environment and sets up automatic sourcing of ros2 humble in each terminal.

```bash
# Install and configure ROS2
cd /path/to/ros2_ws/src/AerialTN/shell_scripts
./ros2_humble_install.sh
```

If you already have ros2 humble installed, check the script for any package installations that you might not have installed. These should be the important ones:

```bash
#ROS2 Humble dependencies that will be needed.
sudo apt install python3-colcon-common-extensions
sudo apt-get install ros-humble-rviz2 ros-humble-turtle-tf2-py ros-humble-tf2-ros ros-humble-tf2-tools
sudo apt-get install ros-humble-usb-cam
sudo apt install ros-humble-vision-opencv
sudo apt install ros-humble-message-filters
sudo apt-get install ros-humble-image-view
```

Second, run _shebang.sh_. This script will install all package dependencies and install thirdparty packages used by ORBSLAM3, ROVTIO and VINS-MULTI. Thirdparty packages will be installed in the directory ~/thirdparty.
We assume that there is a system installation of OpenCV 4.5.4 which is always installed alongside ROS2 Humble. Do not build OpenCV from source unless absolutely necessary, as it can cause issues with the pre-built opencv from ros2 humble. This can cause obscure errors in ORBSLAM3s ros2 wrapper and other ros2 packages with opencv dependencies. If this package is installed on an Ubuntu system with no GUI, you can probably save some space by not installing certain GUI specific libraries. But modify this at your own discretion.

```bash
# Install dependencies and build thirdparty packages from source in ~/thirdparty folder
cd /path/to/ros2_ws/src/AerialTN/shell_scripts
./shebang.sh
```

Now you should be able to install the remaining packages with their respective shell scripts. First, create a ros2 workspace. The script assumes that the ros2 packages will be cloned into the directory ~/ros2_ws/src and their core libraries will either be installed inside the ros2 package or in the directory ~/thirdparty.

```bash
# Install ORBSLAM3 and its ros2_wrapper
cd /path/to/ros2_ws/src/AerialTN/shell_scripts
./orbslam3_install.sh
```

```bash
# Install VINS-Multi and its ros2_wrapper
cd /path/to/ros2_ws/src/AerialTN/shell_scripts
./vins_multi_install.sh
```

```bash
# ROVTIO IS NOT FUNCTIONAL FOR ROS2 YET, DO NOT INSTALL
# Install ROVTIO and its ros2_wrapper
cd /path/to/ros2_ws/src/AerialTN/shell_scripts
./rovtio_ros2_install.sh
```

## Testing

### ORBSLAM3

To test ORBSLAM3's core module you can download the euroc machine hall dataset and extract the MH_01_easy dataset. You will need to unzip the zip file inside the folder, since it will be read by ORBSLAM3. Assuming you placed the MH_01_easy dataset in the directory ~/datasets/MH_01_easy you can run an ORBSLAM3 example with the command line:

```bash
# Test ORBSLAM3 core (without ros2 wrapping)
cd ~/thirdparty/ORBSLAM3/Examples/
./Monocular/mono_euroc ../Vocabulary/ORBvoc.txt ./Monocular/EuRoC.yaml ~/datasets/MH_01_easy ./Monocular/EuRoC_TimeStamps/MH01.txt
```

If you do now want to download a dataset to test ORBSLAM3, you can simply test if the ros2 wrapper can launch a node and wait for data. Source your workspace and run the monocular node.

```bash
# Test ORBSLAM3 ros2 wrapper
cd ~/ros2_ws/
source install/setup.bash
ros2 run orbslam3 mono ~/ros2_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt ~/ros2_ws/src/orbslam3_ros2/config/monocular/TUM1.yaml
```

If the node opens two viewports and does not crash, the installation should be working.

### VINS-Multi

To test VINS-Multi simply source the workspace and run the sample launch file.

```bash
# Test VINS-Multi ros2 wrapper
cd ~/ros2_ws/
source install/setup.bash
ros2 launch vins_multi_ros2 vins_estimator.launch.py
```

If the terminal hangs at the line _vins_multi_ros2 initialized, waiting for data..._ then the package should be working
