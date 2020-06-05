# ROS Wrapper for FRAMOS D400e camera series

This readme file provides instructions on how to use the D400e camera series with ROS.

## Supported platforms

Ubuntu 16.04 x86_64

Ubuntu 18.04 x86_64

Ubuntu 18.04 ARM64

## Prerequisites

FRAMOS CameraSuite version 4.1.2.0 or higher

Intel® RealSense™ SDK with support for D400e camera series version 2.29.5 or higher

## Notes

These instructions are based on the instructions available at the [ROS Wrapper for Intel® RealSense™ Devices GitHub repository](https://github.com/IntelRealSense/realsense-ros).

## Installation

### Prerequisites

Make sure that `main`, `universe`, `restricted` and `multiverse` repositories are added

```
sudo add-apt-repository main
sudo add-apt-repository universe
sudo add-apt-repository restricted
sudo add-apt-repository multiverse
sudo apt update
```

### Install the ROS distribution

Install ROS Kinetic on Ubuntu 16 and ROS Melodic on Ubuntu 18. See the [ROS Kinetic wiki](http://wiki.ros.org/kinetic/Installation/Ubuntu) and [ROS Melodic wiki](http://wiki.ros.org/melodic/Installation/Ubuntu) for details.

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
```
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
```
sudo apt-get update
```
#### Install ROS Kinetic on Ubuntu 16:
```
sudo apt-get install ros-kinetic-desktop-full ros-kinetic-ddynamic-reconfigure -y
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
#### Install ROS Melodic on Ubuntu 18:
```
sudo apt-get install ros-melodic-desktop-full python-rosdep ros-melodic-ddynamic-reconfigure -y
```
On Jetson TX2, change the OpenCV include path before proceeding:
```
sudo sed -i 's/include\/opencv/include\/opencv4/g' /opt/ros/melodic/share/cv_bridge/cmake/cv_bridgeConfig.cmake
```
```
sudo rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Install Intel® RealSense™ ROS from Sources

```
mkdir -p ~/catkin_ws/src/realsense-ros
cd ~/catkin_ws/src/realsense-ros
cp -r /usr/src/librealsense2/wrappers/ros/. .
cd ..
catkin_init_workspace
```

```
cd ~/catkin_ws
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Usage Instructions for single camera

The launch file used in this example is `rs_camera.launch` located in `~/catkin_ws/src/realsense-ros/realsense2_camera/launch`.

Start the camera node in ROS

```
roslaunch realsense2_camera rs_camera.launch
```

Launch `rviz` in another terminal

```
rosrun rviz rviz
```

In the `rviz` GUI 
- change the `Fixed Frame` from `map` to `camera_depth_frame`
- click `Add`, select `By topic` and choose `depth/color/points/PointCloud2`

## Usage instruction for two cameras

The launch file used in this example is `framos_multiple_devices.launch` located in `~/catkin_ws/src/realsense-ros/realsense2_camera/launch`.

Enter the serial numbers or IP addresses of cameras in the `serial_no` or `ip_address` fields in the file `multicam_config_file.yaml` located in `~/catkin_ws/src/realsense-ros/realsense2_camera/config`. Serial number has precedence over IP address. If both fields are empty, the first detected camera is used.

Start the camera nodes in ROS

```
roslaunch realsense2_camera framos_multiple_devices.launch
```

Launch `rviz` in another terminal

```
rosrun rviz rviz
```

## Usage instruction for three or more cameras

The launch file used in this example is `framos_multiple_devices.launch` located in `~/catkin_ws/src/realsense-ros/realsense2_camera/launch`.

Create a new `cameraN` entry for each additional camera in the file `multicam_config_file.yaml` located in `~/catkin_ws/src/realsense-ros/realsense2_camera/config`. Make sure that the prefixes of properties match the camera name. Use the existing `camera1` and `camera2` entries as a reference.

Start the camera nodes in ROS

```
roslaunch realsense2_camera framos_multiple_devices.launch
```

Launch `rviz` in another terminal

```
rosrun rviz rviz
```
