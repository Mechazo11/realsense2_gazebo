![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)
![License](https://img.shields.io/badge/License-Apache%202.0-green)
#  Realsense2 Simulated RGBD Camera for Gazebo Harmonic or later

**TODO** descritpion

## Package description

* realsense2_gazebo: Metapackage
* realsense2_description: URDF, mesh and rviz files 
* realsense2_gz: Launch files to bringup camera in gazebo

## Features

* RGB image -- **TODO**. Defaults to 640 x 480 with 
* Depth image -- **TODO**
* Pointcloud -- **NOT SUPPORTED**


## Installation

**TODO**

## Usage

**TODO**


## Useful notes

* Comparison of various Intel realsense cameras: https://www.intelrealsense.com/compare-depth-cameras/
* Official example: https://github.com/gazebosim/docs/blob/master/harmonic/tutorials/sensors/sensor_tutorial.sdf
* Official documentations of supported sensors: https://gazebosim.org/docs/latest/sensors/
* An excellent example of using RGBD, depth and lidar camera is here: https://github.com/MOGI-ROS/Week-5-6-Gazebo-sensors/blob/main/bme_gazebo_sensors/urdf/mogi_bot.gazebo

* This may be an easier way to automatically add GZ_SIM_RESOURCE_PATH

```python
gazebo_models_path, ignore_last_dir = os.path.split(pkg_bme_gazebo_sensors)
os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path
```