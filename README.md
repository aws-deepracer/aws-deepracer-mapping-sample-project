# Mapping with ROS Noetic on Ubuntu 20.04

<p align="center">
<img src="/media/deepracer_circle_sticker.png" width="250" height="250" >
</p>

## Overview

The Mapping sample project provides instructions for using the AWS DeepRacer and Intel [RealSense™ D435](https://www.intelrealsense.com/depth-camera-d435/) along with other open source tools to build a map of your surrounding using SLAM (Simultaneous Localization and Mapping).

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Setup

### Prerequisites

The DeepRacer device comes with all the pre-requisite packages and libraries installed for running the DeepRacer core application. More details about pre installed set of packages and libraries on the DeepRacer, and installing required build systems can be found in the [Getting Started](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md) section of the AWS DeepRacer Opensource page. The Mapping sample project requires the AWS DeepRacer application to be installed on the device as it leverages the manual drive feature to control the car manually.

#### Intel [RealSense™ D435](https://www.intelrealsense.com/depth-camera-d435/) / [D435i](https://www.intelrealsense.com/depth-camera-d435i/)

Connect and mount the Intel RealSense D435 or Intel RealSense D435i to the AWS DeepRacer device. The Intel RealSense cameras provides depth sensor and other required capabilities to collect information from your surrounding.

#### [ROS Noetic](http://wiki.ros.org/noetic) on AWS DeepRacer device

Open up a terminal and install [ROS Noetic](http://wiki.ros.org/noetic) as a root user by following the steps here - http://wiki.ros.org/noetic/Installation/Ubuntu

#### Download and install other components

Open up a terminal and run the following commands as root user on the AWS DeepRacer device:

- **Set up ROS Noetic environment** - After installing ROS Noetic on your AWS DeepRacer, source the setup script for ROS Noetic

            source /opt/ros/noetic/setup.bash

- Install the software packages required to create the map: 

    - [*realsense2_camera*](https://github.com/IntelRealSense/realsense-ros):

            export ROS_VER=noetic
            sudo apt-get install ros-$ROS_VER-realsense2-camera
            sudo apt-get install ros-$ROS_VER-realsense2-description

    - [*imu_filter_madgwick*](https://github.com/ccny-ros-pkg/imu_tools/tree/noetic):

            sudo apt-get install ros-$ROS_VER-imu-filter-madgwick

    - [*rtabmap_ros*](https://github.com/introlab/rtabmap_ros/tree/noetic-devel):

            sudo apt-get install ros-$ROS_VER-rtabmap-ros

    - [*robot_localization*](https://github.com/cra-ros-pkg/robot_localization/tree/noetic-devel):

            sudo apt-get install ros-$ROS_VER-robot-localization

## Usage

- Open up a terminal and run the following commands as root user on the AWS DeepRacer device:

        source /opt/ros/noetic/setup.bash

- Trigger the opensource tracking launch script provided by Intel RealSense:

        roslaunch realsense2_camera opensource_tracking.launch

- Open up another terminal and launch rviz by running the following command as root user:

        source /opt/ros/noetic/setup.bash
        rosrun rviz rviz

- Personalize, collect and visualize the point cloud data by following the steps [here](https://github.com/IntelRealSense/realsense-ros/wiki/SLAM-with-D435i#personalize-rviz).

- Move the AWS DeepRacer car slowly around the room in **Manual mode** using the device console.

### Sample Demo:

RVIZ when mapping the room by navigating the AWS DeepRacer car:

![mapping](/media/mapping.gif)


## Resources

* AWS DeepRacer Opensource getting started: [https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
* Installing ROS Noetic instructions: [http://wiki.ros.org/noetic/Installation/Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu)
