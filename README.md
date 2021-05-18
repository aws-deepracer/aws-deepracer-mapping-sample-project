# Mapping with ROS Noetic on Ubuntu 20.04

<p align="center">
<img src="/media/deepracer_circle_sticker.png" width="250" height="250" >
</p>

## Overview

The AWS DeepRacer Mapping sample project provides instructions for using the AWS DeepRacer and Intel [RealSense™ D435](https://www.intelrealsense.com/depth-camera-d435/), along with other open-source tools, to build a map of your surroundings using SLAM (Simultaneous Localization and Mapping).

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Setup
Follow these instructions to set up the AWS DeepRacer Mapping sample project. 

### Prerequisites

The AWS DeepRacer device comes with all the prerequisite packages and libraries installed for running the AWS DeepRacer core application. For more information about the pre-installed set of packages and libraries on the AWS DeepRacer and about installing the required build systems, see [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md). The Mapping sample project requires the AWS DeepRacer application to be installed on the device as it leverages the manual drive feature to control the car manually.

#### Intel [RealSense™ D435](https://www.intelrealsense.com/depth-camera-d435/) / [D435i](https://www.intelrealsense.com/depth-camera-d435i/)

Connect and mount the Intel RealSense D435 or Intel RealSense D435i to the AWS DeepRacer device. The Intel RealSense cameras provide a depth sensor and other required capabilities to collect information from your surroundings.

#### [ROS Noetic](http://wiki.ros.org/noetic) on the AWS DeepRacer device

Open a terminal and install [ROS Noetic](http://wiki.ros.org/noetic) as a root user by following the steps on the [Ubuntu install of ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) page.

#### Download and install other components

Open a terminal and run the following commands as the root user on the AWS DeepRacer device:

- **Set up the ROS Noetic environment**: After installing ROS Noetic on your AWS DeepRacer, source the setup script for ROS Noetic:

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
Follow these steps to use the AWS DeepRacer Mapping sample project. 

- Open a terminal and run the following commands as the root user on the AWS DeepRacer device:

        source /opt/ros/noetic/setup.bash

- Initiate the open-source tracking launch script provided by Intel RealSense:

        roslaunch realsense2_camera opensource_tracking.launch

- Open another terminal and launch `rviz` by running the following command as the root user:

        source /opt/ros/noetic/setup.bash
        rosrun rviz rviz

- Personalize, collect, and visualize the point cloud data by following [these steps](https://github.com/IntelRealSense/realsense-ros/wiki/SLAM-with-D435i#personalize-rviz).

- Move the AWS DeepRacer car slowly around the room in `manual` mode using the device console.

### Sample demo:

The following example shows using `rviz` when mapping the room by navigating the AWS DeepRacer car:

![mapping](/media/mapping.gif)


## Resources

* [Getting started with AWS DeepRacer OpenSource](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md)
* [Installing ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
