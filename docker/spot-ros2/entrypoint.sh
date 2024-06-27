#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
source /home/robot/spot_ros2_ws/install/setup.bash
export $SPOT_NAME="hkaspot"
ros2 launch spot_driver spot_driver.launch.py config_file:=./src/spot_ros2/configs/spot_ros_config.yaml spot_name:=$SPOT_NAME publish_point_clouds:=True
