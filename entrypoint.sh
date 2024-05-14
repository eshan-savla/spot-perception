#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
source /home/robot/spot_ros2_ws/install/setup.bash
ros2 launch spot_driver spot_driver.launch.py config_file:=./src/spot_ros2/configs/spot_ros_config.yaml publish_point_clouds:=True spot_name:=hkaspot
# launch_rviz:=True rviz_config_file:=/home/robot/spot-perception/configs/spot.rviz
