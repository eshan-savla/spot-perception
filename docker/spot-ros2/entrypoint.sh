#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash # sourcing ROS installation
source /home/robot/spot_ros2_ws/install/setup.bash # Sourcing Driver package
# Launch driver with path to yaml file, spot-name, and point cloud publishing set to on for obstacle detection
ros2 launch spot_driver spot_driver.launch.py config_file:=./src/spot_ros2/configs/spot_ros_config.yaml spot_name:=hkaspot publish_point_clouds:=True
