#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
source /home/robot/spot_ros2_ws/install/setup.bash
ros2 launch spot_driver spot_driver.launch.py config_file:=./src/spot_ros2/configs/spot_ros_config.yaml spot_name:=hkaspot &
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/hkaspot/camera/back/image \
    depth_topic:=/hkaspot/depth_registered/back/image \
    camera_info_topic:=/hkaspot/camera/back/camera_info \
    odom_topic:=/hkaspot/odometry \
    frame_id:=hkaspot/odom \
    approx_sync:=true \
    approx_rgbd_sync:=false \
    rgbd_sync:=false \
    wait_imu_to_init:=false \
    queue_size:=10000 \
    qos:=1 \
    rviz:=true
