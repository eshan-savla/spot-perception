#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/robot/spot_ros2_ws/install/setup.bash

ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/hkaspot/camera/frontright/image \
    depth_topic:=/hkaspot/depth_registered/frontright/image \
    camera_info_topic:=/hkaspot/camera/fronright/camera_info \
    odom_topic:=/hkaspot/odometry \
    frame_id:=/hkaspot/base_link \
    approx_sync:=true \
    wait_imu_to_init:=false \
    qos:=1 \
    rviz:=true
