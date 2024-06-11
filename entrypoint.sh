#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
source /home/robot/spot_ros2_ws/install/setup.bash
ros2 launch spot_driver spot_driver.launch.py config_file:=./src/spot_ros2/configs/spot_ros_config.yaml spot_name:=hkaspot &
ros2 launch rtabmap_launch rtabmap.launch.py \
    rgb_topic:=/hkaspot/camera/back/image \
    depth_topic:=/hkaspot/depth_registered/back/image \
    camera_info_topic:=/hkaspot/camera/back/camera_info \
    database_path:=/home/robot/spot-perception/map/rtabmap.db \
    odom_frame_id:=hkaspot/odom \
    odom_topic:=/hkaspot/odometry \
    visual_odometry:=false \
    frame_id:=hkaspot/body \
    subscribe_depth:=false \
    subscribe_rgbd:=false \
    rgbd_sync:=true \
    approx_sync:=true \
    queue_size:=1000 \
    qos:=1 \
    rviz:=false \
    rtabmap_viz:=false \
    localization:=true & 
ros2 launch nav2_bringup navigation_launch.py params_file:=../spot-perception/configs/nav2_params.yaml & 
ros2 launch nav2_bringup rviz_launch.py &
ros2 run spot_driver nav2_to_spot_node
