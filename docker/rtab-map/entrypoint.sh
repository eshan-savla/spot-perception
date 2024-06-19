#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash

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
    subscribe_rgbd:=true \
    rgbd_sync:=true \
    approx_sync:=true \
    queue_size:=1000 \
    qos:=1 \
    rviz:=false \
    rtabmap_viz:=false \
    localization:=true & 
ros2 run rtabmap_util point_cloud_xyz -r depth/image:=/hkaspot/depth/frontleft/Image -r depth/camera_info:=/hkaspot/depth/fronleft/camera_info -p voxel_size:=0.05 -p decimation:=4 -p max_depth:=10 &
ros2 run rtabmal_util obstacles_detection -p frame_id:=hkaspot/body &
ros2 launch nav2_bringup navigation_launch.py params_file:=spot-perception/configs/nav2_params.yaml & 
ros2 launch nav2_bringup rviz_launch.py
