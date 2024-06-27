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
    approx_sync:=false \
    queue_size:=1000 \
    qos:=1 \
    rviz:=true \
    rtabmap_viz:=true \
    localization:=false & 
ros2 run rtabmap_util point_cloud_xyz --ros-args -r depth/image:=/hkaspot/depth_registered/frontleft/image -r depth/camera_info:=/hkaspot/depth_registered/frontleft/camera_info -p voxel_size:=0.05 -p decimation:=4 -p max_depth:=10.0 -p approx_sync:=false &
ros2 run rtabmap_util obstacles_detection --ros-args -p frame_id:=hkaspot/body -p map_frame_id:=map &
ros2 launch nav2_bringup navigation_launch.py params_file:=spot-perception/configs/nav2_params.yaml
# ros2 launch nav2_bringup rviz_launch.py
