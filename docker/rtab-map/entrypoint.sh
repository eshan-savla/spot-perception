source /opt/ros/$ROS_DISTRO/setup.bash
source /home/robot/spot_ros2_ws/install/setup.bash
ros2 launch zed_wrapper zed.launch.py

ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/hkaspot/camera/frontright/Image \
    depth_topic:=/hkaspot/depth/frontright/Image \
    camera_info_topic:=/hkaspot/depth/fronright/camera_info \
    frame_id:=/hkaspot/vision \
    approx_sync:=false \
    wait_imu_to_init:=false \
    qos:=1 \
    rviz:=true