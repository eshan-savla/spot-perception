#!/bin/bash
xhost + 
# docker pull eshansavla0512/ros2-spot-arm64:latest
docker run -it --net=host \
    --rm \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/home/spot/spot-perception:/home/robot/spot-perception:rw" \
    --name spot_ros2_slam \
    --privileged eshansavla0512/rtabmap-spot-arm64:latest
