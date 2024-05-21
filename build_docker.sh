#!/bin/bash

# Install qemu dependencies for cross compiling
if [[ "$1" == "--cross-compile" ]]; then
    docker run --rm --privileged multiarch/qemu-user-static --reset -p yes -c yes  
fi
# Build the Docker containers
 docker buildx build --platform linux/arm64 -m 10g --memory-swap 3g -t eshansavla0512/ros2-spot-arm64:latest .

# Build the slam docker container
#cd ./docker/rtab-map
#docker buildx build --platform linux/arm64 -m 10g --memory-swap 3g -t eshansavla0512/rtabmap-spot-arm64:latest .
