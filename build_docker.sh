#!/bin/bash

# Install qemu dependencies for cross compiling
if [[ "$1" == "--cross-compile" ]]; then
    docker run --rm --privileged multiarch/qemu-user-static --reset -p yes -c yes  
fi
docker buildx build --platform linux/arm64 -t eshansavla0512/ros2-spot-arm64:latest .
# Build the Dockedocker run --rm --privileged multiarch/qemu-user-static --reset -p yes -c yes  r image