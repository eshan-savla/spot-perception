# Built docker image available publically for arm64 under eshansavla0512/ros2-spot-arm64
FROM arm64v8/ros:humble-ros-base-jammy

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

#install desktop-full packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop-full=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*
    
##############################################################################
##                                 Base Image                               ##
##############################################################################
ARG ROS_DISTRO=humble
ARG ROS_PKG=desktop-full-jammy
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

##############################################################################
##                                 Global Dependecies                       ##
##############################################################################
#POSIX standards-compliant default locale. Only strict ASCII characters are valid, extended to allow the basic use of UTF-8
ENV LANG C.UTF-8 
ENV LC_ALL C.UTF-8
ENV LC_ALL=C

RUN apt-get update && apt-get install --no-install-recommends -y \
    dirmngr gnupg2 lsb-release can-utils iproute2\
    apt-utils bash nano aptitude util-linux \
    htop git tmux sudo wget gedit bsdmainutils \
    pip && \
    rm -rf /var/lib/apt/lists/*
	
##############################################################################
##                                 Create User                              ##
##############################################################################
ARG USER=robot
ARG PASSWORD=robot
ARG UID=1000
ARG GID=1000
ARG DOMAIN_ID=8
ARG VIDEO_GID=44
ENV ROS_DOMAIN_ID=${DOMAIN_ID}
ENV UID=${UID}
ENV GID=${GID}
ENV USER=${USER}
RUN groupadd -g "$GID" "$USER"  && \
    useradd -m -u "$UID" -g "$GID" --shell $(which bash) "$USER" -G sudo && \
    groupadd realtime && \
    groupmod -g ${VIDEO_GID} video && \
    usermod -aG video "$USER" && \
    usermod -aG dialout "$USER" && \
    usermod -aG realtime "$USER" && \
    echo "$USER:$PASSWORD" | chpasswd && \
    echo "%sudo ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudogrp
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc
RUN echo "export ROS_DOMAIN_ID=${DOMAIN_ID}" >> /etc/bash.bashrc

USER $USER 
# RUN mkdir -p /home/$USER/ros2_ws/src

##############################################################################
##                                 Nav2 Dependecies                         ##
##############################################################################
USER root
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-navigation2 \
                                         ros-$ROS_DISTRO-nav2-bringup && \
                                        rm -rf /var/lib/apt/lists/*

RUN apt-get update && DEBIAN_FRONTEND=noninteractive && \
    apt-get install -y ros-$ROS_DISTRO-joint-state-publisher-gui \
                       ros-$ROS_DISTRO-ros-ign \
                    #    ros-$ROS_DISTRO-navigation2 \
                    #    ros-$ROS_DISTRO-nav2-bringup \
                    #   ros-$ROS_DISTRO-gazebo-ros-pkgs \
                    #    ros-$ROS_DISTRO-robot-localization \
                    #   ros-$ROS_DISTRO-gazebo-ros2-control \
                       ros-$ROS_DISTRO-joint-state-broadcaster \
                    #    ros-$ROS_DISTRO-diff-drive-controller && \
    && rm -rf /var/lib/apt/lists/*

##############################################################################
##                               Spot-ROS2 Drivers                          ##
##############################################################################

RUN apt-get install -y git

RUN mkdir -p /home/$USER/spot_ros2_ws/src
	
WORKDIR /home/$USER/spot_ros2_ws/src

RUN git clone https://github.com/bdaiinstitute/spot_ros2.git

WORKDIR /home/$USER/spot_ros2_ws/src/spot_ros2

RUN git submodule init && git submodule update

RUN ./install_spot_ros2.sh --arm64

#RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install # --packages-ignore proto2ros_tests

USER $USER

##############################################################################
##                                 Build ROS and run                        ##
##############################################################################
WORKDIR /home/$USER/
 
CMD /bin/bash

