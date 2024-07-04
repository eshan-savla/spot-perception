# Built docker image available publically for arm64 under eshansavla0512/ros2-spot-arm64
FROM introlab3it/rtabmap_ros:humble-latest

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
ARG DOMAIN_ID=33
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

COPY docker/dds_profile.xml /home/$USER
RUN chown $USER:$USER /home/$USER/dds_profile.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/dds_profile.xml
##############################################################################
##                                 Nav2 Dependecies                         ##
##############################################################################

# install ros2 packages

USER root

RUN apt-get update && DEBIAN_FRONTEND=noninteractive && \
    apt-get install -y \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-opennav-docking \
    ros-$ROS_DISTRO-opennav-docking-bt \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-joint-state-broadcaster \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-tf-transformations \
    ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-depth-image-proc \
    ros-$ROS_DISTRO-tl-expected \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && DEBIAN_FRONTEND=noninteractive && \
    apt-get install -y ros-$ROS_DISTRO-opennav-docking \
    && rm -rf /var/lib/apt/lists/*
COPY --chown=$USER --chmod=0444 launch/rtabmap.launch.py /opt/ros/humble/share/rtabmap_launch/launch/rtabmap.launch.py
COPY --chown=$USER --chmod=0444 launch/navigation_launch.py /opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py
COPY --chown=$USER --chmod=0444 configs/rgbd.rviz /opt/ros/humble/share/rtabmap_launch/launch/config/rgbd.rviz
COPY --chown=$USER configs/nav2_params.yaml /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml
USER ${USER}
WORKDIR /home/${USER}
COPY --chown=$USER:$USER --chmod=0755 docker/rtab-map/entrypoint.sh /home/$USER/entrypoint.sh
CMD [ "/bin/bash" ]
ENTRYPOINT [ "/home/robot/entrypoint.sh" ]
