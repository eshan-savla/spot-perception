# Built docker image available publically for arm64 under eshansavla0512/ros2-spot-arm64
FROM arm64v8/ros:humble-ros-base-jammy
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
# important to keep domain ID common for all containers and ROS machines used in the network
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
##                               install dependencies and ROS2 and Nav2     ##
##############################################################################
USER root
RUN apt-get update && DEBIAN_FRONTEND=noninteractive && apt-get install -y --no-install-recommends ros-$ROS_DISTRO-desktop \
    ros-$ROS_DISTRO-desktop-full \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-joint-state-broadcaster \
    ros-$ROS_DISTRO-ros-ign \
    && apt-get -y autoremove \
    && apt-get clean autoclean \
    && rm -fr /var/lib/apt/lists/*

    # removed for now:     ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
##############################################################################
##                               Spot-ROS2 Drivers                          ##
##############################################################################
USER $USER

RUN mkdir -p /home/$USER/spot_ros2_ws/src
	
WORKDIR /home/$USER/spot_ros2_ws/src

RUN git clone https://github.com/Banane01/spot_ros2.git

WORKDIR /home/$USER/spot_ros2_ws/src/spot_ros2

RUN git checkout NavigateTo_bugfix && git submodule init && git submodule update && yes|./install_spot_ros2.sh --arm64 && mkdir -p /home/$USER/spot_ros2_ws/src/spot_ros2/configs

WORKDIR /home/$USER/spot_ros2_ws

COPY --chown=$USER ./configs/CMakeLists.txt /home/$USER/spot_ros2_ws/src/spot_ros2/spot_driver/CMakeLists.txt

COPY --chown=$USER --chmod=0444 ./launch/translate_node.py /home/$USER/spot_ros2_ws/src/spot_ros2/spot_driver/spot_driver/translate_node.py

RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install --packages-ignore proto2ros_tests --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY --chown=$USER --chmod=0444 configs/spot_ros_config.yaml /home/$USER/spot_ros2_ws/src/spot_ros2/configs/spot_ros_config.yaml

##############################################################################
##                                 Build ROS and run                        ##
##############################################################################
WORKDIR /home/$USER/spot_ros2_ws

COPY --chown=$USER --chmod=0755 docker/spot-ros2/entrypoint.sh /home/$USER/spot_ros2_ws/entrypoint.sh
CMD [ "/bin/bash" ]

ENTRYPOINT ["./entrypoint.sh"]
