ARG BASE_IMAGE=osrf/ros:melodic-desktop-full

FROM $BASE_IMAGE as ros_base
ENV DEBIAN_FRONTEND=noninteractive

# Use bash
SHELL ["/bin/bash", "-c"]

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Setup Locales
RUN apt-get update && apt-get install -y locales
ENV LANG="en_US.UTF-8" LC_ALL="en_US.UTF-8" LANGUAGE="en_US.UTF-8"

RUN echo "en_US.UTF-8 UTF-8" > /etc/locale.gen && \
  locale-gen --purge $LANG && \
    dpkg-reconfigure --frontend=noninteractive locales && \
    update-locale LANG=$LANG LC_ALL=$LC_ALL LANGUAGE=$LANGUAGE

# Install basic dev tools
RUN apt-get update && \
    apt-get install -y apt-utils git lsb-release build-essential software-properties-common stow neovim tmux nano && \
    rm -rf /var/lib/apt/lists/*

# Install ROS packages. May not be required if that comes in from the base image
RUN apt-get update && \
    apt-get install -y ros-melodic-rviz && \
    rm -rf /var/lib/apt/lists/*

FROM ros_base as package_installation_stage
## remote packages
RUN mkdir -p /root/src/remote_packages

# install catkin tools
RUN apt-get update && \
    apt-get install -y python-catkin-tools

# Intel Realsense install
RUN git clone --single-branch --branch v2.37.0 https://github.com/IntelRealSense/librealsense.git /root/src/remote_packages/librealsense
RUN git clone https://github.com/IntelRealSense/realsense-ros.git /root/src/remote_packages/realsense-ros && \
    cd /root/src/remote_packages/realsense-ros/ && \
    git checkout `2.2.15` && \
    git clone https://github.com/pal-robotics/ddynamic_reconfigure /root/src/remote_packages/ddynamic_reconfigure

## local packages
RUN mkdir -p /root/src/local_packages

FROM package_installation_stage as build_stage
# build workspace
WORKDIR /root
RUN source /opt/ros/melodic/setup.bash && \
    catkin init && \
    catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False && \
    echo "source /root/devel/setup.bash" >> .bashrc

# Install realsense calibration tool (TODO: Make this optional)
RUN apt-get update && \
    apt-get install -y libusb-dev libusb-1.0-0-dev && \
    apt-get install -y libglfw3 libglfw3-dev && \
    apt-get install -y freeglut3 freeglut3-dev && \
    echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main' | tee /etc/apt/sources.list.d/realsense-public.list && \
    apt-key adv --keyserver keys.gnupg.net --recv-key 6F3EFCDE && \
    apt-get update && \
    apt-get install -y librscalibrationtool

# Set up timezone
ENV TZ 'America/Los_Angeles'
RUN echo $TZ > /etc/timezone && \
    rm /etc/localtime && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata

# Add version info from Github to invalidate the cache if the repo has been updated
ARG GIT_USERNAME
ENV GIT_USERNAME=${GIT_USERNAME}
ADD https://api.github.com/repos/${GIT_USERNAME}/dotfiles/git/refs/heads/master /root/.dotfile.version.json
RUN git clone https://github.com/${GIT_USERNAME}/dotfiles.git /root/dotfiles && \
    /root/dotfiles/setup.sh

# Enable colors
ENV TERM=xterm-256color

FROM build_stage as dev_stage
COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
