ARG BASE_IMAGE=osrf/ros:melodic-desktop-full

FROM $BASE_IMAGE
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
    apt-get install -y apt-utils git lsb-release build-essential stow neovim tmux && \
    rm -rf /var/lib/apt/lists/*

# Install ROS packages. May not be required if that comes in from the base image
RUN apt-get update && \
    apt-get install -y ros-melodic-rviz ros-melodic-gmapping \
    ros-melodic-map-server ros-melodic-amcl ros-melodic-move-base ros-melodic-dwa-local-planner && \
    rm -rf /var/lib/apt/lists/*


# Set up timezone
ENV TZ 'America/Los_Angeles'
RUN echo $TZ > /etc/timezone && \
    rm /etc/localtime && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata

# Add version info from Github to invalidate the cache if the repo has been updated
ADD https://api.github.com/repos/pvishal/dotfiles/git/refs/heads/master /root/.dotfile.version.json
RUN git clone https://github.com/pvishal/dotfiles.git /root/dotfiles && \
    /root/dotfiles/setup.sh

WORKDIR /code
VOLUME /code

# Enable colors
ENV TERM=xterm-256color

COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
#CMD ["bash"]
