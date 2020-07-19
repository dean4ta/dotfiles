#!/bin/bash

set -e
source /opt/ros/$ROS_DISTRO/setup.bash
source /root/devel/setup.bash
tmux new -s ROS_SHELL
exec "$@"

