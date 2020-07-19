Dotfiles and container for ROVIO
========

This repo is meant to ease installation of a basic VIO module (ROVIO).

The included Dockerfile produces and image that:
 - installs ROS melodic
 - installs dependencies for ROVIO
 - installs ROVIO
 - sets a display environment (allows for Rviz and other displayed applications)

## Requirements
 - [Docker](https://docs.docker.com/get-docker/)
 - Tested on Ubuntu 18.04

## Running via Docker

```shell
# set environment variables in .env
make build
make up-display # make up # for no display
```

## Reconmended usages

#### Bagfiles
As of right now, the only way I have implemented the use of this repo is with bag files from [ETH Zurich](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)

```shell
# in one terminal instance, play a rosbag ... ie
rosbag play bags/V1_03_difficult.bag
# in a second terminal instance, launch 
roslaunch rovio rovio_node.launch
```

### Todo
- [x] source workspace on launch
- [x] mount container directories to host system (only mounting bags folder)
- [ ] add support for D435i
- [ ] simulate with FlightGoggles?
- [ ] if many more git repos are used, use something like git subodules, wstool, or rosinstall
- [ ] try on raspberry pi
