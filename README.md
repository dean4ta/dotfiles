Dotfiles and container for ROVIO
========

This repo is meant to ease installation of a basic VIO module (ROVIO).

The included Dockerfile produces and image that:
 - installs ROS melodic
 - installs ROVIO
 - installs IntelRealsense2
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
Using bagfiles from [ETH Zurich](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)

```shell
# in one terminal instance, play a rosbag ... ie
rosbag play bags/V1_03_difficult.bag
# in a second terminal instance, launch 
roslaunch rovio rovio_node.launch
```

#### D435i
Using Intel Realsense D435i

```shell
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False
source /root/devel/setup.bash
roslaunch hummingbird_bringup bringup.launch
```

---

## Misc

### Todo
- [x] source workspace on launch
- [x] mount container directories to host system (only mounting bags folder)
- [x] add support for D435i
- [ ] simulate with FlightGoggles?
- [ ] if many more git repos are used, use something like git subodules, wstool, or rosinstall
- [ ] try on raspberry pi
- [x] instead of mounting hummingbird_bringup, copy to the container in the Dockerfile to build during docker build
- [ ] fuse imu data (gyro has a faster max fps than the accel but it is throttled to make one imu unified datapoint)
- [ ] calibrate camera and imu with Kalibr
- [ ] feed raw images into into rovio with calibration instead of rectified images
- [ ] disable IR projector on D435i
- [ ] convert dockerfile to a multistage build

### Current Progress
- `07/19/2020:` [Uncalibrated D435i with ROVIO](https://drive.google.com/file/d/1-D7BJQ109dQNYXzZPIWAQNCqemv22a61/view?usp=sharing)

### Calibrating the D435i with Kalibr (currently not included in Dockerfile)

Download and extract the Kalibr package [Kalibr-CDE 64-bit](https://github.com/ethz-asl/kalibr/wiki/downloads).

Print an aprilgrid and find the tag size and spacing and make a yaml file describing the aprilgrid

run this command after collecting a bagfile of both infrared cameras looking at the aprilgrid target:
```shell
kalibr_calibrate_cameras --bag ../../../dotfiles/bags/d435_camera_calibration.bag --topics /cam0/image_raw /cam1/image_raw --models pinhole-radtan pinhole-radtan --target ../../../assets/aprilgrid.yaml
```
or something similar

Tag spacing on [printed apriltag grid](https://github.com/ethz-asl/kalibr/wiki/downloads): 
```
a = 0.026m
b = 0.008m
tag_spacing = b/a = 0.3076923076923077
```