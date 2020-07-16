Dotfiles and container for ROVIO
========

This repo is meant to ease installation of a basic VIO module (ROVIO).

The included Dockerfile produces and image that:
 - installs ROS melodic
 - installs dependencies for ROVIO
 - installs ROVIO
 - sets a display environment (allows for Rviz and other displayed applications)


## Running via Docker

```shell
# set environment variables in .env
make build
make up-display # make up # for no display
```

## Running on host

```shell
git clone https://github.com/pvishal/dotfiles.git 
~/dotfiles
~/dotfiles/setup.sh
```
