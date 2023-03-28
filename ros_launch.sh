#!/bin/bash

CONTAINER_IMAGE=ros_noetic_gui:latest
CONTAINER_NAME=ros_noetic_container

EXTERNAL_MOUNT="$(pwd)"/workspace
INTERNAL_MOUNT=/ros_ws

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth

DRIVER=/dev/dri

sudo rm -r $XAUTH

touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

# --privileged -v /dev/video0:/dev/video0

docker run -it --rm --name=$CONTAINER_NAME --volume=$XSOCK:$XSOCK:rw --volume=$DRIVER:$DRIVER --volume=$XAUTH:$XAUTH:rw --volume=$EXTERNAL_MOUNT:$INTERNAL_MOUNT --env="XAUTHORITY=${XAUTH}" --env="DISPLAY" $CONTAINER_IMAGE

#docker run -it --rm --name=$CONTAINER_NAME --privileged -v /dev/video0:/dev/video0 --volume=$XSOCK:$XSOCK:rw --volume=$DRIVER:$DRIVER --volume=$XAUTH:$XAUTH:rw --volume=$EXTERNAL_MOUNT:$INTERNAL_MOUNT --env="XAUTHORITY=${XAUTH}" --env="DISPLAY" $CONTAINER_IMAGE