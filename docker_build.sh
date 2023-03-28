#!/bin/bash

CONTAINER_IMAGE=ros_noetic_gui:latest
docker build -t $CONTAINER_IMAGE .