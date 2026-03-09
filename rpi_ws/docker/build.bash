#!/usr/bin/env bash

# Written by Nikolay Dema <ndema2301@gmail.com>, Jun 2025
# Modified by Arsenii Iarmolinskii <yarmolinskiyam@gmail.com>, Nov 2025

KOB_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

IMG_NAME="nedoros_ros_02022026"
# BASE_IMG="dustynv/ros:humble-ros-base-l4t-r32.7.1"
# BASE_IMG="alterafpga/ros2"

# docker pull alterafpga/ros2

### DOCKER BUILD --------------------------------------------------------- #

printf "\n BUILDING DOCKER IMAGE: ${IMG_NAME}"
printf "\n                  FROM: ${BASE_IMG}\n\n"

docker build -t "${IMG_NAME}" \
             -f $KOB_ROOT/docker/Dockerfile $KOB_ROOT \
            --network=host \
            --build-arg base_img=${BASE_IMG} \
            --build-arg workspace=${KOB_ROOT}/workspace
