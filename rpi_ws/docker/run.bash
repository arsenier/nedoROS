#!/bin/bash

# Written by Nikolay Dema <ndema2301@gmail.com>, Jun 2025
# Modified by Arsenii Iarmolinskii <yarmolinskiyam@gmail.com>, Nov 2025

KOBUKI_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

xhost +local:docker > /dev/null || true

IMG_NAME="nedoros_ros_02022026"
CTR_NAME="zmrobo"


### DOCKER RUN ----------------------------------------------------------- #

# docker run  -d -ti --rm                            \
#             -e "DISPLAY"                           \
#             -e "QT_X11_NO_MITSHM=1"                \
#             -e XAUTHORITY                          \
#             -e ROS_DOMAIN_ID=42                    \
#             -v /tmp/.X11-unix:/tmp/.X11-unix:rw    \
#             -v /etc/localtime:/etc/localtime:ro    \
#             -v ${KOBUKI_ROOT}/workspace:/workspace \
#             -v /dev:/dev                           \
#             -v /dev/shm:/dev/shm                   \
#             --net=host                             \
#             --privileged                           \
#             --ipc=host                             \
#             --pid=host                             \
#             --name ${CTR_NAME} ${IMG_NAME}         \
#             > /dev/null

#docker run  -d --rm -it                             \
#            --runtime nvidia                        \
#            --network host                          \
#            --gpus all                              \
#            -e DISPLAY                              \
#            -e "QT_X11_NO_MITSHM=1"                \
#            -e XAUTHORITY                          \
#            -e ROS_DOMAIN_ID=42                     \
#            -v /tmp/.X11-unix:/tmp/.X11-unix:rw    \
#            -v /etc/localtime:/etc/localtime:ro    \
#            -v ${KOBUKI_ROOT}/workspace:/workspace \
#            -v /dev:/dev                           \
#            -v /dev/shm:/dev/shm                   \
#            --privileged                           \
#            --ipc=host                             \
#            --pid=host                             \
#            --name ${CTR_NAME} ${IMG_NAME}          \
#            > /dev/null

docker run  -d --rm -it                             \
            --network host                          \
            -e DISPLAY                              \
            -e "QT_X11_NO_MITSHM=1"                \
            -e XAUTHORITY                          \
            -e ROS_DOMAIN_ID=42                     \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw    \
            -v /etc/localtime:/etc/localtime:ro    \
            -v ${KOBUKI_ROOT}/workspace:/workspace \
            -v /dev:/dev                           \
            -v /dev/shm:/dev/shm                   \
            --privileged                           \
            --ipc=host                             \
            --pid=host                             \
            --name ${CTR_NAME} ${IMG_NAME}          \
            > /dev/null
