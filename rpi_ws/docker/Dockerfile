# Written by Nikolay Dema <ndema2301@gmail.com>, Jun 2025
# Modified by Arsenii Iarmolinskii <yarmolinskiyam@gmail.com>, Nov 2025

# ARG base_img="nickodema/kobuki:humble-22.04-100625"
# ARG base_img="dustynv/ros:humble-ros-base-l4t-r32.7.1"
# ARG base_img="alterafpga/ros2"

# FROM ${base_img}
FROM arm64v8/ros:humble

# https://answers.ros.org/question/410123/
# apt update fix
# RUN rm /etc/apt/sources.list.d/ros2.list
# RUN curl http://repo.ros2.org/repos.key | sudo apt-key add -
# RUN cp /etc/apt/trusted.gpg /usr/share/keyrings/ros-archive-keyring.gpg
# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

 RUN apt-get update

# # lidar driver
RUN apt-get install -y ros-humble-diagnostic-updater
RUN apt-get install -y libpcap-dev
RUN apt-get install -y libboost-all-dev
RUN apt-get install -y python3-pip && pip install pyserial && pip install transforms3d
# # zmrobo driver
# RUN apt-get install -y ros-humble-ackermann-msgs

# RUN . /opt/ros/humble/setup.sh && git clone https://github.com/ZhaoXiangBox/serial.git && \
#     cd serial && \
#     mkdir build && \
#     cd build && \
#     cmake .. && \
#     make && \
#     sudo make install


# RUN apt-get update && apt-get install -y ros-humble-pointcloud-to-laserscan ros-humble-slam-toolbox

# RUN apt-get install -y python3-pip

# RUN python3.10 -m pip install torch torchvision

### ---------------------------------------------------------------------- #

RUN apt-get install -y ros-humble-usb-cam
