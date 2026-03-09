ARG ROS_VERSION=hubmle

# FROM osrf/ros:${ROS_VERSION}-desktop
# ARG base_img="dustynv/ros:humble-ros-base-l4t-r32.7.1"

# FROM ${base_img}
FROM arm64v8/ros:humble

SHELL ["/bin/bash", "-c"]

# Username
ENV user=vizanti

# https://answers.ros.org/question/410123/
# apt update fix
# RUN rm /etc/apt/sources.list.d/ros2.list
# RUN curl http://repo.ros2.org/repos.key | sudo apt-key add -
# RUN cp /etc/apt/trusted.gpg /usr/share/keyrings/ros-archive-keyring.gpg
# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and build the vizanti package
RUN apt-get update

# Need openssh-client for git to work properly, can be removed when rosbridge has binaries properly updated.
RUN apt-get install openssh-client -y
RUN apt-get install ros-$ROS_DISTRO-rmw-fastrtps-dynamic-cpp -y
RUN apt-get install ros-$ROS_DISTRO-rmw-cyclonedds-cpp -y

# Only add our user if does not exist.
RUN id -u $user &>/dev/null || useradd -ms /bin/bash $user
RUN chown $user: /home
RUN usermod -o -u 1000 -g 1000 $user

USER $user
RUN mkdir -p /home/$user/colcon_ws/src
COPY . /home/$user/colcon_ws/src/vizanti/.

# Note: There are breaking changes in the rosbridge suite for ROS versions. We need to switch based on version.
RUN if [ "${ROS_DISTRO}" == "humble" ] ; then \
      cd /home/$user/colcon_ws/src/ \
      && git clone -b humble https://github.com/RobotWebTools/rosbridge_suite.git ; \
    else \
      cd /home/$user/colcon_ws/src/ \
      && git clone -b ros2 https://github.com/RobotWebTools/rosbridge_suite.git ; \
    fi

# Install the RWS implementation alternative to ROSbridge.
RUN cd /home/$user/colcon_ws/src/ \
    && git clone -b $ROS_DISTRO https://github.com/v-kiniv/rws.git ;

RUN rosdep update

USER root
RUN apt-get update
RUN cd /home/$user/colcon_ws && rosdep install -i --from-path src -y ;
# USER $user

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && cd /home/$user/colcon_ws && colcon build

# Launch the Vizanti with RWS or ROSBridge
CMD source /home/$user/colcon_ws/install/setup.bash \
    && \
    if [ "${USE_RWS}" == "true" ] ; then \
      ros2 launch vizanti_server vizanti_rws.launch.py ; \
    else \
      ros2 launch vizanti_server vizanti_server.launch.py ; \
    fi