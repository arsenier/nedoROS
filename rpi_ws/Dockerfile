FROM ros:jazzy
SHELL ["/bin/bash", "-c"]
WORKDIR /app
RUN apt-get update && apt-get install -y \
ros- ${ROS_DISTRO}-turtlesim
CMD ["bash"]
