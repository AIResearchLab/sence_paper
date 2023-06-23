FROM ros:noetic

# locale stuff
RUN apt-get update && apt-get install locales
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8

# install ros package
RUN apt-get update && apt-get install -y \
      #ros-${ROS_DISTRO}-demo-nodes-cpp \
      #ros-${ROS_DISTRO}-demo-nodes-py \
      ros-${ROS_DISTRO}-dynamixel-sdk \
      ros-${ROS_DISTRO}-dynamixel-sdk-examples && \
    rm -rf /var/lib/apt/lists/*

# launch ros package
CMD ["bash"]
