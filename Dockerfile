FROM ros:humble-ros-base

ARG USERNAME=rosdev
ARG UID=1000
ARG GID=$UID

# Install some dependencies packages
RUN apt update -q \
    && apt upgrade -q -y \
    && apt install -y --no-install-recommends \
    software-properties-common \
    python3-pip \
    xauth \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Create and switch to user
RUN groupadd -g $GID $USERNAME \
    && useradd -lm -u $UID -g $USERNAME -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && groupadd -f -g 125 i2c \
    && usermod -aG i2c $USERNAME
USER $USERNAME

# Create workspace so that user own this directory
RUN mkdir -p /home/$USERNAME/ros2_ws/src
WORKDIR /home/$USERNAME/ros2_ws

# Copy configuration files
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /home/$USERNAME/.bashrc \
    && echo 'source /home/'$USERNAME'/ros2_ws/install/setup.bash' >> /home/$USERNAME/.bashrc

# Setup entrypoint
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# Upgrade all packages
RUN sudo apt update && sudo apt upgrade -y


# Install essential packages
RUN sudo apt install -y wget
RUN sudo apt install zip unzip
RUN sudo apt install curl gnupg lsb-release -y
RUN echo 'source /workspaces/ros2-workspace/swerve_robot/install/setup.bash' >> ~/.bashrc
RUN echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc
RUN echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
RUN pip install pyserial
RUN sudo apt install screen -y

RUN sudo apt update && sudo apt upgrade -y
RUN sudo apt install ros-humble-joy -y
RUN sudo pip3 install adafruit-circuitpython-bno055
RUN pip3 install lgpio
RUN sudo apt install i2c-tools -y
