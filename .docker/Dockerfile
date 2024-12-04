# Specify the base image
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base AS base

# Set environment variables
ARG ROS_DISTRO
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm-256color

# Install basic utilities and dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    curl \
    terminator \
    iputils-ping \
    openssh-server \
    iproute2 \
    python3.10 \
    python3.10-distutils \
    python3.10-venv \
    python3-pip \
    nano \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-flake8 \
    python3-setuptools \
    python3-vcstool \
    freeglut3-dev \
    lsb-release \
    gnupg \
    libglew-dev \
    libqt5opengl5-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --no-cache-dir --upgrade pip \
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-return \
    flake8-length \
    flake8-todo \
    flake8-quotes \
    black \
    mypy \
    prospector \
    pylint \
    numpy \
    opencv-python \
    pydantic==1.10.14 \
    pandas \
    matplotlib \
    rosbags

# Add the Gazebo package repository and install Ignition Fortress
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
    && apt-get update && apt-get install -y ignition-fortress

# Install ROS 2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-rqt \
    ros-$ROS_DISTRO-rqt-common-plugins \
    ros-$ROS_DISTRO-rqt-robot-dashboard \
    ros-$ROS_DISTRO-rqt-robot-steering \
    ros-$ROS_DISTRO-rqt-tf-tree \
    ros-$ROS_DISTRO-image-transport-plugins \
    ros-$ROS_DISTRO-compressed-image-transport \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-rosbag2-py \
    ros-$ROS_DISTRO-usb-cam \
    ros-$ROS_DISTRO-ur \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-ros-testing \
    ros-$ROS_DISTRO-ros2bag \
    ros-$ROS_DISTRO-ign-ros2-control \
    ros-$ROS_DISTRO-moveit \
    ros-$ROS_DISTRO-moveit-servo \
    ros-$ROS_DISTRO-ros-ign-bridge \
    ros-$ROS_DISTRO-ros-ign-gazebo \
    ros-$ROS_DISTRO-ament-cmake \
    ros-$ROS_DISTRO-moveit-visual-tools \
    libgflags-dev \
    nlohmann-json3-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Create a ROS 2 workspace and set the working directory
RUN mkdir -p /dev_ws/src
WORKDIR /dev_ws

# Copy source files into the workspace
COPY . /dev_ws/src

# Install dependencies using rosdep
RUN apt-get update && rosdep install --ignore-src --from-paths src -y \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Clone the Orbbec SDK ROS 2 repository
WORKDIR /dev_ws/src
RUN git clone https://github.com/orbbec/OrbbecSDK_ROS2.git

# Build the workspace with ROS 2
WORKDIR /dev_ws
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    rm -rf build install log && \
    colcon build --merge-install --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Add setup and entrypoint scripts
RUN /bin/bash -c "cp /dev_ws/src/.docker/setup.bash /dev_ws/setup.bash && chmod 777 /dev_ws/setup.bash"
RUN /bin/bash -c "cp /dev_ws/src/.docker/entrypoint.bash / && chmod 777 /entrypoint.bash"

# Rebuild the workspace to ensure clean setup
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    rm -rf build install log && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Set the entrypoint
ENTRYPOINT ["bash", "/entrypoint.bash"]

# Default command
CMD ["bash"]

