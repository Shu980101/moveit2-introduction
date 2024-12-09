ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base AS base

ARG ROS_DISTRO

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm-256color
ENV CMAKE_PREFIX_PATH=/opt/ros/$ROS_DISTRO:$CMAKE_PREFIX_PATH


RUN apt-get update && apt-get install -y --no-install-recommends\
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
    lsb-release gnupg \
    libglew-dev \
    libqt5opengl5-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --upgrade pip\
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
    panda \
    matplotlib \
    rosbags     


RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
    && apt-get update && apt-get install -y ignition-fortress

RUN apt-get update && apt-get install -y --no-install-recommends\
    ros-$ROS_DISTRO-rqt \
    ros-$ROS_DISTRO-rqt-common-plugins \
    ros-$ROS_DISTRO-rqt-robot-dashboard \
    ros-$ROS_DISTRO-rqt-robot-steering \
    ros-$ROS_DISTRO-rqt-tf-tree \
    ros-$ROS_DISTRO-rqt-tf-tree \
    ros-$ROS_DISTRO-image-transport-plugins \
    ros-$ROS_DISTRO-compressed-image-transport \
    # ros-$ROS_DISTRO-clearpath-desktop \
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
    ros-$ROS_DISTRO-ament-clang-format \
    ros-$ROS_DISTRO-moveit-visual-tools \
    libgflags-dev nlohmann-json3-dev  \
    ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager \
    ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-statistics-msgs \
    ros-$ROS_DISTRO-backward-ros libdw-dev\
    && apt-get clean && rm -rf /var/lib/apt/lists/*


RUN mkdir -p /dev_ws/src

# Clone the Orbbec SDK ROS2 repository into the workspace
WORKDIR /dev_ws/src
RUN git clone https://github.com/orbbec/OrbbecSDK_ROS2.git

# Go back to the workspace root
WORKDIR /dev_ws
COPY . /dev_ws/src

# Update apt package lists before installing dependencies
RUN apt-get update && apt-get install -y libgoogle-glog-dev

# Install dependencies with rosdep
RUN rosdep install --from-paths src --ignore-src -y


# Clean the workspace (if necessary) to avoid layout issues from previous builds
RUN rm -rf install build log

RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-ament-cmake
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc


# Build the workspace
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --merge-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Ensure that .docker/setup.bash exists before attempting to copy it
# If it's not available, you can create it or adjust the source path
RUN ["/bin/bash", "-c", "cp /dev_ws/src/.docker/setup.bash /dev_ws/setup.bash && chmod 777 /dev_ws/setup.bash"]
RUN ["/bin/bash", "-c", "cp /dev_ws/src/.docker/entrypoint.bash / && chmod 777 /entrypoint.bash"]



ENTRYPOINT ["bash", "/entrypoint.bash"]

CMD ["bash"]


