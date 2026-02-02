ARG BASE_IMAGE=ubuntu:22.04
FROM ${BASE_IMAGE} AS downloader

ARG WEBOTS_VERSION=R2025a
ARG WEBOTS_PACKAGE_PREFIX=

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install --yes wget bzip2 && rm -rf /var/lib/apt/lists/ && \
  wget https://github.com/cyberbotics/webots/releases/download/$WEBOTS_VERSION/webots-$WEBOTS_VERSION-x86-64$WEBOTS_PACKAGE_PREFIX.tar.bz2 && \
  tar xjf webots-*.tar.bz2 && rm webots-*.tar.bz2

FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

# Install Webots runtime dependencies
RUN apt-get update && apt-get install --yes wget xvfb locales && rm -rf /var/lib/apt/lists/ && \
  wget https://raw.githubusercontent.com/cyberbotics/webots/master/scripts/install/linux_runtime_dependencies.sh && \
  chmod +x linux_runtime_dependencies.sh && ./linux_runtime_dependencies.sh && rm ./linux_runtime_dependencies.sh && rm -rf /var/lib/apt/lists/

# Install Webots
WORKDIR /usr/local
COPY --from=downloader /webots /usr/local/webots/
ENV QTWEBENGINE_DISABLE_SANDBOX=1
ENV WEBOTS_HOME=/usr/local/webots
ENV PATH=/usr/local/webots:${PATH}

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y \
  software-properties-common \
  curl \
  vim \
  git \
  nano \
  gnupg2 \
  lsb-release && \
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
  apt-get update && apt-get install -y \
  ros-humble-desktop \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip && \
  rm -rf /var/lib/apt/lists/*

# Install webots_ros2
RUN apt-get update && apt-get install -y \
  ros-humble-webots-ros2 \
  ros-humble-webots-ros2-driver && \
  rm -rf /var/lib/apt/lists/*

# Install Navigation and Mapping packages
RUN apt-get update && apt-get install -y \
  # Navigation2 stack
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-common \
  ros-humble-nav2-msgs \
  # SLAM and Mapping
  ros-humble-slam-toolbox \
  ros-humble-rtabmap \
  ros-humble-rtabmap-ros \
  ros-humble-rtabmap-conversions \
  ros-humble-rtabmap-util \
  ros-humble-rtabmap-slam \
  ros-humble-rtabmap-viz \
  # Localization
  ros-humble-robot-localization \
  ros-humble-nav2-amcl \
  # Sensor processing
  ros-humble-pointcloud-to-laserscan \
  ros-humble-laser-filters \
  ros-humble-depthimage-to-laserscan \
  # Perception
  ros-humble-perception-pcl \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-image-transport \
  ros-humble-cv-bridge \
  ros-humble-vision-opencv \
  # TF and transforms
  ros-humble-tf2-tools \
  ros-humble-tf2-geometry-msgs \
  ros-humble-tf-transformations \
  # Visualization
  ros-humble-rviz2 \
  ros-humble-rviz-common \
  ros-humble-rviz-default-plugins \
  # Common utilities
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  # Controllers
  ros-humble-controller-manager \
  ros-humble-diff-drive-controller \
  ros-humble-joint-state-broadcaster \
  # Additional tools
  ros-humble-teleop-twist-keyboard \
  ros-humble-teleop-twist-joy \
  ros-humble-joy \
  && rm -rf /var/lib/apt/lists/*

# Install additional system dependencies for rtabmap
RUN apt-get update && apt-get install -y \
  libsqlite3-dev \
  libpcl-dev \
  libopencv-dev \
  libproj-dev \
  libqt5svg5-dev \
  libopenni-dev \
  libopenni2-dev \
  freeglut3-dev \
  libglew-dev \
  libusb-1.0-0-dev \
  libudev-dev \
  && rm -rf /var/lib/apt/lists/*

# Install Python dependencies for navigation and mapping
RUN pip3 install --no-cache-dir \
  transforms3d \
  numpy \
  scipy \
  matplotlib \
  opencv-python

# ROS BRIDGE
RUN apt-get update && apt-get install -y \
    ros-humble-ament-cmake-mypy \
    ros-humble-rosbridge-suite \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Set up workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Set the locales
RUN locale-gen en_US.UTF-8
ENV LANG='en_US.UTF-8'
ENV LANGUAGE='en_US:en'
ENV LC_ALL='en_US.UTF-8'

# Create non-root user for better security (optional, commented out for compatibility)
# RUN useradd -m -s /bin/bash -u 1000 ros_user && \
#     chown -R ros_user:ros_user /ros2_ws
# USER ros_user

# Set user (keeping root for compatibility with mounted volumes)
ENV USER=root

# Source ROS 2 in bashrc for interactive shells
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
  echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> ~/.bashrc

CMD ["/bin/bash"]