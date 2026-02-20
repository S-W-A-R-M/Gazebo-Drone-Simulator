FROM osrf/ros:jazzy-desktop-full

# 1. Install prerequisites for adding repositories
RUN apt-get update && apt-get install -y \
    curl \
    lsb-release \
    gnupg

# 2. Add the OSRF GPG key and repository
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# 3. Now install Gazebo Harmonic and the ROS-GZ Bridge
RUN apt-get update && apt-get install -y \
    gz-harmonic \
    ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# Set up the environment
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

WORKDIR /home/drone_ws