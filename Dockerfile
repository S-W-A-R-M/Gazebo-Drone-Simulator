# Use the official ROS 2 Humble desktop image
FROM osrf/ros:humble-desktop

# Install Gazebo Fortress and the ROS-Gz bridge
RUN apt-get update && apt-get install -y \
    ros-humble-ros-gz \
    ros-humble-gz-ros2-control \
    python3-colcon-common-extensions \
    git \
    && rm -rf /var/lib/apt/lists/*

# Set up environment for the modern Gazebo (Fortress)
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

WORKDIR /ros2_ws