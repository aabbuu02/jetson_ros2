#!/bin/bash

echo "🔧 Installing Zeus ROS2 Dependencies..."

# Update package list
sudo apt update

# Install ROS2 Humble (if not already installed)
if ! command -v ros2 &> /dev/null; then
    echo "📦 Installing ROS2 Humble..."
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install -y curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install -y ros-humble-desktop
fi

# Install system dependencies
echo "📦 Installing system dependencies..."
sudo apt install -y \
    libmodbus-dev \
    libserial-dev \
    libboost-all-dev \
    libtinyxml2-dev \
    libsqlite3-dev \
    libdxflib-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip

# Install Python dependencies
echo "🐍 Installing Python dependencies..."
pip3 install --user \
    numpy \
    matplotlib \
    scipy \
    pyyaml

# Install ROS2 development tools
echo "🛠️ Installing ROS2 development tools..."
sudo apt install -y \
    ros-humble-ament-cmake \
    ros-humble-rclcpp \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-visualization-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-ament-index-cpp

echo "✅ Dependencies installation complete!"
echo "🚀 You can now build and test your Zeus ROS2 workspace!"
