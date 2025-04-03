#!/bin/bash

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to print error and exit
error_exit() {
    echo -e "${RED}Error: $1${NC}" >&2
    exit 1
}

# Function to print success message
success_message() {
    echo -e "${GREEN}$1${NC}"
}

# Function to print info message
info_message() {
    echo -e "${YELLOW}$1${NC}"
}


# Setup ROS 2 Humble
info_message "Setting up ROS 2 Humble..."

# Configure locale
locale  # check for UTF-8
apt update && apt install locales -y || error_exit "Failed to install locales"
locale-gen en_US en_US.UTF-8 || error_exit "Failed to generate locale"
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 || error_exit "Failed to update locale"
export LANG=en_US.UTF-8

#install build-essential nano
apt install build-essential nano -y || error_exit "Failed to install build-essential"

#install net tools
apt install net-tools -y || error_exit "Failed to install net-tools"
# Verify locale settings
locale  # verify settings

# Install required system packages
apt install software-properties-common -y || error_exit "Failed to install software-properties-common"
add-apt-repository universe || error_exit "Failed to add universe repository"
apt update && apt install curl -y || error_exit "Failed to install curl"

# Add ROS 2 GPG key
info_message "Adding ROS 2 GPG key..."
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg || error_exit "Failed to add ROS 2 GPG key"

# Add ROS 2 repository
info_message "Adding ROS 2 repository..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null || error_exit "Failed to add ROS 2 repository"

# Update package lists
info_message "Updating package lists..."
apt update || error_exit "Failed to update package lists"

# Install ROS 2 Humble  
info_message "Installing ROS 2 Humble..."
apt install ros-humble-ros-base -y || error_exit "Failed to install ROS 2 Humble"

# Install development tools
info_message "Installing development tools..."
apt install ros-dev-tools -y || error_exit "Failed to install development tools"

rosdep init || error_exit "Failed to initialize rosdep"
rosdep fix-permissions || error_exit "Failed to fix rosdep permissions"
rosdep update || error_exit "Failed to update rosdep"

# Setup RIO ROS2 Repository
info_message "Setting up RIO ROS2 Repository..."

# Source ROS 2 environment
source /opt/ros/humble/setup.bash || error_exit "Failed to source ROS 2 environment"

# Set up Micro-ROS workspace
info_message "Setting up Micro-ROS workspace..."
mkdir -p ~/uros_ws/src || error_exit "Failed to create Micro-ROS workspace directory"
cd ~/uros_ws/src || error_exit "Failed to change to Micro-ROS workspace directory"
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git || error_exit "Failed to clone Micro-ROS setup repository"

# Build Micro-ROS
info_message "Building Micro-ROS..."
cd ~/uros_ws || error_exit "Failed to change to Micro-ROS workspace"
rosdep update && rosdep install --from-paths src --ignore-src -y || error_exit "Failed to install Micro-ROS dependencies"
colcon build || error_exit "Failed to build Micro-ROS"
source install/setup.bash || error_exit "Failed to source Micro-ROS setup"

# Build Micro-ROS Agent
info_message "Building Micro-ROS Agent..."
ros2 run micro_ros_setup create_agent_ws.sh || error_exit "Failed to create Micro-ROS agent workspace"
ros2 run micro_ros_setup build_agent.sh || error_exit "Failed to build Micro-ROS agent"
source install/setup.bash || error_exit "Failed to source Micro-ROS agent setup"

# Set up RIO workspace
info_message "Setting up RIO workspace..."
mkdir -p ~/rio_ws/src || error_exit "Failed to create RIO workspace directory"
cd ~/rio_ws/src || error_exit "Failed to change to RIO workspace directory"
git clone https://github.com/botforge-robotics/rio_ros2.git || error_exit "Failed to clone RIO repository"

#install dependencies
info_message "Installing dependencies..."
apt-get install python3-pip -y || error_exit "Failed to install python3-pip"
pip install opencv-python aiortc aiohttp_cors aiohttp || error_exit "Failed to install dependencies"

# info_message "Installing ollama..."
# curl -fsSL https://ollama.com/install.sh | sh || error_exit "Failed to install ollama"

# info_message "Starting ollama server..."
# nohup ollama serve >/dev/null 2>&1 &

# # Wait for ollama server to start (10 seconds)
# info_message "Waiting for ollama server to initialize..."
# sleep 10

# info_message "Pulling gemma3:1b model..."
# ollama pull gemma3:1b || error_exit "Failed to pull gemma3:1b model"

# Build RIO packages
info_message "Installing RIO dependencies..."
cd ~/rio_ws || error_exit "Failed to change to RIO workspace"
rosdep install --from-paths src --ignore-src -r -y
info_message "Building RIO packages..."
colcon build || error_exit "Failed to build RIO packages"
source install/setup.bash || error_exit "Failed to source RIO setup"

#update and install rmw cyclonedds
info_message "Updating and installing rmw cyclonedds..."
apt update && apt install ros-humble-rmw-cyclonedds-cpp -y || error_exit "Failed to install rmw cyclonedds"

# Source all environments
source /opt/ros/$ROS_DISTRO/setup.bash || error_exit "Failed to source ROS 2 environment"
source ~/uros_ws/install/setup.bash || error_exit "Failed to source Micro-ROS environment"
source ~/rio_ws/install/setup.bash || error_exit "Failed to source RIO environment"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=156
export ROS_LOCALHOST_ONLY=0

# Add environment sources to .bashrc
info_message "Adding environment sources to .bashrc..."
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc || error_exit "Failed to add ROS 2 source to .bashrc"
echo "source ~/uros_ws/install/setup.bash" >> ~/.bashrc || error_exit "Failed to add Micro-ROS source to .bashrc"
echo "source ~/rio_ws/install/setup.bash" >> ~/.bashrc || error_exit "Failed to add RIO source to .bashrc"
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc || error_exit "Failed to add RMW_IMPLEMENTATION to .bashrc"
echo "export ROS_DOMAIN_ID=156" >> ~/.bashrc || error_exit "Failed to add ROS_DOMAIN_ID to .bashrc"
echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc || error_exit "Failed to add ROS_LOCALHOST_ONLY to .bashrc"

info_message "Installation complete! You can now start the ROS 2 Humble environment with CycloneDDS as the middleware and domain ID 156."