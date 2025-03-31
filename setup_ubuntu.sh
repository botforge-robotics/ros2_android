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

# Check if running as root
if [ "$EUID" -eq 0 ]; then 
    error_exit "Please do not run as root"
fi

echo -e "${GREEN}Starting installation of Ubuntu 22.04 and ROS 2 Humble on Termux...${NC}"

# Install neofetch and show system info
info_message "Installing neofetch and updating system..."
pkg install neofetch -y || error_exit "Failed to install neofetch"
apt update -y && apt upgrade -y || error_exit "Failed to update system"
neofetch

# Install required packages
info_message "Installing required packages..."
pkg update -y || error_exit "Failed to update pkg"
pkg install wget curl proot tar -y || error_exit "Failed to install required packages"

# Install net-tools
info_message "Installing net-tools..."
apt install net-tools -y || error_exit "Failed to install net-tools"

# Download and setup Ubuntu 22.04
info_message "Downloading Ubuntu 22.04 setup script..."
wget https://raw.githubusercontent.com/AndronixApp/AndronixOrigin/master/Installer/Ubuntu22/ubuntu22.sh -O ubuntu22.sh || error_exit "Failed to download Ubuntu setup script"
chmod +x ubuntu22.sh || error_exit "Failed to run Ubuntu setup script"

# Start Ubuntu environment
info_message "To start the Ubuntu environment, run './start-ubuntu22.sh' in your terminal."
