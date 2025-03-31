# ROS 2 Humble on Android (Termux)

This repository contains scripts to install and run ROS 2 Humble on Android devices using Termux. The setup includes Ubuntu 22.04, ROS 2 Humble, Micro-ROS, and RIO ROS2 packages.

## Prerequisites

1. Android device running Android 7.0 or higher
2. At least 4GB of free storage space
3. Internet connection
4. Termux app (latest version)

## Installation Steps

### 1. Install Termux App

1. Download the latest Termux app from the official releases:
   - For Android 7+ devices: [termux-app_v0.119.0-beta.2+apt-android-7-github-debug_universal.apk](https://github.com/termux/termux-app/releases/download/v0.119.0-beta.2/termux-app_v0.119.0-beta.2+apt-android-7-github-debug_universal.apk)
   - For Android 5/6 devices: [termux-app_v0.119.0-beta.2+apt-android-5-github-debug_universal.apk](https://github.com/termux/termux-app/releases/download/v0.119.0-beta.2/termux-app_v0.119.0-beta.2+apt-android-5-github-debug_universal.apk)

2. Install the downloaded APK on your device
3. Open Termux app and grant necessary permissions

### 2. Installation Process (Two-Step)

#### Step 1: Setup Ubuntu (in Termux)
```bash
# Update Termux and get required packages
pkg update -y
pkg upgrade -y
pkg install wget -y

# Download and run Ubuntu setup script
wget https://raw.githubusercontent.com/botforge-robotics/ros2_android/refs/heads/humble/setup_ubuntu.sh
chmod +x setup_ubuntu.sh
./setup_ubuntu.sh
```
#### Step 2: Start Ubuntu (in Termux)
```bash
./start-ubuntu22.sh
```
#### Step 3: Install ROS 2 and RIO (in Ubuntu)
After Ubuntu is installed and you're in the Ubuntu environment:
```bash
# Download and run ROS 2 setup script
wget https://raw.githubusercontent.com/botforge-robotics/ros2_android/refs/heads/humble/setup_ros2.sh
chmod +x setup_ros2.sh
./setup_ros2.sh
```

The installation process will:
1. First script (`setup_ubuntu.sh`):
   - Install required Termux packages
   - Download and setup Ubuntu 22.04
   - Configure basic Ubuntu environment

2. Second script (`setup_ros2.sh`):
   - Install ROS 2 Humble
   - Setup Micro-ROS
   - Install RIO ROS2 packages
   - Configure all environments

## Post-Installation

After successful installation:
1. Restart your terminal
2. Your ROS 2 environment will be automatically sourced
3. Run `ros2 --help` to verify the installation

## Troubleshooting
If you encounter any issues:

1. termux no mirror or mirror group selected
```bash
termux-change-repo
```
2. Make sure you have enough storage space
3. Check your internet connection
4. If the first script fails:
   - Try running `setup_ubuntu.sh` again
5. If the second script fails:
   - Make sure you're in the Ubuntu environment
   - Try running `setup_ros2.sh` again
6. Check the error messages in the script output

## Notes

- The installation process may take some time depending on your device and internet connection
- Make sure your device is plugged into a power source during installation
- Some features may require additional permissions or hardware capabilities

## Contributing

Feel free to submit issues and enhancement requests!

## License

This project is licensed under the MIT License - see the LICENSE file for details.