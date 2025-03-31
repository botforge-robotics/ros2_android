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

### 2. Install ROS 2 and RIO

1. Open Termux and run:
```bash
wget https://raw.githubusercontent.com/botforge-robotics/ros2_android/refs/heads/humble/install.sh
chmod +x install.sh
./install.sh
```

The script will:
- Install Ubuntu 22.04 using Andronix
- Set up ROS 2 Humble
- Install Micro-ROS
- Set up RIO ROS2 packages
- Configure all necessary environments


## Troubleshooting

If you encounter any issues:

1. Make sure you have enough storage space
2. Check your internet connection
3. Try running the installation script again
4. Check the error messages in the script output

## Notes

- The installation process may take some time depending on your device and internet connection
- Make sure your device is plugged into a power source during installation
- Some features may require additional permissions or hardware capabilities

## Contributing

Feel free to submit issues and enhancement requests!

## License

This project is licensed under the MIT License - see the LICENSE file for details.