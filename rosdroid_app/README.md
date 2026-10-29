# 🤖 Rosdroid: Headless ROS 2 Android Platform

Transform any standard Android smartphone into a high-performance, cost-effective robotics companion computer—acting as an all-in-one replacement for a Single Board Computer (Raspberry Pi 5) + sensor breakout boards + battery pack + display + LTE/Wi-Fi modem.

Rosdroid runs a complete **headless Ubuntu Server 24.04** environment powered by **PRoot** directly on Android, pre-provisioned with **ROS 2 Jazzy Jalisco**, **Cyclone DDS**, **Dropbear SSH**, and a native hardware sensor hub.

---

## 🌟 Key Features

- **⚡ 1-Click Provisioning**: Automated download and rootless extraction of Ubuntu 24.04 (Noble) + ROS 2 Jazzy + Dropbear SSH in minutes. Zero root required!
- **🚀 Pure Headless Architecture**: All graphical desktop bloat (VNC, X11, Openbox, WebSockets) removed to save ~150-200MB RAM, reduce thermals, prevent background process termination, and optimize battery life.
- **📱 Hardware Sensor Hub (Bridged to ROS 2)**:
  - **Vision**: Dual-camera streaming (Front & Back) with hardware JPEG encoding -> `/camera/front/image_raw/compressed` and `/camera/back/image_raw/compressed`.
  - **LED Torch Control**: Bidirectional physical flash control via topic `/torch` (`std_msgs/msg/Bool`).
  - **IMU**: High-rate Accelerometer, Gyroscope, and Magnetometer fused with a native C++ **Madgwick AHRS filter** -> `/imu/data`.
  - **Power & Battery**: Real-time voltage, percentage, and charging telemetry -> `/battery`.
  - **GNSS / GPS**: High-accuracy outdoor positioning -> `/gps`.
  - **Environmental**: Ambient light (`/illuminance`), Proximity/ToF (`/tof`).
  - **Audio & Speech**: Speech recognition and hotword detection (`/speech_recognition/*`).
- **🔌 Hardware USB Serial micro-ROS Bridge**:
  - Connects directly to ESP32 / STM32 / Teensy motor controllers via USB OTG cable.
  - Automatic USB permission handling via `usb-serial-for-android`.
  - Relays USB serial packets to localhost UDP port `8888` for `micro_ros_agent`.
- **🌐 Remote Terminal Access**:
  - **Dropbear SSH**: Port `2222` (`robot:rosdroid`) and Port `2224` (`root:rosdroid`).
  - **RosBridge Server**: Internal high-speed JSON communication on port `9871`.
- **🤖 Animated Cybernetic Robot Face**:
  - Expressive eyes with blinking, gaze tracking, and dynamic moods (Idle, Happy, Thinking, Speaking, Alert) rendered in high-contrast neon green for outdoor visibility.
- **🖥️ In-App Live Terminal**:
  - Interactive shell inside the PRoot container with 1-tap quick action chips (`ros2 topic list`, `ros2 node list`, `echo /battery`, `echo /imu/data`, `source jazzy`, etc.).

---

## 🚀 Quick Start & Installation

### 1. Install & Launch the App
```bash
cd rosdroid_app
./gradlew assembleDebug
adb install -r app/build/outputs/apk/debug/app-debug.apk
```

### 2. Permissions & 1-Click Environment Setup
1. Launch **Rosdroid** on your smartphone.
2. Grant required permissions (**Camera**, **Location**, **Notification**).
3. On the **Setup** / **Dashboard** tab, tap **Start 1-Click Installation**.
4. The app extracts the Ubuntu rootfs, installs `patch_getifaddrs.so`, configures DDS, starts Dropbear SSH, and launches the `/rosdroid_robot` ROS 2 node.

---

## 🔑 Remote Access: SSH

Rosdroid runs Dropbear SSH servers inside the container for full command-line control:

| Account | Port | Default Password | Access Level |
|---------|------|------------------|--------------|
| `robot` | **2222** | `rosdroid` | Standard ROS 2 development user |
| `root`  | **2224** | `rosdroid` | Root administrator inside PRoot |

### Connecting via SSH

```bash
# Connect as standard robot user
ssh robot@<phone-ip> -p 2222

# Or connect as root
ssh root@<phone-ip> -p 2224
```
*(Find your phone's IP address displayed at the top of the app's Dashboard screen, e.g., `192.168.0.152`).*

### ROS 2 Inside the Phone
The guest environment automatically sources ROS 2 Jazzy. If opening a fresh non-login subshell:
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=156
```

Verify your active nodes:
```bash
ros2 node list
# Outputs: /rosdroid_robot
```

---

## 🌐 ROS 2 Communication from Remote PC

By default, Rosdroid operates on **`ROS_DOMAIN_ID=156`** using **ROS 2 Jazzy**.

### 1. 1-Step Setup on PC (Recommended)
In the repository root on your Linux PC, run:
```bash
source source_pc.sh
```

This script automatically:
- Sources `/opt/ros/jazzy/setup.bash`
- Sets `export ROS_DOMAIN_ID=156`
- Configures `FASTRTPS_DEFAULT_PROFILES_FILE=fastdds_pc.xml` (restricting DDS discovery to your active Wi-Fi interface and adding the phone's static IP and subnet broadcast locators)
- Resets the `ros2 daemon`

### 2. Verify Topics from PC
```bash
ros2 topic list
```
You will immediately see all active phone topics:
```
/battery
/camera/back/image_raw/compressed
/camera/front/image_raw/compressed
/cmd_vel
/expression
/gps
/illuminance
/imu/data
/parameter_events
/rosout
/speech_recognition/hotword_detected
/speech_recognition/result
/speech_recognition/status
/tof
/torch
```

### 3. Read Live Telemetry
```bash
# Read real-time battery status
ros2 topic echo /battery --once

# Read fused IMU orientation (quaternion + angular rates + linear acceleration)
ros2 topic echo /imu/data --once

# View compressed camera stream
ros2 run image_view image_view --ros-args -r image:=/camera/back/image_raw/compressed -p image_transport:=compressed
```

### 4. Control Physical Phone Hardware Over ROS 2
```bash
# Turn ON physical camera LED torch
ros2 topic pub -w 0 -1 /torch std_msgs/msg/Bool "data: true"

# Turn OFF physical camera LED torch
ros2 topic pub -w 0 -1 /torch std_msgs/msg/Bool "data: false"

# Change robot face expression
ros2 topic pub -w 0 -1 /expression std_msgs/msg/String "data: 'happy'"

# Send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```

---

## 🔌 Hardware USB Serial micro-ROS Bridge

To connect motor drivers, sensors, or robot bases (ESP32, Teensy, STM32, Arduino):

1. Connect the microcontroller board to the phone's USB Type-C port using an OTG adapter.
2. In the Android pop-up dialog, select **Always allow** and tap **OK**.
3. Rosdroid's `UsbSerialBridge` automatically captures the USB CDC/ACM serial stream and forwards it asynchronously to localhost UDP port `8888`.
4. In the phone terminal (via SSH or the app's Console tab), start the Micro-ROS Agent:
   ```bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   ```

---

## 🎭 Interactive App Screens

- **Face Tab**: Displays animated expressive robot eyes with blinking, saccades, and mood state transitions. The eye colors are rendered in high-contrast neon green for high outdoor and indoor visibility.
- **Dashboard Tab**:
  - Real-time IP address and battery percentage.
  - Sensor stream toggle switches (Battery, IMU, GPS, Cameras).
  - Physical Torch LED switch.
  - 1-tap copy buttons for Dropbear SSH credentials.
- **Console Tab**: Embedded interactive PRoot terminal. Tap preset chips like `ros2 topic list`, `ros2 node list`, or type commands directly on your phone.

---

## 🛠️ Troubleshooting & FAQs

### Topics aren't showing up on PC
1. Ensure both your PC and phone are on the exact same Wi-Fi network (or phone Wi-Fi hotspot).
2. Check that your PC has `ROS_DOMAIN_ID=156`.
3. If using Fast-DDS on PC with Docker installed, make sure to `source source_pc.sh` so DDS does not route traffic through Docker bridges.
4. If topics don't refresh immediately, restart the ROS 2 daemon on your PC:
   ```bash
   ros2 daemon stop
   ros2 topic list --no-daemon
   ```

### Permission Denied on `getifaddrs()`
Android PRoot cannot query network interfaces via standard netlink sockets. Rosdroid includes a custom preload library (`patch_getifaddrs.so`) that intercepts `getifaddrs()` and maps the phone's IP and `wlan0` interface directly. This is pre-loaded via `LD_PRELOAD=/usr/local/lib/patch_getifaddrs.so`.

### Torch doesn't turn on
The phone's back camera and torch share the same physical camera sensor module. Rosdroid controls the torch through the active CameraX session lock, so ensure the app has Camera permission granted.

---

## 📄 License
MIT License. Built with ❤️ for open robotics by BotForge.
