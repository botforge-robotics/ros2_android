<div align="center">

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy_Jalisco-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Platform](https://img.shields.io/badge/platform-Ubuntu_24.04_Noble-orange.svg)](https://ubuntu.com/)
[![Architecture](https://img.shields.io/badge/architecture-ARM64_aarch64-green.svg)](https://arm.com)
[![Mode](https://img.shields.io/badge/mode-Headless_Server-brightgreen.svg)](#-key-features)
[![Release](https://img.shields.io/badge/release-v1.0.0-brightgreen.svg)](https://github.com/botforge-robotics/ros2_android/releases)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](../LICENSE)

# 🤖 Rosdroid: Headless ROS 2 Android Platform

*Transform any standard Android smartphone into a high-performance, cost-effective robotics companion computer—acting as an all-in-one replacement for a Single Board Computer (Raspberry Pi 5) + sensor breakout boards + battery pack + display + LTE/Wi-Fi modem.*
</div>

---

## 📱 App Live Screenshots

<div align="center">

| 🤖 Animated Cybernetic Robot Face | 📊 Real-Time Telemetry & Sensors |
|:---:|:---:|
| <img src="../docs/images/screenshot_face.png" width="340" alt="Robot Face Screen" /> | <img src="../docs/images/screenshot_dashboard.png" width="340" alt="Dashboard Screen" /> |
| **High-visibility neon green expressive eyes** | **Battery telemetry, active Wi-Fi IP & camera controls** |

| 💻 Remote Terminal Access (SSH Server) | 🖥️ In-App Multi-Terminal Console |
|:---:|:---:|
| <img src="../docs/images/screenshot_ssh.png" width="340" alt="SSH Remote Server Card" /> | <img src="../docs/images/screenshot_console.png" width="340" alt="In-App Terminal Console" /> |
| **Dropbear SSH (:2222) with inline commands for rosdroid & root** | **Multi-session terminal with Split View, Ctrl+C & TAB completion** |

</div>

---

## 🌟 Key Features

- **⚡ 1-Click Provisioning**: Automated download and rootless extraction of Ubuntu 24.04 (Noble) + ROS 2 Jazzy + Dropbear SSH in minutes. Zero root required!
- **🚀 Pure Headless Architecture**: All graphical desktop bloat (VNC, X11, Openbox, WebSockets) removed to save ~150-200MB RAM, reduce thermals, prevent background process termination, and optimize battery life.
- **⚡ Ultra-Fast Graph Introspection (< 1s)**: Built-in CNode accelerator executing `ros2 topic list`, `ros2 node list`, and `ros2 service list` in under 1 second (5x faster than standard mobile CLI).
- **📱 Hardware Sensor Hub (Bridged to ROS 2)**:
  - **Vision**: Dual-camera streaming (Front & Back) with hardware JPEG encoding -> `/camera/front/image_raw/compressed` and `/camera/back/image_raw/compressed`.
  - **LED Torch Control**: Bidirectional physical flash control via topic `/torch` (`std_msgs/msg/Bool`).
  - **IMU**: High-rate Accelerometer, Gyroscope, and Magnetometer fused with a native C++ **Madgwick AHRS filter** -> `/imu/data`.
  - **Power & Battery**: Real-time voltage, percentage, and charging telemetry -> `/battery`.
  - **GNSS / GPS**: High-accuracy outdoor positioning -> `/gps`.
  - **Environmental**: Ambient light (`/illuminance`), Proximity/ToF (`/tof`).
  - **Audio & Speech**: Wake word detection (`"Hey Robot"` / `"Robot"` -> `/speech_recognition/hotword_detected`) and speech-to-text transcripts (`/speech_recognition/result`).
- **🔌 Hardware USB Serial micro-ROS Bridge**:
  - Connects directly to ESP32 / STM32 / Teensy motor controllers via USB OTG cable.
  - Automatic USB permission handling via `usb-serial-for-android`.
  - Relays USB serial packets to localhost UDP port `8888` for `micro_ros_agent`.
- **🌐 Remote Terminal Access**:
  - **Dropbear SSH**: Port `2222` (`rosdroid:rosdroid`) and Port `2224` (`root:rosdroid`).
  - **RosBridge Server**: Internal high-speed JSON communication on port `9871`.
- **🤖 Animated Cybernetic Robot Face**:
  - Expressive eyes with blinking, gaze tracking, and dynamic moods (Idle, Happy, Thinking, Speaking, Alert) rendered in high-contrast neon green for outdoor visibility.
- **🖥️ In-App Multi-Terminal Console**:
  - Multi-session tabs (`Term 1`, `Term 2`, etc.) with running process indicators (`●`).
  - **Split View Mode** for simultaneous dual-terminal operation.
  - Dedicated hardware **`Ctrl+C`** interrupt button.
  - **`TAB ⇥`** auto-completion for ROS 2 commands and topic names.
  - Bash command history navigation with **`▲`** and **`▼`** buttons.
  - Automatic log auto-scroll with safe circular buffer management.
  - 1-tap quick action chips (`ros2 topic list`, `ros2 node list`, `echo /battery`, `echo /imu/data`, etc.).

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
2. Grant required permissions (**Camera**, **Location**, **Notification**, **Microphone**).
3. On the **Setup** / **Dashboard** tab, tap **Start 1-Click Installation**.
4. The app extracts the Ubuntu rootfs, installs `patch_getifaddrs.so`, configures DDS, starts Dropbear SSH, and launches the `/rosdroid_robot` ROS 2 node.

---

## 🔑 Remote Access: SSH

Rosdroid runs Dropbear SSH servers inside the container for full command-line control:

| Account | Port | Default Password | Access Level |
|---------|------|------------------|--------------|
| `rosdroid` | **2222** | `rosdroid` | Primary ROS 2 development user |
| `root`     | **2224** | `rosdroid` | Root administrator inside PRoot |

*(A legacy `robot` user alias is also available).*

### Connecting via SSH

```bash
# Connect as rosdroid user
ssh rosdroid@<phone-ip> -p 2222

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

## 📄 License

MIT License. Developed with ❤️ for open robotics by BotForge.
