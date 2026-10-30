# Developer Architecture & Engineering Guide 🛠️

This guide provides deep technical documentation for developers, robotics engineers, and contributors working on the **Rosdroid** codebase.

---

## 📑 Table of Contents

- [System Architecture](#system-architecture)
- [Module Breakdown](#module-breakdown)
  - [1. Native Android Application Layer](#1-native-android-application-layer)
  - [2. Native C++ & JNI Layer](#2-native-c--jni-layer)
  - [3. PRoot Guest Container Layer](#3-proot-guest-container-layer)
  - [4. ROS 2 Jazzy & DDS Subsystem](#4-ros-2-jazzy--dds-subsystem)
- [Network Interface Hook (`patch_getifaddrs.c`)](#network-interface-hook-patch_getifaddrsc)
- [USB OTG Micro-ROS Bridge Subsystem](#usb-otg-micro-ros-bridge-subsystem)
- [Building & Packaging the Rootfs](#building--packaging-the-rootfs)
- [Compiling and Running the Project](#compiling-and-running-the-project)
- [Debugging Tips](#debugging-tips)

---

## System Architecture

Rosdroid uses a four-tier architecture designed to run real-time robotics without root access on consumer Android devices:

```
┌────────────────────────────────────────────────────────────────────────┐
│                      Android OS (Kernel / SELinux)                     │
└──────────────────────────────────┬─────────────────────────────────────┘
                                   │
┌──────────────────────────────────▼─────────────────────────────────────┐
│ 1. Android Application Layer (Kotlin / Jetpack Compose)                │
│    • Foreground Service: RobotCoreService (Wakelock + High-Perf Wi-Fi) │
│    • Hardware SensorHub (Accelerometer, Gyro, Mag, Barometer, Light)   │
│    • CameraX Pipeline (Hardware NV21 -> JPEG compression)              │
│    • UsbSerialBridge (CDC/ACM USB-OTG relay to UDP 8888)               │
│    • RosBridge TCP Socket Server (localhost:9871)                      │
└──────────────────┬─────────────────────────────────────────────────────┘
                   │ JNI High-Rate Buffer Transfer
┌──────────────────▼─────────────────────────────────────────────────────┐
│ 2. Native C++ Layer (CMake / NDK / arm64-v8a)                          │
│    • Madgwick AHRS Filter (Fused Quaternion Orientation)               │
│    • Network Shim: patch_getifaddrs.so (Netlink / ifaddrs override)    │
│    • Native PRoot & Dropbear binaries (libproot.so, libdropbear.so)   │
└──────────────────┬─────────────────────────────────────────────────────┘
                   │ PTRACE / PRoot Sandboxing (Zero Root Required)
┌──────────────────▼─────────────────────────────────────────────────────┐
│ 3. Guest User Space (Headless Ubuntu Server 24.04 LTS Noble)           │
│    • Dropbear Multi-Port SSH (Port 2222: robot, Port 2224: root)       │
│    • /etc/profile.d/ros.sh (Jazzy environment + CycloneDDS settings)  │
│    • micro_ros_agent (UDP4 port 8888)                                  │
└──────────────────┬─────────────────────────────────────────────────────┘
                   │ Cyclone DDS Multicast & Direct Peer Unicast
┌──────────────────▼─────────────────────────────────────────────────────┐
│ 4. ROS 2 Jazzy Core Node (/rosdroid_robot)                             │
│    • Sensor Publishers: /imu/data, /battery, /camera/*, /gps, /tof     │
│    • Actuator Subscribers: /torch, /cmd_vel, /expression               │
└────────────────────────────────────────────────────────────────────────┘
```

---

## Module Breakdown

### 1. Native Android Application Layer
- **`RobotCoreService.kt`**: Core Android `ForegroundService` that orchestrates PRoot processes, SensorHub streams, and camera sessions. Holds a `PARTIAL_WAKE_LOCK` and `WIFI_MODE_FULL_HIGH_PERF` Wi-Fi lock to ensure the OS does not throttle CPU or networking when the screen sleeps.
- **`SensorHub.kt`**: Manages Android `SensorManager` listeners for Accelerometer, Gyroscope, Magnetometer, Ambient Light, Barometer, and Battery State. Dispatches high-rate samples into the C++ Madgwick filter.
- **`CameraStreamer.kt`**: Uses Android `CameraX` to capture camera frames and compress them into hardware JPEG payloads directly pushed to the ROS 2 node. Also controls the physical LED flash torch through CameraX's `cameraControl.enableTorch()`.
- **`RosBridgeServer.kt`**: High-performance local TCP socket server on port `9871` bridging Android native telemetry into the PRoot ROS 2 node using newline-delimited JSON messages.
- **`UsbSerialBridge.kt`**: Implements `usb-serial-for-android` to handle hot-plugged microcontrollers over USB-C OTG cables and forwards bidirectional packets to UDP localhost port `8888`.

### 2. Native C++ & JNI Layer
- **`madgwick.cpp` / `madgwick.h`**: 6-DOF / 9-DOF gradient descent orientation filter implemented in standard C++ with zero heap allocations in the hot path. Computes orientation quaternions at up to 200 Hz with minimal battery consumption.
- **`native_bridge.cpp`**: JNI boundary exposing `updateIMU()` and `getQuaternion()` to Kotlin.
- **`CMakeLists.txt`**: Builds `librosdroid_native.so` targeting `arm64-v8a`.

### 3. PRoot Guest Container Layer
- **`PRootManager.kt`**: Executes the sandboxed guest container using `libproot.so`. Binds `/dev`, `/proc`, `/sys`, and sets up Android-compatible environment variables (`TMPDIR`, `LD_PRELOAD`, `PATH`).
- **`RemoteAccessManager.kt`**: Manages Dropbear SSH daemons on ports `2222` (`rosdroid`) and `2224` (`root`). Generates host keys and verifies process health.

### 4. ROS 2 Jazzy & DDS Subsystem
- **Core Node**: `/rosdroid_robot` running inside PRoot.
- **DDS Middleware**: Eclipse Cyclone DDS (`rmw_cyclonedds_cpp`).
- **Daemon Configuration**: `use_daemon = False` is enforced in `ros2cli` strategies to prevent daemon crashes in PRoot environments.

---

## Network Interface Hook (`patch_getifaddrs.c`)

### The Problem
Android restricts access to `/proc/net/` and netlink routing sockets for non-root apps via strict SELinux policies. When Cyclone DDS calls `getifaddrs()`, glibc attempts a netlink socket query, resulting in `getifaddrs(): Permission denied` or an empty interface list.

### The Solution
`patch_getifaddrs.so` intercepts `getifaddrs()` calls via `dlsym(RTLD_NEXT, "getifaddrs")` and `LD_PRELOAD`.
1. It reads `/etc/cyclonedds/ifaddrs.txt` (written dynamically by the Android app with the current Wi-Fi IP).
2. It builds synthetic `struct ifaddrs` linked lists with the `wlan0` and `lo` network interfaces.
3. It intercepts `open()`, `stat()`, and `lstat()` calls that return `ENOSYS` in PRoot sandboxes, converting them to `ENOENT` to prevent glibc/Python crashes.

---

## USB OTG Micro-ROS Bridge Subsystem

The USB bridge enables zero-driver communication with embedded motor controllers:
1. When an ESP32 or STM32 board is plugged into the phone, Android detects the USB device via `device_filter.xml`.
2. `UsbSerialBridge` opens the port at 115200 baud (or configured baud rate).
3. Packets read from USB are forwarded to `127.0.0.1:8888` via UDP.
4. Packets sent by `micro_ros_agent udp4 --port 8888` are written back to the USB serial device.

---

## Building & Packaging the Rootfs

To regenerate the headless Ubuntu 24.04 rootfs:
```bash
cd rosdroid_app/rootfs_builder
./build_rootfs.sh
```
This runs a multi-stage Docker build that:
1. Bootstraps an `arm64` Ubuntu 24.04 Noble base.
2. Installs ROS 2 Jazzy (`ros-jazzy-ros-base`, `ros-jazzy-rmw-cyclonedds-cpp`, `ros-jazzy-micro-ros-agent`).
3. Compiles `patch_getifaddrs.so` and configures Dropbear SSH.
4. Exports a compressed `ubuntu-jazzy-headless-arm64.tar.gz`.

---

## Compiling and Running the Project

```bash
# Clone the repo
git clone https://github.com/botforge-robotics/ros2_android.git
cd ros2_android/rosdroid_app

# Compile C++ and Kotlin with Gradle
./gradlew assembleDebug

# Deploy to connected device
adb install -r app/build/outputs/apk/debug/app-debug.apk
```

---

## Debugging Tips

### 1. View App Logs
```bash
adb logcat -s Rosdroid:V RobotCoreService:V SensorHub:V PRootManager:V
```

### 2. Enter Phone Shell via ADB (Inside PRoot)
```bash
adb shell "run-as org.botforge.rosdroid /data/data/org.botforge.rosdroid/lib/libproot.so \
  -0 -r /data/data/org.botforge.rosdroid/files/rootfs/ubuntu -b /dev -b /proc -b /sys \
  /bin/bash -l"
```

### 3. Connect via SSH
```bash
ssh rosdroid@<phone-ip> -p 2222
# Password: rosdroid
```
