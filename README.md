
<div align="center">
<img src="https://img.shields.io/badge/ROS2-Humble-blue" alt="ROS2"/>
<img src="https://img.shields.io/badge/license-MIT-blue" alt="License"/>
<img src="https://img.shields.io/badge/platform-Ubuntu%2022.04-orange" alt="Platform"/>

# 🤖 ROS 2 Humble on Android (Termux)


*Run ROS 2 Humble, Micro-ROS, Ollama and RIO ROS2 packages on your Android device using Termux and ROS2Sense Mobile App for sensor hub*

</div>

---

## 📋 Prerequisites

- 📱 Android device running Android 7.0 or higher
- 💾 At least 16GB of free storage space
- 🌐 Internet connection
- 📱 Termux app (latest version)
- 📱 ROS2Sense Mobile App (latest version)

---

## 🚀 Installation Steps

### 📥 1. Install Termux App

1. Download the latest Termux app from the official releases:
   <details>
   <summary>Download Links</summary>

   - For Android 7+ devices: [termux-app_v0.119.0-beta.2+apt-android-7-github-debug_universal.apk](https://github.com/termux/termux-app/releases/download/v0.119.0-beta.2/termux-app_v0.119.0-beta.2+apt-android-7-github-debug_universal.apk)
   - For Android 5/6 devices: [termux-app_v0.119.0-beta.2+apt-android-5-github-debug_universal.apk](https://github.com/termux/termux-app/releases/download/v0.119.0-beta.2/termux-app_v0.119.0-beta.2+apt-android-5-github-debug_universal.apk)
   </details>

2. Install the downloaded APK on your device
3. Open Termux app and grant necessary permissions

### ⚙️ 2. Installation Process (Two-Step)

<details>
<summary><b>Step 1: Setup Ubuntu (in Termux)</b></summary>

```bash
# Update Termux and get required packages
pkg update -y && pkg upgrade -y && pkg install wget -y

# Download and run Ubuntu setup script
wget https://raw.githubusercontent.com/botforge-robotics/ros2_android/refs/heads/humble/setup_ubuntu.sh
chmod +x setup_ubuntu.sh
./setup_ubuntu.sh
```
</details>

<details>
<summary><b>Step 2: Install ROS 2 and RIO (in Ubuntu)</b></summary>

After Ubuntu is installed:
1. Start Ubuntu environment:
```bash
./start-ubuntu22.sh
```

2. In the Ubuntu environment, run:
```bash
wget https://raw.githubusercontent.com/botforge-robotics/ros2_android/refs/heads/humble/setup_ros2.sh
chmod +x setup_ros2.sh
sudo ./setup_ros2.sh
```
</details>

The installation process will:
- 📦 Install Ubuntu 22.04 using Andronix
- 🔧 Set up ROS 2 Humble
- 🤖 Install Micro-ROS
- 🚀 Set up RIO ROS2 packages
- ⚙️ Configure all environments

## 📝 Post-Installation

After successful installation:
1. 🔄 Restart your Termux app
2. 🔧 Your ROS 2 environment will be automatically sourced 
3. ✅ Run `ros2 --help` to verify the installation


---



## 📱 Using ROS2Sense Mobile App

ROS2Sense transforms your smartphone into a powerful ROS 2 sensor hub, providing:

### Available Sensors 🎯
- 📡 Accelerometer – Detect motion and orientation
- 🔄 Gyroscope – Measure angular velocity
- 🧭 Magnetometer – Get compass direction
- 📍 GPS – Track precise location for navigation
- 📸 Cameras – Use mobile cameras for vision-based applications
- 🌫 IR Sensor – Use infrared commands to control home appliances like ACs and TVs (Comming Soon)
- 💡 Ambient Light Sensor – Detect environment light
- 🔊 Microphone & Speaker – Enable voice interactions & Text to speech
- 🆔 Fingerprint/Face ID – Biometric security and authentication for robots
- 📲 Display Output – Use your mobile screen for animated robotic expressions
- Inbuilt Madgwick Filter for IMU data fusion, Hotword Detection.


### Setup Instructions 🔧
1. Download ROS2Sense from [Google Play Store](https://play.google.com/store/apps/details?id=com.botforge.rio)
2. Enter 0.0.0.0 as ROS_IP as ros2 is running on the same device.
   
---

### 🌐 ROS Domain Setup for Remote Communication

To enable communication between your mobile device (running ROS 2 in Termux/Ubuntu) and a remote PC running ROS 2:

1. **On your mobile device (Termux/Ubuntu)** 📱
   ```bash
   export ROS_DOMAIN_ID=0  # Choose any number between 0-232
   export ROS_LOCALHOST_ONLY=0  # Enable non-localhost communication
   ```

2. **On your remote PC** 💻
   ```bash
   export ROS_DOMAIN_ID=0  # Use the same number as mobile device
   export ROS_LOCALHOST_ONLY=0
   ```

> 🔒 **Note:** For security, use unique domain IDs when multiple ROS 2 systems are on the same network to prevent unintended cross-communication.

---

### 🔑 SSH Login (Termux)

To remotely access your Termux environment via SSH from another device (like your computer), follow these steps:

1. **Get your Termux username** 👤
   ```bash
   whoami
   ```
   This will display your Termux username that you'll need for SSH login.

2. **Find your device's IP address** 📍
   ```bash
   ifconfig
   ```
   Look for the `wlan0` section and find the `inet` address - this is your device's IP address on the WiFi network.
   For example: `inet 192.168.1.100`

3. **Connect via SSH from another device** 🔌
   ```bash
   ssh <username>@<ip_address> -p 8022
   ```
   Example: `ssh u0_a123@192.168.1.100 -p 8022`
   
   > **Note:** Port 8022 is Termux's default SSH port, different from the standard port 22.

4. **Enter your password** 🔐
   - You'll be prompted for the password you set during the Ubuntu installation
   - Type your password (it won't be visible as you type)
   - Press Enter to submit

5. **Success!** ✨
   You should now see the Termux prompt, indicating a successful SSH connection.

6. **Start Ubuntu** 🚀
   ```bash
   ./start-ubuntu22.sh
   ```
   This launches your Ubuntu environment within the SSH session.

---
## 🔗 Related Repositories

- [RIO Hardware](https://github.com/botforge-robotics/rio_hardware) - Hardware design files, BOM and assembly instructions
- [RIO Firmware](https://github.com/botforge-robotics/rio_firmware) - Micro-ROS firmware for the RIO controller board
- [RIO ROS2](https://github.com/botforge-robotics/rio_ros2) - ROS2 packages for navigation, control, and sensor integration

---
## ❗ Troubleshooting

If you encounter any issues:

1. **Mirror Selection Issue** 🔄
   ```bash
   termux-change-repo
   ```

2. 🌐 Check your internet connection

3. 🔄 If the first script fails:
   - Try running `setup_ubuntu.sh` again

4. 🔧 If the second script fails:
   - Make sure you're in the Ubuntu environment
   - Try running `setup_ros2.sh` again

5. 📝 Check the error messages in the script output


6. **ROS2Sense App Connection Issues** 📱
   If you're using the ROS2Sense mobile app:
   - Ensure both devices (phone and computer) are on the same WiFi network
   - Verify the ROS_DOMAIN_ID matches between devices
   - Check if the correct ROS_IP/ROS_HOSTNAME is set
   - Try restarting the ROS2 daemon:
     ```bash
     ros2 daemon stop
     ros2 daemon start
     ```

7. **Permission Issues** 🔒
   If you encounter permission errors:
   ```bash
   chmod +x setup_ubuntu.sh
   chmod +x setup_ros2.sh
   ```



## 📌 Notes

- ⏳ The installation process may take some time depending on your device and internet connection
- 🔋 Make sure your device is plugged into a power source during installation

## 🤝 Contributing

Feel free to submit issues and enhancement requests!

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

<div align="center">
<b>Made with ❤️ by BotForge Robotics</b>
</div>