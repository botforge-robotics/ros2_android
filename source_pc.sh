#!/usr/bin/env bash
# Source ROS 2 Jazzy environment for communicating with Rosdroid phone
source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID=156
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/chaitu/Projects/ros2_android/fastdds_pc.xml

# Restart ros2 daemon to pick up new DDS network settings
ros2 daemon stop >/dev/null 2>&1 || true

echo "=== Rosdroid PC Environment Configured ==="
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "FASTRTPS_DEFAULT_PROFILES_FILE: $FASTRTPS_DEFAULT_PROFILES_FILE"
echo ""
echo "Test phone connection with:"
echo "  ros2 topic list"
echo "  ros2 topic echo /battery"
echo "=========================================="
