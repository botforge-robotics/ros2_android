#!/usr/bin/env bash
set -e

# Configuration
IMAGE_NAME="rosdroid-rootfs-builder:arm64"
OUTPUT_TAR="rosdroid-ubuntu24-jazzy-arm64.tar.gz"

echo "=========================================================="
echo " Building Rosdroid Ubuntu 24.04 + ROS 2 Jazzy Rootfs"
echo "=========================================================="

# Check for docker
if ! command -v docker >/dev/null 2>&1; then
    echo "Error: docker is required but not installed." >&2
    exit 1
fi

# Ensure QEMU multiarch is registered for cross-building ARM64
echo "Registering QEMU binfmt handlers..."
docker run --privileged --rm tonistiigi/binfmt --install all || true

# Build docker image
echo "Building ARM64 Docker container..."
docker buildx build --platform linux/arm64 -t "$IMAGE_NAME" .

# Create container and export filesystem
echo "Exporting rootfs filesystem..."
CONTAINER_ID=$(docker create --platform linux/arm64 "$IMAGE_NAME")

# Export and compress directly
docker export "$CONTAINER_ID" | gzip -9 > "$OUTPUT_TAR"
docker rm "$CONTAINER_ID"

echo "=========================================================="
echo " Rootfs archive created successfully: $OUTPUT_TAR"
echo " Size: $(du -h "$OUTPUT_TAR" | cut -f1)"
echo "=========================================================="
