#!/bin/bash

# Default device path
DEVICE=${1:-/dev/ttyAMA0}

# Check if docker or podman is available
if command -v podman &>/dev/null; then
  CONTAINER_ENGINE="podman"
elif command -v docker &>/dev/null; then
  CONTAINER_ENGINE="docker"
else
  echo "Error: Neither podman nor docker is installed. Please install one of them."
  exit 1
fi

echo "Using $CONTAINER_ENGINE with device $DEVICE"

# Run the container with the specified or default device
sudo $CONTAINER_ENGINE run --rm -it --name micro-ros-agent \
  --device $DEVICE \
  ghcr.io/jkaflik/omros2-firmware:micro-ros \
  serial --dev $DEVICE -b 115200
