#!/bin/bash

DEVICE=${1:-/dev/ttyAMA0}
CONTAINER_NAME="micro-ros-agent"

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

if sudo $CONTAINER_ENGINE ps -a | grep -q $CONTAINER_NAME; then
  echo "Container '$CONTAINER_NAME' already exists. Stopping and removing it."
  sudo $CONTAINER_ENGINE stop $CONTAINER_NAME
  sudo $CONTAINER_ENGINE rm $CONTAINER_NAME
fi

# Run the container with the specified or default device
sudo $CONTAINER_ENGINE run --rm -it --name $CONTAINER_NAME \
  --device $DEVICE \
  ghcr.io/jkaflik/omros2-firmware:micro-ros \
  serial --dev $DEVICE -b 115200
