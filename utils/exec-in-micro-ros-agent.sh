#!/bin/bash

# Check if docker or podman is available
if command -v podman &>/dev/null; then
  CONTAINER_ENGINE="podman"
elif command -v docker &>/dev/null; then
  CONTAINER_ENGINE="docker"
else
  echo "Error: Neither podman nor docker is installed. Please install one of them."
  exit 1
fi

echo "Using $CONTAINER_ENGINE"

# Run the container with the specified or default device
sudo $CONTAINER_ENGINE exec -it micro-ros-agent bash
