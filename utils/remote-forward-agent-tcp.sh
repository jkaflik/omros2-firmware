#!/bin/bash

# remote-forward-agent-tcp.sh
# This script connects to a remote host via SSH, exposes a serial port over TCP,
# and runs a specified program. When the program terminates, the forwarding is terminated.

# Check if we have at least 3 arguments
if [ $# -lt 3 ]; then
    echo "Usage: $0 <ssh-host> <serial-port> <tcp-port> [program and args...]"
    echo "Example: $0 user@example.com /dev/ttyUSB0 8888 ./my_program arg1 arg2"
    exit 1
fi

SSH_HOST="$1"
SERIAL_PORT="$2"
TCP_PORT="$3"
# Remove the first 3 arguments, so $@ now contains only the program and its args
shift 3

# Function to start the SSH forwarding session
start_ssh_forwarding() {
    echo "Setting up TCP forwarding from $SSH_HOST:$SERIAL_PORT to $SSH_HOST:$TCP_PORT..."
    
    # Start the SSH session with port forwarding in the background
    ssh "$SSH_HOST" \
        "killall socat || true && socat TCP-LISTEN:$TCP_PORT,reuseaddr FILE:$SERIAL_PORT,raw,b115200,cs8,clocal=1,cread=1" &
    
    SSH_PID=$!
    
    # Wait a moment to ensure the connection is established
    sleep 2
    
    # Check if SSH is still running
    if ! kill -0 $SSH_PID 2>/dev/null; then
        echo "Failed to establish SSH connection or TCP forwarding"
        exit 1
    fi
    
    echo "TCP forwarding established with PID $SSH_PID"
}

# Function to clean up
cleanup() {
    echo "Terminating SSH forwarding session..."
    if [ -n "$SSH_PID" ]; then
        kill $SSH_PID 2>/dev/null
        wait $SSH_PID 2>/dev/null
    fi
    echo "Cleanup complete"
}

# Set up trap to ensure cleanup happens
trap cleanup EXIT INT TERM

# Start the SSH forwarding
start_ssh_forwarding

# Run the specified program with its arguments
if [ $# -gt 0 ]; then
    echo "Running command: $@"
    "$@"
    CMD_EXIT_CODE=$?
    echo "Command exited with code $CMD_EXIT_CODE"
else
    echo "No command specified, keeping the forwarding active."
    echo "Press Ctrl+C to terminate."
    # Keep the script running until it receives a signal
    while true; do
        sleep 1
    done
fi