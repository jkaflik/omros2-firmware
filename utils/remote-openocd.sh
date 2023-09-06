#!/usr/bin/env bash

set -e

# this script is used as openocd entrypoint on a remote machine
# usage: remote-openocd.sh <action>
# example: remote-openocd.sh flash <firmware_path>
# example: remote-openocd.sh debug

if [ $# -lt 1 ]; then
    echo "usage: remote-openocd.sh <action>"
    exit 1
fi

action=$1

if [ "$action" == "flash" ]; then
    if [ $# -lt 2 ]; then
        echo "usage: remote-openocd.sh flash <firmware_path>"
        exit 1
    fi
    firmware_path=$2
fi

if [ -z "$(whereis raspinfo)" ]; then
    echo "This script runs only on Raspberry Pi"
    exit 1
fi

if [ -z "$(whereis openocd)" ]; then
    sudo apt install -y openocd
fi

function restartPico() {
    if ! groups | grep -q "gpio"; then
        echo "User is not in gpio group, please add the user to gpio group and re-login: sudo adduser $USER gpio"
        exit 1
    fi

    echo "23" > /sys/class/gpio/export || true
    sleep 1
    echo "out" > /sys/class/gpio/gpio23/direction

    echo "Restarting Pico..."
    echo "1" > /sys/class/gpio/gpio23/value
    echo "Pico restarted"
}

function cleanupRestartPico() {
    echo "0" > /sys/class/gpio/gpio23/value
    echo "23" > /sys/class/gpio/unexport
    echo "Pico restart finished"
}

if [ "$action" == "flash" ]; then
  echo "Flashing firmware from $firmware_path..."

  restartPico
  openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program $firmware_path verify reset exit"
  cleanupRestartPico
fi

if [ "$action" == "debug" ]; then
  echo "Starting debug session..."

  restartPico
  openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "bindto 0.0.0.0" || cleanupRestartPico
  cleanupRestartPico
fi
