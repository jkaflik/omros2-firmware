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

if ! command -v raspi-config &>/dev/null; then
    echo "Is this a Raspberry Pi? raspi-config could not be found, please install it first"
    exit 1
fi

if ! groups | grep -q "gpio"; then
    echo "User is not in gpio group, please add the user to gpio group and re-login: sudo adduser $USER gpio"
    exit 1
fi

if ! command -v /usr/local/bin/openocd &>/dev/null; then
    echo "openocd could not be found, installing it now..."

    sudo apt-get update >/dev/null
    sudo apt-get install -y libtool libjim-dev

    git clone git@github.com:openocd-org/openocd.git
    cd openocd
    ./bootstrap
    ./configure --disable-werror
    make -j4
    sudo make install
fi

function openocdConfig() {
    cat <<EOF >/tmp/openocd.cfg
adapter driver linuxgpiod

# pintctrl GPIO chip is num 4

adapter gpio swclk -chip 4 25
adapter gpio swdio -chip 4 24

reset_config none

# Target is RP2040 chip on the Pico
source [find target/rp2040.cfg]
EOF
}

killall openocd >/dev/null || true
openocdConfig

# set 23 pin to output high - Pico's RUN
pinctrl set 23 op dh

if [ "$action" == "flash" ]; then
    echo "Flashing firmware from $firmware_path..."
    /usr/local/bin/openocd \
        -f /tmp/openocd.cfg \
        -c "program $firmware_path verify reset exit"

    # It happens OpenOCD doesn't do a proper target reset
    pinctrl set 23 op dl
    sleep 0.1
    pinctrl set 23 op dh
fi

if [ "$action" == "debug" ]; then
    echo "Starting debug session..."

    /usr/local/bin/openocd \
        -f /tmp/openocd.cfg \
        -c "bindto 0.0.0.0"
fi
