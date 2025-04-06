# omros2-firmware

## Overview

This is a [OpenMower](https://github.com/ClemensElflein/OpenMower) mainboard firmware replacement to work with [OpenMowerROS2](https://jkaflik.github.io/OpenMowerROS2/) alternative ROS software.
More details about this firmware can be found in [documentation](https://jkaflik.github.io/OpenMowerROS2/omros2-firmware.html).

## Features

- [x] ROS2 node using Micro-ROS
- [x] Charging
- [x] LED status
- [x] IMU
- [ ] Emergency mode 

## Build

### Prepare environment

> [!IMPORTANT]
> This firmware is meant to be run on a [OpenMower](https://github.com/ClemensElflein/OpenMower) hardware or compatible.

Firmware is build against a RP2040 chip only using PlatformIO.
Default target is a remotely run OpenOCD with a Pico board as a SWD target.
Everything should work out of the box. Make sure your `--upload-port` is your Raspberry Pi host. (one used for the OpenMower)

> [!NOTE]
> Only Raspberry Pi5 is supported. OpenOCD configuration is crafted for it. If you want to use another board, see older commits in this repository.

On the hardware side, you need to connect the Raspberry Pi SWD interface to the target SWD interface. Luckily, on the OpenMower mainboard there is are circuits to do so. All you have to do is to use 4 jumpers on J22.

![J22](docs/openmower_j22_swd.png)

When executing build/debug target, along with firmware, [the bash script](utils/remote-openocd.sh) will be deployed to target Raspberry Pi and executed.

### Build (& run)

```bash
platformio run -t upload
```
(**note**: you can use `pio run` as well, so it doesn't upload to target)

## Debugging

Use your IDE integration to debug the code. You can use `pio debug` command as well.
This will run the same bash script as in the upload command, but it will not upload the firmware. You can use `pio debug -t upload` to upload the firmware and start debugging.
