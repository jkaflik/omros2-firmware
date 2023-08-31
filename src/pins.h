// Contents originally comes from the OpenMower repository: https://github.com/ClemensElflein/OpenMower/blob/main/Firmware/LowLevel/src/pins.h
// This file is stripped down to the recent 0.13.x hardware version

#pragma once

#ifdef HW_0_13_X
#define PIN_IMU_SCK 6
#define PIN_IMU_TX 7
#define PIN_IMU_RX 4
#define PIN_IMU_CS 5

#define PIN_ANALOG_BATTERY_VOLTAGE 27
#define PIN_ANALOG_CHARGE_VOLTAGE 26
#define PIN_ANALOG_CHARGE_CURRENT 28

#define PIN_ENABLE_CHARGE 22

#define PIN_ESC_SHUTDOWN 20
#define PIN_RASPI_POWER 21

#define PIN_EMERGENCY_1 18
#define PIN_EMERGENCY_2 19
#define PIN_EMERGENCY_3 3
#define PIN_EMERGENCY_4 2

#define PIN_MUX_IN 11
#define PIN_MUX_OUT 12
#define PIN_MUX_ADDRESS_0 13
#define PIN_MUX_ADDRESS_1 14
#define PIN_MUX_ADDRESS_2 15

#define PIN_NEOPIXEL 10

#define PIN_UI_TX 8
#define PIN_UI_RX 9

#else
#error No hardware version defined
#endif
