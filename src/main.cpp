#include <std_msgs/msg/bool.h>

#include <Arduino.h>
#include <EMA.h>
#include <micro_ros_platformio.h>
#include <omros2_firmware_msgs/msg/emergency_status.h>
#include <omros2_firmware_msgs/msg/power_status.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/imu.h>

#include <algorithm>

#include "emergency.h"
#include "hardware.h"
#include "imu.h"
#include "led_status.hpp"
#include "pins.h"
#include "ros.h"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error Only Micro-ROS serial transport is supported
#endif

#define CHARGING_UPDATE_INTERVAL 50

extern "C" int clock_gettime(clockid_t unused, struct timespec* tp);

EMA<2> batteryVoltageEMA;
EMA<2> chargeCurrentEMA;
EMA<2> chargeVoltageEMA;

float batteryVoltage = 0.0f;
float chargeCurrent = 0.0f;
float chargeVoltage = 0.0f;
bool isChargerPresent = false;
float batteryPercentage = 0.0f;
uint8_t powerSupplyStatus = 0;
uint8_t powerSupplyHealth = 0;
bool batteryPresent = false;
bool chargeEnabled = false;
bool chargingAllowed = false;
bool batteryFullLatched = false;
uint8_t chargingState = omros2_firmware_msgs__msg__PowerStatus__CHARGING_STATE_DISCHARGING;
uint8_t chargeInhibitReason = omros2_firmware_msgs__msg__PowerStatus__INHIBIT_NONE;
uint32_t chargeRetryRemainingMs = 0;
ulong chargingDisabledTime = 0;
ulong lastChargingLoop = 0;

bool shouldStartRetryDelay(uint8_t reason)
{
  return reason == omros2_firmware_msgs__msg__PowerStatus__INHIBIT_BATTERY_OVERVOLTAGE
         || reason == omros2_firmware_msgs__msg__PowerStatus__INHIBIT_CHARGE_OVERVOLTAGE
         || reason == omros2_firmware_msgs__msg__PowerStatus__INHIBIT_OVERCURRENT;
}

uint8_t getChargingInhibitReason()
{
  if (!batteryPresent)
  {
    return omros2_firmware_msgs__msg__PowerStatus__INHIBIT_NO_BATTERY;
  }

  if (batteryVoltage > BATT_ABSOLUTE_MAX)
  {
    return omros2_firmware_msgs__msg__PowerStatus__INHIBIT_BATTERY_OVERVOLTAGE;
  }

  if (chargeVoltage > CHARGE_VOLTAGE_CUTOFF)
  {
    return omros2_firmware_msgs__msg__PowerStatus__INHIBIT_CHARGE_OVERVOLTAGE;
  }

  if (chargeCurrent >= CHARGE_CURRENT_CUTOFF)
  {
    return omros2_firmware_msgs__msg__PowerStatus__INHIBIT_OVERCURRENT;
  }

  if (batteryFullLatched)
  {
    return omros2_firmware_msgs__msg__PowerStatus__INHIBIT_BATTERY_FULL;
  }

  return omros2_firmware_msgs__msg__PowerStatus__INHIBIT_NONE;
}

void setChargeEnabled(bool enabled)
{
  chargeEnabled = enabled;
  digitalWrite(PIN_ENABLE_CHARGE, chargeEnabled ? HIGH : LOW);
}

void manageBatteryCharging()
{
  uint16_t readFiltered;

  readFiltered = batteryVoltageEMA(analogRead(PIN_ANALOG_BATTERY_VOLTAGE));
  batteryVoltage = (float)readFiltered * (3.3f / 4096.0f) * ((VIN_R1 + VIN_R2) / VIN_R2);

  readFiltered = chargeCurrentEMA(analogRead(PIN_ANALOG_CHARGE_CURRENT));
  chargeCurrent = (float)readFiltered * (3.3f / 4096.0f) / (CURRENT_SENSE_GAIN * R_SHUNT);

  batteryPresent = batteryVoltage > BATT_PRESENT_VOLTAGE;

  readFiltered = chargeVoltageEMA(analogRead(PIN_ANALOG_CHARGE_VOLTAGE));
  chargeVoltage = (float)readFiltered * (3.3f / 4096.0f) * ((VIN_R1 + VIN_R2) / VIN_R2);

  isChargerPresent = chargeVoltage >= CHARGER_PRESENT_VOLTAGE;

  batteryPercentage = std::min(
    std::max((batteryVoltage - BATT_EMPTY) / (BATT_FULL - BATT_EMPTY) * 100, 0.0f), 100.0f);

  if (batteryVoltage >= BATT_FULL)
  {
    batteryFullLatched = true;
  }
  else if (batteryVoltage <= BATT_FULL_HYSTERESIS)
  {
    batteryFullLatched = false;
  }

  const auto now = millis();
  const uint8_t previousInhibitReason = chargeInhibitReason;

  chargeRetryRemainingMs = 0;
  chargeInhibitReason = omros2_firmware_msgs__msg__PowerStatus__INHIBIT_NONE;

  if (chargeVoltage < CHARGE_REGEN_VOLTAGE)
  {
    setChargeEnabled(true);
    chargingAllowed = false;
    chargingDisabledTime = 0;
    chargingState = omros2_firmware_msgs__msg__PowerStatus__CHARGING_STATE_REGEN_PATH;
  }
  else
  {
    const uint8_t limitReason = getChargingInhibitReason();
    if (limitReason != omros2_firmware_msgs__msg__PowerStatus__INHIBIT_NONE)
    {
      if (shouldStartRetryDelay(limitReason)
          && (chargingAllowed || chargeEnabled || previousInhibitReason != limitReason))
      {
        chargingDisabledTime = now;
      }
      else if (!shouldStartRetryDelay(limitReason))
      {
        chargingDisabledTime = 0;
      }

      setChargeEnabled(false);
      chargingAllowed = false;
      chargeInhibitReason = limitReason;
      chargingState = omros2_firmware_msgs__msg__PowerStatus__CHARGING_STATE_NOT_CHARGING;
    }
    else if (chargingDisabledTime != 0 && now - chargingDisabledTime < CHARGING_RETRY_MILLIS)
    {
      setChargeEnabled(false);
      chargingAllowed = false;
      chargeInhibitReason = omros2_firmware_msgs__msg__PowerStatus__INHIBIT_RETRY_DELAY;
      chargeRetryRemainingMs = CHARGING_RETRY_MILLIS - (now - chargingDisabledTime);
      chargingState = omros2_firmware_msgs__msg__PowerStatus__CHARGING_STATE_RETRY_WAIT;
    }
    else
    {
      setChargeEnabled(true);
      chargingAllowed = true;
      chargingDisabledTime = 0;
      chargingState = omros2_firmware_msgs__msg__PowerStatus__CHARGING_STATE_NOT_CHARGING;
      if (chargeVoltage > batteryVoltage)
      {
        chargingState = omros2_firmware_msgs__msg__PowerStatus__CHARGING_STATE_CHARGING;
      }
    }
  }

  if (chargingAllowed)
  {
    powerSupplyStatus = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_NOT_CHARGING;
    if (chargeVoltage > batteryVoltage)
    {
      powerSupplyStatus = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_CHARGING;
    }
  }
  else
  {
    powerSupplyStatus = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;
  }

  if (batteryPresent)
  {
    if (batteryVoltage < BATT_ABSOLUTE_MIN)
    {
      powerSupplyHealth = sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_DEAD;
    }
    else if (batteryVoltage > BATT_ABSOLUTE_MAX)
    {
      powerSupplyHealth = sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_OVERVOLTAGE;
    }
    else
    {
      powerSupplyHealth = sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_GOOD;
    }
  }
  else
  {
    powerSupplyHealth = sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNKNOWN;
  }
}

void chargingLoop()
{
  auto now = millis();
  if (now - lastChargingLoop < CHARGING_UPDATE_INTERVAL)
  {
    return;
  }

  manageBatteryCharging();

  ledStatus.setFlag(LED_STATUS_CHARGING, chargeEnabled);
  ledStatus.setFlag(LED_STATUS_DISCHARGING, !isChargerPresent);
  ledStatus.setFlag(LED_STATUS_BATTERY_LOW, batteryPercentage < 10.0f);

  lastChargingLoop = now;
}

bool isFirstCoreSetupDone;
bool isSecondCoreSetupDone;

uros::Support* support;

void setup1()
{
  while (!isFirstCoreSetupDone)
  {
    delay(100);
  }

  ledStatus.setColor(0, 0, 255, false);
  delay(1000);

  Serial1.begin(115200);
  set_microros_serial_transports(Serial1);

  ledStatus.setColor(0, 0, 255, true);
  delay(2000);

  support = new uros::Support();
  auto node = new uros::Node(*support, "openmower_mainboard");
  auto imuPublisher = new uros::Publisher<sensor_msgs__msg__Imu>(
    *node, "imu/data_raw", UROS_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu));
  auto imuTimer = new uros::Timer(*node,
                                  1000 / 50,  // 50 Hz
                                  [imuPublisher]()
                                  {
                                    auto& msg = imuPublisher->get_message();

                                    // todo: we should read raw,
                                    if (!imuRead(&msg))
                                    {
                                      return;
                                    }

                                    auto ns = rmw_uros_epoch_nanos();
                                    msg.header.stamp.sec = ns / 1000000000;
                                    msg.header.stamp.nanosec = ns % 1000000000;
                                    msg.header.frame_id.capacity = 3;
                                    msg.header.frame_id.data = "imu";
                                    msg.header.frame_id.size = 3;

                                    imuPublisher->publish();
                                  });

  auto batteryStatePublisher = new uros::Publisher<sensor_msgs__msg__BatteryState>(
    *node, "power", UROS_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState));
  auto powerStatusPublisher = new uros::Publisher<omros2_firmware_msgs__msg__PowerStatus>(
    *node, "power/status", UROS_GET_MSG_TYPE_SUPPORT(omros2_firmware_msgs, msg, PowerStatus));
  auto powerTimer = new uros::Timer(
    *node,
    1000,
    [batteryStatePublisher, powerStatusPublisher]()
    {
      auto& batteryMsg = batteryStatePublisher->get_message();

      auto ns = rmw_uros_epoch_nanos();
      batteryMsg.header.stamp.sec = ns / 1000000000;
      batteryMsg.header.stamp.nanosec = ns % 1000000000;
      batteryMsg.voltage = batteryVoltage;
      batteryMsg.current = chargeCurrent;
      batteryMsg.capacity = BATT_DESIGNED_CAPACITY;
      batteryMsg.design_capacity = BATT_DESIGNED_CAPACITY;
      batteryMsg.percentage = batteryPercentage / 100.0f;
      batteryMsg.present = batteryPresent;
      batteryMsg.power_supply_status = powerSupplyStatus;
      batteryMsg.power_supply_health = powerSupplyHealth;
      batteryStatePublisher->publish();

      auto& powerStatusMsg = powerStatusPublisher->get_message();
      powerStatusMsg.stamp.sec = ns / 1000000000;
      powerStatusMsg.stamp.nanosec = ns % 1000000000;
      powerStatusMsg.battery_voltage = batteryVoltage;
      powerStatusMsg.battery_percentage = batteryPercentage;
      powerStatusMsg.battery_present = batteryPresent;
      powerStatusMsg.charge_voltage = chargeVoltage;
      powerStatusMsg.charge_current = chargeCurrent;
      powerStatusMsg.charger_present = isChargerPresent;
      powerStatusMsg.charge_path_enabled = chargeEnabled;
      powerStatusMsg.charging_allowed = chargingAllowed;
      powerStatusMsg.charging_state = chargingState;
      powerStatusMsg.inhibit_reason = chargeInhibitReason;
      powerStatusMsg.retry_remaining_ms = chargeRetryRemainingMs;
      powerStatusPublisher->publish();
    });

  auto emergencyCommandSubscription = new uros::Subscriber<std_msgs__msg__Bool>(
    *node,
    "emergency/command",
    UROS_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    [](const std_msgs__msg__Bool& msg) { emergency::handleCommand(msg.data); });

  auto emergencyStatusPublisher = new uros::Publisher<omros2_firmware_msgs__msg__EmergencyStatus>(
    *node,
    "emergency/status",
    UROS_GET_MSG_TYPE_SUPPORT(omros2_firmware_msgs, msg, EmergencyStatus));
  auto emergencyTimer = new uros::Timer(
    *node,
    100,
    [emergencyStatusPublisher]()
    {
      auto state = emergency::getState();
      auto& msg = emergencyStatusPublisher->get_message();

      msg.active = state.active;
      msg.stop_active = state.stop_active;
      msg.lift_active = state.lift_active;
      msg.tilt_active = state.tilt_active;
      msg.software_requested = state.software_requested;
      msg.release_blocked = state.release_blocked;
      msg.lifted_wheels = state.lifted_wheels;
      emergencyStatusPublisher->publish();
    });

  isSecondCoreSetupDone = true;
}

void loop1()
{
  support->spin();

  ledStatus.setFlag(LED_STATUS_ROS_CONNECTED, support->get_state() == uros::State::AGENT_CONNECTED);
  ledStatus.setFlag(LED_STATUS_EMERGENCY, emergency::getState().active);

  delay(10);  // Small delay to prevent tight loop
}

void setup()
{
  analogReadResolution(12);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_RASPI_POWER, OUTPUT);
  pinMode(PIN_ENABLE_CHARGE, OUTPUT);
  pinMode(PIN_ESC_SHUTDOWN, OUTPUT);

  digitalWrite(PIN_RASPI_POWER, HIGH);
  digitalWrite(PIN_ENABLE_CHARGE, LOW);
  digitalWrite(PIN_ESC_SHUTDOWN, LOW);

  emergency::init();

  ledStatus.init(PIN_NEOPIXEL, 1);
  ledStatus.setColor(0, 0, 0, false);  // reset

  if (!initIMU())
  {
    ledStatus.setFlag(LED_STATUS_IMU_FAILED, true);
  }

  isFirstCoreSetupDone = true;
}

void loop()
{
  while (!isSecondCoreSetupDone)
  {
    emergency::update();
    delay(10);
  }

  emergency::update();
  ledStatus.update();
  chargingLoop();

  delay(10);
}
