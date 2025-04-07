#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>

#include <Arduino.h>
#include <EMA.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/imu.h>

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
ulong lastChargingLoop = 0;

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

  batteryPercentage =
    std::min((batteryVoltage - BATT_EMPTY) / (BATT_FULL - BATT_EMPTY) * 100, 100.0f);

  bool shouldCharge = chargeVoltage <= CHARGE_VOLTAGE_CUTOFF
                      && chargeCurrent < CHARGE_CURRENT_CUTOFF
                      && batteryVoltage <= BATT_ABSOLUTE_MAX && batteryVoltage;
  chargeEnabled = shouldCharge && batteryVoltage < BATT_FULL_HYSTERESIS;

  digitalWrite(PIN_ENABLE_CHARGE, chargeEnabled ? HIGH : LOW);

  bool isCharging = chargeVoltage > batteryVoltage;

  if (shouldCharge)
  {
    powerSupplyStatus = isCharging
                          ? sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_CHARGING
                          : sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_NOT_CHARGING;
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
      powerSupplyHealth = sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_OVERHEAT;
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
  auto randomNumberPublisher = new uros::Publisher<std_msgs__msg__Float32>(
    *node, "random_number", UROS_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32));

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
  auto batteryStateTimer = new uros::Timer(*node,
                                           1000,
                                           [batteryStatePublisher]()
                                           {
                                             auto& msg = batteryStatePublisher->get_message();

                                             auto ns = rmw_uros_epoch_nanos();
                                             msg.header.stamp.sec = ns / 1000000000;
                                             msg.header.stamp.nanosec = ns % 1000000000;
                                             msg.voltage = batteryVoltage;
                                             msg.charge = chargeCurrent;
                                             msg.percentage = batteryPercentage;
                                             msg.present = batteryPresent;
                                             msg.power_supply_status = powerSupplyStatus;
                                             msg.power_supply_health = powerSupplyHealth;

                                             batteryStatePublisher->publish();
                                           });
  auto chargeVoltagePublisher = new uros::Publisher<std_msgs__msg__Float32>(
    *node, "power/charge_voltage", UROS_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32));
  auto chargeVoltageTimer = new uros::Timer(*node,
                                            1000,
                                            [chargeVoltagePublisher]()
                                            {
                                              auto& msg = chargeVoltagePublisher->get_message();
                                              msg.data = chargeVoltage;
                                              chargeVoltagePublisher->publish();
                                            });
  auto chargerPresentPublisher = new uros::Publisher<std_msgs__msg__Bool>(
    *node, "power/charger_present", UROS_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool));
  auto chargerPresentTimer = new uros::Timer(*node,
                                             1000,
                                             [chargerPresentPublisher]()
                                             {
                                               auto& msg = chargerPresentPublisher->get_message();
                                               msg.data = isChargerPresent;
                                               chargerPresentPublisher->publish();
                                             });

  isSecondCoreSetupDone = true;
}

void loop1()
{
  support->spin();

  ledStatus.setFlag(LED_STATUS_ROS_CONNECTED, support->get_state() == uros::State::AGENT_CONNECTED);

  delay(10);  // Small delay to prevent tight loop
}

void setup()
{
  analogReadResolution(12);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_RASPI_POWER, OUTPUT);
  pinMode(PIN_ENABLE_CHARGE, OUTPUT);

  digitalWrite(PIN_RASPI_POWER, HIGH);
  digitalWrite(PIN_ENABLE_CHARGE, LOW);

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
    delay(10);
  }

  ledStatus.update();
  chargingLoop();

  delay(10);
}
