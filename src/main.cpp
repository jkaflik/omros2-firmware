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

rcl_publisher_t batteryStatePublisher;
rcl_publisher_t chargeVoltagePublisher;
rcl_publisher_t chargerPresentPublisher;
rcl_publisher_t imuPublisher;
rcl_timer_t timer;
rcl_timer_t imuTimer;
rcl_timer_t batteryStateTimer;

sensor_msgs__msg__BatteryState batteryState = {
  .design_capacity = BATT_DESIGNED_CAPACITY,
  .power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LION,
  .present = true,
};
sensor_msgs__msg__Imu imuMsg;
std_msgs__msg__Float32 chargeVoltageMsg;
std_msgs__msg__Bool chargerPresentMsg;

void chargingLoop();

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      return;                    \
    }                            \
  }

#define imuFrequency 50

void error_loop()
{
  // Special error handling - constant red light
  ledStatus.setColor(255, 0, 0, false);
  delay(1000);

  // todo: work around for https://github.com/jkaflik/omros2-firmware/issues/1
  watchdog_reboot(0, 0, 0);
}

void synchronizeCallback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer == NULL)
  {
    return;
  }

  RCSOFTCHECK(rmw_uros_sync_session(10));
}

void publishIMU(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer == NULL)
  {
    return;
  }

  if (!imuRead(&imuMsg))
  {
    return;
  }

  auto ns = rmw_uros_epoch_nanos();
  imuMsg.header.stamp.sec = ns / 1000000000;
  imuMsg.header.stamp.nanosec = ns % 1000000000;
  imuMsg.header.frame_id.capacity = 3;
  imuMsg.header.frame_id.data = "imu";
  imuMsg.header.frame_id.size = 3;

  RCSOFTCHECK(rcl_publish(&imuPublisher, &imuMsg, NULL));
}

void publishBatteryStateCallback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    RCSOFTCHECK(rcl_publish(&batteryStatePublisher, &batteryState, NULL));
    RCSOFTCHECK(rcl_publish(&chargeVoltagePublisher, &chargeVoltageMsg, NULL));
    RCSOFTCHECK(rcl_publish(&chargerPresentPublisher, &chargerPresentMsg, NULL));
  }
}

bool isFirstCoreSetupDone;
bool isSecondCoreSetupDone;

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
  initialize();

  isSecondCoreSetupDone = true;
}

void loop1()
{
  spin();

  ledStatus.setFlag(LED_STATUS_ROS_CONNECTED, state == AGENT_CONNECTED);

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

// void configureNode()
// {
//     // Initialize ROS
//     allocator = rcl_get_default_allocator();
//     if (RCL_RET_OK == rclc_support_init(&support, 0, NULL, &allocator))
//     {

//         // Create node
//         if (RCL_RET_OK == rclc_node_init_default(&node, "openmower_mainboard", "", &support))
//         {

//             // Initialize publishers
//             bool publishers_ok = true;

//             publishers_ok &= (RCL_RET_OK == rclc_publisher_init_default(
//                                                 &batteryStatePublisher,
//                                                 &node,
//                                                 ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg,
//                                                 BatteryState), "power"));

//             publishers_ok &= (RCL_RET_OK == rclc_publisher_init_default(
//                                                 &chargeVoltagePublisher,
//                                                 &node,
//                                                 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg,
//                                                 Float32), "power/charge_voltage"));

//             publishers_ok &= (RCL_RET_OK == rclc_publisher_init_default(
//                                                 &chargerPresentPublisher,
//                                                 &node,
//                                                 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
//                                                 "power/charger_present"));

//             publishers_ok &= (RCL_RET_OK == rclc_publisher_init_default(
//                                                 &imuPublisher,
//                                                 &node,
//                                                 ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg,
//                                                 Imu), "imu/data_raw"));

//             if (publishers_ok)
//             {
//                 // Initialize timers
//                 bool timers_ok = true;

//                 timers_ok &= (RCL_RET_OK == rclc_timer_init_default(
//                                                 &batteryStateTimer,
//                                                 &support,
//                                                 RCL_MS_TO_NS(1000),
//                                                 publishBatteryStateCallback));

//                 timers_ok &= (RCL_RET_OK == rclc_timer_init_default(
//                                                 &imuTimer,
//                                                 &support,
//                                                 RCL_MS_TO_NS(1000 / imuFrequency),
//                                                 publishIMU));

//                 timers_ok &= (RCL_RET_OK == rclc_timer_init_default(
//                                                 &synchronizeTimer,
//                                                 &support,
//                                                 RCL_S_TO_NS(60),
//                                                 synchronizeCallback));

//                 if (timers_ok)
//                 {
//                     // Create executor
//                     if (RCL_RET_OK == rclc_executor_init(&executor, &support.context, 3,
//                     &allocator))
//                     {
//                         bool executor_ok = true;
//                         executor_ok &= (RCL_RET_OK == rclc_executor_add_timer(&executor,
//                         &synchronizeTimer)); executor_ok &= (RCL_RET_OK ==
//                         rclc_executor_add_timer(&executor, &batteryStateTimer)); executor_ok &=
//                         (RCL_RET_OK == rclc_executor_add_timer(&executor, &imuTimer));

//                         if (executor_ok)
//                         {
//                             rosInitialized = true;
//                             rosAgentConnected = true;
//                             agentPingAttempts = 0;

//                             // Sync time with agent
//                             rmw_uros_sync_session(10);
//                         }
//                     }
//                 }
//             }
//         }
//     }
// }

void loop()
{
  while (!isSecondCoreSetupDone)
  {
    delay(10);
  }

  // Update LED based on current status flags
  ledStatus.update();
  chargingLoop();

  delay(10);  // Small delay to prevent tight loop
}

// Global variables to store battery status
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

  batteryState.voltage = batteryVoltage;
  batteryState.charge = chargeCurrent;
  batteryState.percentage = batteryPercentage;
  batteryState.present = batteryPresent;
  batteryState.power_supply_status = powerSupplyStatus;
  batteryState.power_supply_health = powerSupplyHealth;

  chargeVoltageMsg.data = chargeVoltage;

  chargerPresentMsg.data = isChargerPresent;

  lastChargingLoop = now;
}
