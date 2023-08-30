#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/battery_state.h>

#include "pins.h"
#include "hardware.h"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error Only Micro-ROS serial transport is supported
#endif

#define CHARGING_UPDATE_INTERVAL 1000

rcl_publisher_t publisher;
sensor_msgs__msg__BatteryState batteryState = {
    .design_capacity = BATT_DESIGNED_CAPACITY,
    .power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LION,
    .present = true,
};

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

void configureNode();

void chargingLoop();

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
    while(1) {
        delay(100);
    }
}

void publishBatteryStateCallback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        RCSOFTCHECK(rcl_publish(&publisher, &batteryState, NULL));
    }
}

void setup() {
    analogReadResolution(12);

    pinMode(PIN_RASPI_POWER, OUTPUT);
    pinMode(PIN_ENABLE_CHARGE, OUTPUT);

    digitalWrite(PIN_RASPI_POWER, HIGH);

    // Configure serial transport
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(2000);

    configureNode();
}

void configureNode() {
    allocator = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "openmower_mainboard", "", &support));

    // create BatteryState publisher
    RCCHECK(rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
            "power"));

    // create timer,
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(timer_timeout),
            publishBatteryStateCallback));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
    chargingLoop();

    delay(100);
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

ulong lastChargingLoop = 0;
void chargingLoop() {
    auto now = millis();
    if (now - lastChargingLoop < CHARGING_UPDATE_INTERVAL) {
        return;
    }

    batteryState.voltage = (float) analogRead(PIN_ANALOG_BATTERY_VOLTAGE)
                           * (3.3f / 4096.0f) * ((VIN_R1 + VIN_R2) / VIN_R2);
    batteryState.charge = (float) analogRead(PIN_ANALOG_CHARGE_CURRENT)
                          * (3.3f / 4096.0f) / (CURRENT_SENSE_GAIN * R_SHUNT);
    auto chargeVoltage =
            (float) analogRead(PIN_ANALOG_CHARGE_VOLTAGE) * (3.3f / 4096.0f) * ((VIN_R1 + VIN_R2) / VIN_R2);

    batteryState.percentage = std::min(
        (batteryState.voltage - BATT_EMPTY) / (BATT_FULL - BATT_EMPTY) * 100,
        100.0f
    );

    auto shouldCharge = chargeVoltage <= CHARGE_VOLTAGE_CUTOFF
            && batteryState.charge < CHARGE_CURRENT_CUTOFF
            && batteryState.voltage <= BATT_ABSOLUTE_MAX;

    digitalWrite(PIN_ENABLE_CHARGE, shouldCharge ? HIGH : LOW);

    auto isCharging = chargeVoltage > CHARGE_VOLTAGE_MIN;

    batteryState.power_supply_status = shouldCharge
            ? (isCharging ? sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_CHARGING : sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_NOT_CHARGING)
            : sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;

    lastChargingLoop = now;
}
