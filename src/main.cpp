#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/battery_state.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>

#include "pins.h"
#include "hardware.h"

#include <NeoPixelConnect.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error Only Micro-ROS serial transport is supported
#endif

#define CHARGING_UPDATE_INTERVAL 1000

rcl_publisher_t batteryStatePublisher;
rcl_publisher_t chargeVoltagePublisher;
rcl_publisher_t chargerPresentPublisher;
sensor_msgs__msg__BatteryState batteryState = {
    .design_capacity = BATT_DESIGNED_CAPACITY,
    .power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LION,
    .present = true,
};
std_msgs__msg__Float32 chargeVoltageMsg;
std_msgs__msg__Bool chargerPresentMsg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t pingAgentTimer;
rcl_timer_t batteryStateTimer;
bool rosAgentConnected;
uint8_t agentPingAttempts = 0;

void configureNode();

void chargingLoop();

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


NeoPixelConnect *led;
ulong lastLEDUpdate;
bool ledBlinkState;
void setLED(uint8_t r, uint8_t g, uint8_t b, bool blink = false) {
    if (led == NULL) {
        return;
    }

    if (blink && ledBlinkState) {
        led->neoPixelSetValue(0, 0, 0, 0, true);
    } else {
        led->neoPixelSetValue(0, r, g, b, true);
    }
}
void updateLED() {
    auto now = millis();
    if (now - lastLEDUpdate < 500) {
        return;
    }

    if (rosAgentConnected) {
        // green
        setLED(0, 255, 0);
    } else {
        // blink red
        setLED(255, 0, 0, true);
    }

    lastLEDUpdate = now;
    ledBlinkState = !ledBlinkState;
}


// Error handle loop
void error_loop() {
    setLED(255, 0, 0);

    while (1) {
        delay(100);
    }
}

void pingAgentCallback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer == NULL) {
        return;
    }

    rosAgentConnected = RMW_RET_OK == rmw_uros_ping_agent(100, 1);
}

void publishBatteryStateCallback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        RCSOFTCHECK(rcl_publish(&batteryStatePublisher, &batteryState, NULL));
        RCSOFTCHECK(rcl_publish(&chargeVoltagePublisher, &chargeVoltageMsg, NULL));
        RCSOFTCHECK(rcl_publish(&chargerPresentPublisher, &chargerPresentMsg, NULL));

        rosAgentConnected = RMW_RET_OK == rmw_uros_ping_agent(100, 1);

        if (!rosAgentConnected) {
            agentPingAttempts++;
        }

        if (rosAgentConnected) {
            agentPingAttempts = 0;
        }

        if (agentPingAttempts == 5) {
            // todo: work around for https://github.com/jkaflik/omros2-firmware/issues/1
            watchdog_reboot(0, 0, 0);
        }
    }
}

void setup1() {
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop1() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
}

void setup() {
    delay(500);

    led = new NeoPixelConnect(PIN_NEOPIXEL, 1);

    // yellow LED
    setLED(255, 255, 0);

    analogReadResolution(12);

    pinMode(PIN_RASPI_POWER, OUTPUT);
    pinMode(PIN_ENABLE_CHARGE, OUTPUT);

    digitalWrite(PIN_RASPI_POWER, HIGH);
    digitalWrite(PIN_ENABLE_CHARGE, LOW);

    // orange LED
    setLED(255, 128, 0);

    // Configure serial transport
    Serial1.begin(115200);
    set_microros_serial_transports(Serial1);

    configureNode();
}

void configureNode() {
    allocator = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "openmower_mainboard", "", &support));

    // create BatteryState batteryStatePublisher
    RCCHECK(rclc_publisher_init_default(
            &batteryStatePublisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
            "power"));
    RCCHECK(rclc_publisher_init_default(
            &chargeVoltagePublisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "power/charge_voltage"));
    RCCHECK(rclc_publisher_init_default(
            &chargerPresentPublisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
            "power/charger_present"));

    RCCHECK(rclc_timer_init_default(
            &batteryStateTimer,
            &support,
            RCL_MS_TO_NS(1000),
            publishBatteryStateCallback));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &batteryStateTimer));
}

void loop() {
    updateLED();
    chargingLoop();

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

    batteryState.present = batteryState.voltage > BATT_PRESENT_VOLTAGE;

    chargeVoltageMsg.data =
            (float) analogRead(PIN_ANALOG_CHARGE_VOLTAGE) * (3.3f / 4096.0f) * ((VIN_R1 + VIN_R2) / VIN_R2);

    chargerPresentMsg.data = chargeVoltageMsg.data >= CHARGER_PRESENT_VOLTAGE;

    batteryState.percentage = std::min(
        (batteryState.voltage - BATT_EMPTY) / (BATT_FULL - BATT_EMPTY) * 100,
        100.0f
    );

    auto shouldCharge = chargeVoltageMsg.data <= CHARGE_VOLTAGE_CUTOFF
            && batteryState.charge < CHARGE_CURRENT_CUTOFF
            && batteryState.voltage <= BATT_ABSOLUTE_MAX;

    digitalWrite(PIN_ENABLE_CHARGE, shouldCharge ? HIGH : LOW);

    auto isCharging = chargeVoltageMsg.data > CHARGE_VOLTAGE_MIN;

    batteryState.power_supply_status = shouldCharge
            ? (isCharging ? sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_CHARGING : sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_NOT_CHARGING)
            : sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;

    // assume power supply health based on:
    // - it's minimum voltage
    // - it's maximum voltage
    // - it's presence
    batteryState.power_supply_health = batteryState.present
            ? (batteryState.voltage < BATT_ABSOLUTE_MIN
               ? sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_DEAD
               : (batteryState.voltage > BATT_ABSOLUTE_MAX
                  ? sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_OVERHEAT
                  : sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_GOOD))
            : sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNKNOWN;

    digitalWrite(PIN_ENABLE_CHARGE, shouldCharge ? HIGH : LOW);

    lastChargingLoop = now;
}
