# Charging state values.
uint8 CHARGING_STATE_DISCHARGING=0
uint8 CHARGING_STATE_NOT_CHARGING=1
uint8 CHARGING_STATE_CHARGING=2
uint8 CHARGING_STATE_RETRY_WAIT=3
uint8 CHARGING_STATE_REGEN_PATH=4

# Charging inhibit reason values.
uint8 INHIBIT_NONE=0
uint8 INHIBIT_NO_BATTERY=1
uint8 INHIBIT_BATTERY_FULL=2
uint8 INHIBIT_BATTERY_OVERVOLTAGE=3
uint8 INHIBIT_CHARGE_OVERVOLTAGE=4
uint8 INHIBIT_OVERCURRENT=5
uint8 INHIBIT_RETRY_DELAY=6

builtin_interfaces/Time stamp

float32 battery_voltage
float32 battery_percentage
bool battery_present

float32 charge_voltage
float32 charge_current
bool charger_present

# Actual PIN_ENABLE_CHARGE output state.
bool charge_path_enabled

# True only when dock charging is allowed by firmware limits.
bool charging_allowed
uint8 charging_state
uint8 inhibit_reason

uint32 retry_remaining_ms
