#include "emergency.h"

#include <pico/mutex.h>

#include "pins.h"

namespace emergency
{

namespace
{
constexpr unsigned long STOP_DEBOUNCE_MS = 20;
constexpr unsigned long LIFT_PERIOD_MS = 100;
constexpr unsigned long TILT_PERIOD_MS = 2500;
constexpr unsigned long COMMAND_WINDOW_MS = 1000;
constexpr uint8_t COMMAND_CONFIRMATIONS_REQUIRED = 3;

auto_init_mutex(emergency_mutex);

bool latch_active = false;
bool stop_active = false;
bool lift_active = false;
bool tilt_active = false;
bool software_requested = false;
bool release_blocked = false;
uint8_t lifted_wheels = 0;

bool raw_stop_active = false;
bool raw_lift_input_active = false;

unsigned long stop_started_ms = 0;
unsigned long lift_started_ms = 0;
unsigned long tilt_started_ms = 0;

bool command_pending = false;
bool pending_command = false;
uint8_t pending_command_count = 0;
unsigned long pending_command_started_ms = 0;

bool isActiveLowInput(uint8_t pin)
{
  return digitalRead(pin) == LOW;
}

bool elapsedSinceStart(bool condition,
                       unsigned long& started_ms,
                       unsigned long threshold_ms,
                       unsigned long now)
{
  if (!condition)
  {
    started_ms = 0;
    return false;
  }

  if (started_ms == 0)
  {
    started_ms = now;
    return false;
  }

  return now - started_ms >= threshold_ms;
}

bool physicalInputPresent()
{
  return raw_stop_active || raw_lift_input_active;
}

void updateUnlocked()
{
  const unsigned long now = millis();

  lifted_wheels = 0;
  if (isActiveLowInput(PIN_EMERGENCY_1))
  {
    lifted_wheels++;
  }
  if (isActiveLowInput(PIN_EMERGENCY_2))
  {
    lifted_wheels++;
  }

  raw_lift_input_active = lifted_wheels > 0;
  raw_stop_active = isActiveLowInput(PIN_EMERGENCY_3) || isActiveLowInput(PIN_EMERGENCY_4);

  stop_active = elapsedSinceStart(raw_stop_active, stop_started_ms, STOP_DEBOUNCE_MS, now);
  lift_active = elapsedSinceStart(lifted_wheels >= 2, lift_started_ms, LIFT_PERIOD_MS, now);
  tilt_active = elapsedSinceStart(raw_lift_input_active, tilt_started_ms, TILT_PERIOD_MS, now);

  if (stop_active || lift_active || tilt_active)
  {
    latch_active = true;
  }

  release_blocked = latch_active && physicalInputPresent();
}

void requestLatch()
{
  software_requested = true;
  latch_active = true;
}

void requestRelease()
{
  updateUnlocked();

  if (physicalInputPresent())
  {
    release_blocked = true;
    return;
  }

  latch_active = false;
  software_requested = false;
  release_blocked = false;
}

}  // namespace

void init()
{
  pinMode(PIN_EMERGENCY_1, INPUT);
  pinMode(PIN_EMERGENCY_2, INPUT);
  pinMode(PIN_EMERGENCY_3, INPUT);
  pinMode(PIN_EMERGENCY_4, INPUT);
}

void update()
{
  mutex_enter_blocking(&emergency_mutex);
  updateUnlocked();
  mutex_exit(&emergency_mutex);
}

void handleCommand(bool latch_requested)
{
  mutex_enter_blocking(&emergency_mutex);

  const unsigned long now = millis();

  if (!command_pending || pending_command != latch_requested
      || now - pending_command_started_ms > COMMAND_WINDOW_MS)
  {
    command_pending = true;
    pending_command = latch_requested;
    pending_command_count = 1;
    pending_command_started_ms = now;
    mutex_exit(&emergency_mutex);
    return;
  }

  pending_command_count++;
  if (pending_command_count < COMMAND_CONFIRMATIONS_REQUIRED)
  {
    mutex_exit(&emergency_mutex);
    return;
  }

  command_pending = false;
  pending_command_count = 0;

  if (latch_requested)
  {
    requestLatch();
  }
  else
  {
    requestRelease();
  }

  mutex_exit(&emergency_mutex);
}

State getState()
{
  mutex_enter_blocking(&emergency_mutex);
  State state = {latch_active,
                 stop_active,
                 lift_active,
                 tilt_active,
                 software_requested,
                 release_blocked,
                 lifted_wheels};
  mutex_exit(&emergency_mutex);
  return state;
}

}  // namespace emergency
