#pragma once

#include <Arduino.h>

namespace emergency
{

struct State
{
  bool active;
  bool stop_active;
  bool lift_active;
  bool tilt_active;
  bool software_requested;
  bool release_blocked;
  uint8_t lifted_wheels;
};

void init();
void update();
void handleCommand(bool latch_requested);
State getState();

}  // namespace emergency
