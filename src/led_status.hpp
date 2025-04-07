#pragma once

#include <Arduino.h>
#include <NeoPixelConnect.h>

#include <vector>

// LED status flags as bits for multi-status representation
enum LedStatusFlag
{
  LED_STATUS_ROS_CONNECTED = (1 << 0),
  LED_STATUS_CHARGING = (1 << 1),
  LED_STATUS_DISCHARGING = (1 << 2),
  LED_STATUS_BATTERY_LOW = (1 << 3),
  LED_STATUS_IMU_FAILED = (1 << 4),
};

struct StatusColor
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

/**
 * @brief Controls LED status indicators with color and blink patterns.
 *
 * The LedStatus class provides functionality to manage one or more NeoPixel LEDs
 * to indicate various system statuses through different colors and blink patterns.
 * It supports multiple status flags which can be shown in sequence, with configurable
 * durations and transitions between each status.
 *
 * Key features:
 * - Controls LED color and blink patterns based on status flags
 * - Sequences through multiple active status indicators
 * - Customizable timing for status display and transitions
 * - Predefined status colors for common states (ROS connection, battery, etc.)
 *
 * Usage example:
 * @code
 * LedStatus statusLed;
 * statusLed.init(PIN_NEOPIXEL, 1);
 * statusLed.setFlag(LED_STATUS_ROS_CONNECTED, true);
 *
 * // In the main loop
 * statusLed.update();
 * @endcode
 *
 * @note The update() method must be called regularly (typically in the main loop)
 * to handle blinking and status cycling.
 */
class LedStatus
{
private:
  NeoPixelConnect* led;
  uint32_t statusFlags;
  unsigned long lastUpdate;
  bool blinkState;
  const unsigned long blinkInterval = 250;  // ms

  // Sequence control
  unsigned long sequenceStartTime;
  unsigned int currentStatusIndex;
  const unsigned long statusDuration = 800;     // ms for each status in sequence
  const unsigned long separatorDuration = 200;  // ms black between statuses

  // Status colors mapping
  std::vector<std::pair<LedStatusFlag, StatusColor>> statusColorMap;

public:
  LedStatus()
    : led(nullptr),
      statusFlags(0),
      lastUpdate(0),
      blinkState(false),
      sequenceStartTime(0),
      currentStatusIndex(0)
  {
  }

  void init(uint8_t pin, uint8_t numPixels = 1)
  {
    led = new NeoPixelConnect(pin, numPixels);
    setupStatusColorMap();
  }

  void setFlag(LedStatusFlag flag, bool value)
  {
    if (value)
    {
      statusFlags |= static_cast<uint32_t>(flag);
    }
    else
    {
      statusFlags &= ~static_cast<uint32_t>(flag);
    }
  }

  bool getFlag(LedStatusFlag flag) const
  {
    return (statusFlags & static_cast<uint32_t>(flag)) != 0;
  }

  void setColor(uint8_t r, uint8_t g, uint8_t b, bool blink = false)
  {
    if (led == nullptr)
    {
      return;
    }

    if (blink && blinkState)
    {
      led->neoPixelSetValue(0, 0, 0, 0, true);
    }
    else
    {
      led->neoPixelSetValue(0, r, g, b, true);
    }
  }

  void update()
  {
    unsigned long currentTime = millis();

    if (currentTime - lastUpdate >= blinkInterval)
    {
      blinkState = !blinkState;
      lastUpdate = currentTime;
    }

    std::vector<LedStatusFlag> activeStatuses = getActiveStatuses();

    // with no active statuses, turn off the LED
    if (activeStatuses.empty())
    {
      led->neoPixelSetValue(0, 0, 0, 0, true);
      return;
    }

    if (currentTime - sequenceStartTime
        >= (statusDuration + separatorDuration) * activeStatuses.size())
    {
      sequenceStartTime = currentTime;
      currentStatusIndex = 0;
    }

    unsigned long elapsedInSequence = currentTime - sequenceStartTime;
    unsigned long cycleDuration = statusDuration + separatorDuration;
    currentStatusIndex = (elapsedInSequence / cycleDuration) % activeStatuses.size();
    unsigned long elapsedInCycle = elapsedInSequence % cycleDuration;

    if (elapsedInCycle < statusDuration)
    {
      LedStatusFlag currentStatus = activeStatuses[currentStatusIndex];

      for (const auto& statusPair : statusColorMap)
      {
        if (statusPair.first == currentStatus)
        {
          setColor(statusPair.second.r, statusPair.second.g, statusPair.second.b, false);
          break;
        }
      }
    }
    else
    {
      // turn off the LED during the separator duration
      led->neoPixelSetValue(0, 0, 0, 0, true);
    }
  }

private:
  void setupStatusColorMap()
  {
    // Define colors and blink patterns for each status flag
    statusColorMap.push_back({LED_STATUS_ROS_CONNECTED, {0, 255, 0}});  // Green for ROS connected
    statusColorMap.push_back({LED_STATUS_CHARGING, {255, 255, 0}});     // Yellow for charging
    statusColorMap.push_back({LED_STATUS_DISCHARGING, {255, 0, 255}});  // Magenta for discharging
    statusColorMap.push_back({LED_STATUS_BATTERY_LOW, {255, 0, 0}});    // Red for battery low
    statusColorMap.push_back({LED_STATUS_IMU_FAILED, {0, 0, 255}});     // Blue for IMU failed
  }

  std::vector<LedStatusFlag> getActiveStatuses() const
  {
    std::vector<LedStatusFlag> activeStatuses;

    // Otherwise, show all active statuses
    for (const auto& statusPair : statusColorMap)
    {
      if (getFlag(statusPair.first))
      {
        activeStatuses.push_back(statusPair.first);
      }
    }

    return activeStatuses;
  }
};

// Global instance for easy access
inline LedStatus ledStatus;
