#pragma once

#include <Arduino.h>

enum class LedState : uint8_t {
  OFF,
  BLINK_SCANNING,
  SOLID_ON
};

class StatusLed {
public:
  StatusLed();

  // Initializes pin mode and off state
  void begin(int8_t pin, bool activeHigh, bool enabled, bool blinkWhileScanning);

  // Non-blocking update called from loop()
  void update(bool realConnected, bool isSearching);

  // Current logical LED pattern state
  LedState getState() const { return _currentState; }

  // Pure logic state resolution (also used by selfCheck)
  static LedState computeTargetState(bool realConnected, bool isSearching, bool enabled, bool blinkWhileScanning);

  // Boot self-check for state transitions
  static bool selfCheck();

private:
  int8_t _pin;
  bool _activeHigh;
  bool _enabled;
  bool _blinkWhileScanning;

  LedState _currentState;
  bool _currentPinLevel; // Current electrical level driven to pin (true = HIGH, false = LOW)
  unsigned long _lastToggleMs;

  void setPinLevel(bool level);
};
