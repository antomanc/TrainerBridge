#include "StatusLed.h"

static constexpr unsigned long SCAN_BLINK_INTERVAL_MS = 500;

StatusLed::StatusLed()
  : _pin(-1),
    _activeHigh(true),
    _enabled(false),
    _blinkWhileScanning(true),
    _currentState(LedState::OFF),
    _currentPinLevel(false),
    _lastToggleMs(0)
{
}

void StatusLed::begin(int8_t pin, bool activeHigh, bool enabled, bool blinkWhileScanning)
{
  _pin = pin;
  _activeHigh = activeHigh;
  _enabled = enabled;
  _blinkWhileScanning = blinkWhileScanning;
  _currentState = LedState::OFF;
  _lastToggleMs = 0;

  if (_enabled && _pin >= 0)
  {
    pinMode(_pin, OUTPUT);
    // Initial state: OFF (inactive electrical level)
    _currentPinLevel = !_activeHigh;
    digitalWrite(_pin, _currentPinLevel ? HIGH : LOW);
  }
}

void StatusLed::setPinLevel(bool level)
{
  if (_currentPinLevel != level)
  {
    _currentPinLevel = level;
    if (_enabled && _pin >= 0)
    {
      digitalWrite(_pin, _currentPinLevel ? HIGH : LOW);
    }
  }
}

LedState StatusLed::computeTargetState(bool realConnected, bool isSearching, bool enabled, bool blinkWhileScanning)
{
  if (!enabled)
  {
    return LedState::OFF;
  }

  if (realConnected)
  {
    return LedState::SOLID_ON;
  }

  if (isSearching && blinkWhileScanning)
  {
    return LedState::BLINK_SCANNING;
  }

  return LedState::OFF;
}

void StatusLed::update(bool realConnected, bool isSearching)
{
  if (!_enabled || _pin < 0)
  {
    return;
  }

  LedState targetState = computeTargetState(realConnected, isSearching, _enabled, _blinkWhileScanning);
  unsigned long now = millis();

  if (targetState != _currentState)
  {
    _currentState = targetState;
    _lastToggleMs = now;

    if (_currentState == LedState::SOLID_ON)
    {
      setPinLevel(_activeHigh);
    }
    else if (_currentState == LedState::OFF)
    {
      setPinLevel(!_activeHigh);
    }
    else if (_currentState == LedState::BLINK_SCANNING)
    {
      setPinLevel(_activeHigh);
    }
    return;
  }

  // Handle blink toggling during scanning
  if (_currentState == LedState::BLINK_SCANNING)
  {
    if (now - _lastToggleMs >= SCAN_BLINK_INTERVAL_MS)
    {
      _lastToggleMs = now;
      setPinLevel(!_currentPinLevel);
    }
  }
}

bool StatusLed::selfCheck()
{
  // Disabled always yields OFF
  if (computeTargetState(true, true, false, true) != LedState::OFF) return false;
  if (computeTargetState(false, true, false, true) != LedState::OFF) return false;

  // Enabled & connected yields SOLID_ON regardless of searching flag
  if (computeTargetState(true, false, true, true) != LedState::SOLID_ON) return false;
  if (computeTargetState(true, true, true, true) != LedState::SOLID_ON) return false;

  // Enabled & searching with blink enabled yields BLINK_SCANNING
  if (computeTargetState(false, true, true, true) != LedState::BLINK_SCANNING) return false;

  // Enabled & searching with blink disabled yields OFF
  if (computeTargetState(false, true, true, false) != LedState::OFF) return false;

  // Enabled & idle (not searching, not connected) yields OFF
  if (computeTargetState(false, false, true, true) != LedState::OFF) return false;

  return true;
}
