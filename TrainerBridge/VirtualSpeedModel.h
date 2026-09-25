#pragma once

#include <Arduino.h>
#include "Config.h"

class VirtualSpeedModel {
public:
  VirtualSpeedModel();

  // Updates the model with current watts and timestamp.
  // Computes steady-state speed, applies acceleration/flywheel coasting,
  // accumulates distance and wheel revolutions, and updates event time.
  void update(int16_t watts, unsigned long nowMs);

  // Brings the virtual wheel to an immediate stop.
  void stop(unsigned long nowMs);

  // Resets revolutions, speed, and distance to zero.
  void reset();

  // Getters
  float getSpeedMps() const { return _speedMps; }
  float getSpeedKmh() const { return _speedMps * 3.6f; }
  uint32_t getCumulativeRevs() const { return _cumulativeRevs; }

  // CSC Service (0x1816) uses 1/1024 second resolution (Garmin Speed Sensor standard)
  uint16_t getLastWheelEventTime1024() const;

  // CPS Service (0x1818) uses 1/2048 second resolution
  uint16_t getLastWheelEventTime2048() const;

  float getDistanceKm() const;
  uint32_t getDistanceMeters() const;

  // Solves flat-road steady-state speed using aerodynamic and rolling resistance formula.
  static float solveSteadyStateSpeed(int16_t watts);

  // Boot-time self-check
  static bool selfCheck();

private:
  float _speedMps;
  uint32_t _cumulativeRevs;
  unsigned long _lastEventMs;
  float _wheelRemainder;
  unsigned long _lastUpdateMs;
};
