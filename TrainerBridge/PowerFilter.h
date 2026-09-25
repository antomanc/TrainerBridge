#pragma once

#include <Arduino.h>

class PowerFilter {
public:
  PowerFilter(
    float scale = 1.0f,
    bool smooth = true,
    uint8_t shift = 2,
    unsigned long staleTimeoutMs = 1500
  );

  // Updates the filter with a new raw reading from the trainer.
  // Applies scaling, negative clamping (>= 0), zero-lag coasting, and Q8 EMA smoothing.
  int16_t update(int16_t rawWatts, unsigned long nowMs);

  // Checks whether the power reading has become stale due to lack of packets.
  bool isStale(unsigned long nowMs) const;

  // Resets the smoothing state and zeroes outputs.
  void reset();

  // Calibration scale
  void setScale(float scale);
  float getScale() const;

  // Getters
  int16_t getLastRaw() const { return _lastRawWatts; }
  int16_t getLastOutput() const { return _lastOutputWatts; }
  unsigned long getLastPacketTime() const { return _lastPacketTime; }

private:
  float _scale;
  bool _smooth;
  uint8_t _shift;
  unsigned long _staleTimeoutMs;

  int16_t _lastRawWatts;
  int16_t _lastOutputWatts;
  int32_t _smoothedQ8;
  bool _hasSmoothed;
  unsigned long _lastPacketTime;
};
