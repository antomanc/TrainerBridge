#include "PowerFilter.h"

PowerFilter::PowerFilter(
  float scale,
  bool smooth,
  uint8_t shift,
  unsigned long staleTimeoutMs
)
  : _scale(scale),
    _smooth(smooth),
    _shift(shift),
    _staleTimeoutMs(staleTimeoutMs),
    _lastRawWatts(0),
    _lastOutputWatts(0),
    _smoothedQ8(0),
    _hasSmoothed(false),
    _lastPacketTime(0)
{
}

int16_t PowerFilter::update(int16_t rawWatts, unsigned long nowMs)
{
  _lastRawWatts = rawWatts;
  _lastPacketTime = nowMs;

  // Clamp raw power to 0 if negative
  if (rawWatts < 0) {
    rawWatts = 0;
  }

  int16_t scaledWatts = (int16_t)(rawWatts * _scale);
  if (scaledWatts < 0) {
    scaledWatts = 0;
  }

  // Zero-lag coasting: bypass smoothing immediately if watts <= 0
  if (!_smooth || scaledWatts <= 0) {
    _smoothedQ8 = ((int32_t)scaledWatts) << 8;
    _hasSmoothed = (scaledWatts > 0);
    _lastOutputWatts = scaledWatts;
    return scaledWatts;
  }

  int32_t wattsQ8 = ((int32_t)scaledWatts) << 8;

  // First positive packet after coasting seeds filter directly without delay
  if (!_hasSmoothed) {
    _smoothedQ8 = wattsQ8;
    _hasSmoothed = true;
    _lastOutputWatts = scaledWatts;
    return scaledWatts;
  }

  // Fixed-point Q8 exponential moving average
  _smoothedQ8 += (wattsQ8 - _smoothedQ8) / (1 << _shift);

  _lastOutputWatts = (int16_t)((_smoothedQ8 + 128) >> 8);
  if (_lastOutputWatts < 0) {
    _lastOutputWatts = 0;
  }

  return _lastOutputWatts;
}

bool PowerFilter::isStale(unsigned long nowMs) const
{
  return (_lastPacketTime > 0 && (nowMs - _lastPacketTime > _staleTimeoutMs));
}

void PowerFilter::reset()
{
  _lastRawWatts = 0;
  _lastOutputWatts = 0;
  _smoothedQ8 = 0;
  _hasSmoothed = false;
  _lastPacketTime = 0;
}

void PowerFilter::setScale(float scale)
{
  if (scale >= 0.1f && scale <= 5.0f) {
    _scale = scale;
  }
}

float PowerFilter::getScale() const
{
  return _scale;
}
