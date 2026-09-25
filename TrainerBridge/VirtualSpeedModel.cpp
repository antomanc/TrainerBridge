#include "VirtualSpeedModel.h"

VirtualSpeedModel::VirtualSpeedModel()
  : _speedMps(0.0f),
    _cumulativeRevs(0),
    _lastEventMs(0),
    _wheelRemainder(0.0f),
    _lastUpdateMs(0)
{
}

float VirtualSpeedModel::solveSteadyStateSpeed(int16_t watts)
{
  if (watts <= 0) return 0.0f;

  const float aeroFactor = 0.5f * VIRTUAL_AIR_DENSITY_KG_M3 * VIRTUAL_CDA;
  const float rollingForce = VIRTUAL_CRR * VIRTUAL_TOTAL_MASS_KG * 9.80665f;
  const float wheelPower = (float)watts * VIRTUAL_DRIVETRAIN_EFFICIENCY;

  // Initial estimate
  float speed = 8.0f;

  // Newton-Raphson iterations to solve: aeroFactor * v^3 + rollingForce * v - wheelPower = 0
  for (uint8_t i = 0; i < 6; i++)
  {
    float speedSquared = speed * speed;
    float error = aeroFactor * speedSquared * speed + rollingForce * speed - wheelPower;
    float derivative = 3.0f * aeroFactor * speedSquared + rollingForce;

    speed -= error / derivative;
  }

  return (speed > 0.0f) ? speed : 0.0f;
}

void VirtualSpeedModel::update(int16_t watts, unsigned long nowMs)
{
  if (_lastUpdateMs == 0)
  {
    _lastUpdateMs = nowMs;
    _speedMps = solveSteadyStateSpeed(watts);
    return;
  }

  float dt = (float)(nowMs - _lastUpdateMs) / 1000.0f;
  _lastUpdateMs = nowMs;

  if (dt <= 0.0f) return;
  if (dt > 3.0f) dt = 0.25f; // Clamp anomalous gaps

  float targetSpeed = solveSteadyStateSpeed(watts);

  // Inertia and flywheel deceleration model
  if (targetSpeed > _speedMps)
  {
    float maxAccelDelta = 2.5f * dt; // max 2.5 m/s^2 acceleration
    if (targetSpeed - _speedMps > maxAccelDelta)
    {
      _speedMps += maxAccelDelta;
    }
    else
    {
      _speedMps = targetSpeed;
    }
  }
  else if (targetSpeed < _speedMps)
  {
    float maxDecelDelta = VIRTUAL_COASTING_DECEL_RATE * dt;
    if (_speedMps - targetSpeed > maxDecelDelta)
    {
      _speedMps -= maxDecelDelta;
    }
    else
    {
      _speedMps = targetSpeed;
    }
  }

  if (_speedMps < 0.2f)
  {
    _speedMps = 0.0f;
  }

  // Integrate wheel revolutions
  float deltaDist = _speedMps * dt;
  float deltaRevs = deltaDist / VIRTUAL_WHEEL_CIRCUMFERENCE_M;
  _wheelRemainder += deltaRevs;

  uint32_t completedRevs = (uint32_t)_wheelRemainder;
  if (completedRevs > 0)
  {
    _cumulativeRevs += completedRevs;
    _wheelRemainder -= (float)completedRevs;

    // Calculate the precise millisecond timestamp when the last whole revolution boundary was crossed
    float timeFractionAfterLastRev = (_speedMps > 0.0f) ?
      (_wheelRemainder / (_speedMps / VIRTUAL_WHEEL_CIRCUMFERENCE_M)) : 0.0f;
    unsigned long msSinceLastRev = (unsigned long)(timeFractionAfterLastRev * 1000.0f);
    if (msSinceLastRev > (unsigned long)(dt * 1000.0f))
    {
      msSinceLastRev = 0;
    }
    _lastEventMs = nowMs - msSinceLastRev;
  }
}

uint16_t VirtualSpeedModel::getLastWheelEventTime1024() const
{
  return (uint16_t)(((uint64_t)_lastEventMs * 1024ULL) / 1000ULL);
}

uint16_t VirtualSpeedModel::getLastWheelEventTime2048() const
{
  return (uint16_t)(((uint64_t)_lastEventMs * 2048ULL) / 1000ULL);
}

void VirtualSpeedModel::stop(unsigned long nowMs)
{
  _speedMps = 0.0f;
  _lastUpdateMs = nowMs;
}

void VirtualSpeedModel::reset()
{
  _speedMps = 0.0f;
  _cumulativeRevs = 0;
  _lastEventMs = 0;
  _wheelRemainder = 0.0f;
  _lastUpdateMs = 0;
}

float VirtualSpeedModel::getDistanceKm() const
{
  return ((float)_cumulativeRevs + _wheelRemainder) * VIRTUAL_WHEEL_CIRCUMFERENCE_M / 1000.0f;
}

uint32_t VirtualSpeedModel::getDistanceMeters() const
{
  float dist = ((float)_cumulativeRevs + _wheelRemainder) * VIRTUAL_WHEEL_CIRCUMFERENCE_M;
  if (dist < 0.0f) dist = 0.0f;
  uint32_t distM = (uint32_t)dist;
  return distM & 0x00FFFFFF; // Clamped to 24-bit for FTMS compatibility
}

bool VirtualSpeedModel::selfCheck()
{
  float speedAt0W = solveSteadyStateSpeed(0);
  float speedAt150W = solveSteadyStateSpeed(150) * 3.6f;

  return (speedAt0W == 0.0f) &&
         (speedAt150W >= 30.0f) &&
         (speedAt150W <= 31.0f);
}
