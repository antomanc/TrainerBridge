#pragma once

#include <Arduino.h>

class TrainerMatcher {
public:
  // Evaluates whether an advertised BLE device matches the configured target criteria.
  // Priority:
  // 1. targetMac (exact, case-insensitive match when non-empty)
  // 2. targetNameContains (case-insensitive substring match when non-empty)
  // 3. Any device advertising the FTMS service
  static bool matches(
    const String &name,
    const String &address,
    bool advertisesFtms,
    const char *targetNameContains,
    const char *targetMac
  );

  // Verifies the matcher logic at boot time.
  static bool selfCheck();

private:
  static bool addressesMatch(const String &a, const String &b);
  static bool stringContainsIgnoreCase(String source, String needle);
};
