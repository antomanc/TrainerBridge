#pragma once

#include <stdint.h>

// =====================================================
// USER SETTINGS
// =====================================================

// Keep this 0 for stability. Set 1 only while debugging.
#define DEBUG_LOG 0

static constexpr char DEVICE_NAME[] = "TrainerBridge";

// Trainer selection priority:
// 1. TARGET_MAC, when non-empty (exact, case-insensitive match)
// 2. TARGET_NAME_CONTAINS, when non-empty (substring, case-insensitive match)
// 3. Otherwise, the first device advertising the FTMS service
// RYSEL intentionally tolerates common advertised variants such as
// VANRYSEL, VAN RYSEL, and VARYSEL while remaining trainer-specific.
static constexpr char TARGET_NAME_CONTAINS[] = "RYSEL";
static constexpr char TARGET_MAC[] = "";

// Measured-power calibration. 1.00 leaves the trainer watts unchanged.
static constexpr float DEFAULT_POWER_SCALE = 1.00f;

// Small display smoothing for real trainer watts.
// Alpha 1/4 keeps race/simulation response quick while reducing packet noise.
static constexpr bool SMOOTH_OUTPUT_POWER = true;
static constexpr uint8_t POWER_SMOOTHING_SHIFT = 2;
static constexpr unsigned long POWER_STALE_TIMEOUT_MS = 1500;

// =====================================================
// VIRTUAL SPEED & DISTANCE MODEL
// =====================================================

static constexpr bool ENABLE_VIRTUAL_SPEED = true;
static constexpr bool ENABLE_CSC_SERVICE = true;                // Broadcast standard BLE Speed Sensor (0x1816) for Garmin
static constexpr float VIRTUAL_TOTAL_MASS_KG = 85.0f;           // Rider (75 kg) + Bike (10 kg)
static constexpr float VIRTUAL_CRR = 0.004f;                    // Rolling resistance coefficient
static constexpr float VIRTUAL_CDA = 0.32f;                     // Aerodynamic drag area (m^2)
static constexpr float VIRTUAL_AIR_DENSITY_KG_M3 = 1.225f;      // Sea-level air density
static constexpr float VIRTUAL_DRIVETRAIN_EFFICIENCY = 0.98f;   // 98% efficiency
static constexpr float VIRTUAL_WHEEL_CIRCUMFERENCE_M = 2.096f;  // Standard 700x25c wheel circumference
static constexpr float VIRTUAL_COASTING_DECEL_RATE = 1.5f;      // Flywheel coasting deceleration (m/s^2)

// =====================================================
// ADVANCED BLE SETTINGS
// =====================================================

static constexpr int16_t POWER_MIN_W = 0;
static constexpr int16_t POWER_MAX_W = 800;
static constexpr uint16_t POWER_STEP_W = 1;

static constexpr int16_t RESISTANCE_MIN = 0;
static constexpr int16_t RESISTANCE_MAX = 100;
static constexpr uint16_t RESISTANCE_STEP = 1;

// interval = value * 1.25 ms: 24 = 30 ms, 40 = 50 ms.
static constexpr uint16_t CONN_INTERVAL_MIN = 24;
static constexpr uint16_t CONN_INTERVAL_MAX = 40;
static constexpr uint16_t CONN_LATENCY = 0;
static constexpr uint16_t CONN_TIMEOUT = 200; // 2 s

// If the trainer is unstable with acknowledged Control Point writes, set false.
static constexpr bool REAL_CP_WRITE_WITH_RESPONSE = true;

// Control Point transaction timeout in milliseconds
static constexpr unsigned long CP_TIMEOUT_MS = 1200;

// Sensor Location exposed via Cycling Power Service (0x2A5D)
// 0x00 = Other, 0x0B = Rear Wheel (avoids pedal calibration prompts on Garmin)
static constexpr uint8_t CPS_SENSOR_LOCATION = 0x00;
