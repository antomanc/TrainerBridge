#pragma once

#include <Arduino.h>
#include <NimBLEDevice.h>

// =====================================================
// BLE SERVICE & CHARACTERISTIC UUIDs
// =====================================================

// Services
static const NimBLEUUID UUID_FTMS_SERVICE("1826");
static const NimBLEUUID UUID_CPS_SERVICE("1818");
static const NimBLEUUID UUID_CSC_SERVICE("1816");
static const NimBLEUUID UUID_DIS_SERVICE("180A");

// Fitness Machine Service (FTMS) Characteristics
static const NimBLEUUID UUID_FTMS_FEATURE("2ACC");
static const NimBLEUUID UUID_INDOOR_BIKE_DATA("2AD2");
static const NimBLEUUID UUID_SUPPORTED_RESISTANCE_RANGE("2AD6");
static const NimBLEUUID UUID_SUPPORTED_POWER_RANGE("2AD8");
static const NimBLEUUID UUID_FTMS_CONTROL_POINT("2AD9");
static const NimBLEUUID UUID_FTMS_STATUS("2ADA");

// Cycling Power Service (CPS) Characteristics
static const NimBLEUUID UUID_CYCLING_POWER_MEASUREMENT("2A63");
static const NimBLEUUID UUID_CYCLING_POWER_FEATURE("2A65");
static const NimBLEUUID UUID_SENSOR_LOCATION("2A5D");

// Cycling Speed and Cadence (CSC) Characteristics
static const NimBLEUUID UUID_CSC_MEASUREMENT("2A5B");
static const NimBLEUUID UUID_CSC_FEATURE("2A5C");
static const NimBLEUUID UUID_SC_CONTROL_POINT("2A55");

// Device Information Service (DIS) Characteristics
static const NimBLEUUID UUID_MANUFACTURER_NAME("2A29");
static const NimBLEUUID UUID_MODEL_NUMBER("2A24");

// =====================================================
// FTMS CONTROL POINT OPCODES (0x2AD9)
// =====================================================

static constexpr uint8_t FTMS_CP_OP_REQUEST_CONTROL        = 0x00;
static constexpr uint8_t FTMS_CP_OP_RESET                  = 0x01;
static constexpr uint8_t FTMS_CP_OP_SET_TARGET_SPEED       = 0x02;
static constexpr uint8_t FTMS_CP_OP_SET_TARGET_INCLINATION = 0x03;
static constexpr uint8_t FTMS_CP_OP_SET_TARGET_RESISTANCE  = 0x04;
static constexpr uint8_t FTMS_CP_OP_SET_TARGET_POWER       = 0x05;
static constexpr uint8_t FTMS_CP_OP_SET_TARGET_HEART_RATE  = 0x06;
static constexpr uint8_t FTMS_CP_OP_START_RESUME           = 0x07;
static constexpr uint8_t FTMS_CP_OP_STOP_PAUSE             = 0x08;
static constexpr uint8_t FTMS_CP_OP_SET_INDOOR_BIKE_SIM    = 0x11;
static constexpr uint8_t FTMS_CP_OP_SET_WHEEL_CIRCUMFERENCE= 0x12;
static constexpr uint8_t FTMS_CP_OP_SPIN_DOWN_CONTROL      = 0x13;
static constexpr uint8_t FTMS_CP_OP_RESPONSE_CODE          = 0x80;

// =====================================================
// FTMS CONTROL POINT RESULT CODES
// =====================================================

static constexpr uint8_t FTMS_CP_RES_SUCCESS               = 0x01;
static constexpr uint8_t FTMS_CP_RES_OP_CODE_NOT_SUPPORTED = 0x02;
static constexpr uint8_t FTMS_CP_RES_INVALID_PARAMETER     = 0x03;
static constexpr uint8_t FTMS_CP_RES_OPERATION_FAILED      = 0x04;
static constexpr uint8_t FTMS_CP_RES_CONTROL_NOT_PERMITTED = 0x05;

// =====================================================
// FTMS STATUS EVENT CODES (0x2ADA)
// =====================================================

static constexpr uint8_t FTMS_STATUS_RESET                 = 0x01;
static constexpr uint8_t FTMS_STATUS_STOPPED               = 0x02;
static constexpr uint8_t FTMS_STATUS_STARTED               = 0x04;
static constexpr uint8_t FTMS_STATUS_TARGET_POWER_CHANGED  = 0x08;
static constexpr uint8_t FTMS_STATUS_INDOOR_BIKE_SIM_CHANGED = 0x12;

// =====================================================
// SENSOR LOCATION (Bluetooth SIG Assigned Numbers)
// =====================================================

static constexpr uint8_t SENSOR_LOCATION_OTHER             = 0x00;
static constexpr uint8_t SENSOR_LOCATION_TOP_OF_SHOE       = 0x01;
static constexpr uint8_t SENSOR_LOCATION_IN_SHOE           = 0x02;
static constexpr uint8_t SENSOR_LOCATION_BOTTOM_OF_SHOE    = 0x03;
static constexpr uint8_t SENSOR_LOCATION_REAR_WHEEL        = 0x0B;
static constexpr uint8_t SENSOR_LOCATION_RIGHT_PEDAL       = 0x0D;

// =====================================================
// INDOOR BIKE DATA FLAGS (0x2AD2)
// =====================================================

static constexpr uint16_t FTMS_IBD_FLAG_MORE_DATA          = 0x0001;
static constexpr uint16_t FTMS_IBD_FLAG_AVG_SPEED          = 0x0002;
static constexpr uint16_t FTMS_IBD_FLAG_INST_CADENCE       = 0x0004;
static constexpr uint16_t FTMS_IBD_FLAG_AVG_CADENCE        = 0x0008;
static constexpr uint16_t FTMS_IBD_FLAG_TOTAL_DISTANCE     = 0x0010;
static constexpr uint16_t FTMS_IBD_FLAG_RESISTANCE_LEVEL   = 0x0020;
static constexpr uint16_t FTMS_IBD_FLAG_INST_POWER         = 0x0040;

// =====================================================
// CYCLING SPEED & CADENCE (CSC 0x1816) FLAGS
// =====================================================

static constexpr uint16_t CSC_FEATURE_WHEEL_REV_DATA       = 0x0001;
static constexpr uint16_t CSC_FEATURE_CRANK_REV_DATA       = 0x0002;

static constexpr uint8_t  CSC_MEASUREMENT_WHEEL_REV_PRESENT= 0x01;
static constexpr uint8_t  CSC_MEASUREMENT_CRANK_REV_PRESENT= 0x02;
