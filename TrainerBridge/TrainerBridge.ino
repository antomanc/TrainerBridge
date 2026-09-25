/**
 * SPDX-License-Identifier: GPL-3.0-only
 *
 * TrainerBridge BLE trainer proxy
 *
 * Real trainer:
 *   Van Rysel / FTMS 0x1826
 *   Indoor Bike Data 0x2AD2
 *   Fitness Machine Control Point 0x2AD9
 *
 * Virtual proxy exposed by ESP32:
 *   1. Fitness Machine Service 0x1826 for Zwift/MyWhoosh/TrainerDay
 *   2. Cycling Power Service 0x1818 for Garmin Power
 *   3. Cycling Speed & Cadence Service 0x1816 for Garmin Speed & Distance
 *
 * Garmin receives:
 *   - Cycling Power Measurement 0x2A63 (Watts)
 *   - Cycling Speed Measurement 0x2A5B (Native Speed & Distance)
 *
 * App receives:
 *   - FTMS Indoor Bike Data 0x2AD2
 *   - FTMS Control Point 0x2AD9
 *
 * Architecture:
 *   - Modular components: FtmsConstants, PowerFilter, CommandQueue, TrainerMatcher, VirtualSpeedModel
 *   - Physics-based flat-terrain aerodynamic and rolling resistance model
 *   - Standard 1/1024s CSC wheel event timing for Garmin Edge/watches
 *   - Flywheel coasting deceleration for realistic momentum
 *   - Thread-safe FreeRTOS coalescing command queue
 *   - Non-blocking serial CLI
 *   - Brownout-safe BLE power profile (+6 dBm)
 *   - Deterministic Control Point state machine (zero double-indication violations)
 *   - Value-based BLE address handling (zero heap leaks)
 */

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "Config.h"
#include "FtmsConstants.h"
#include "PowerFilter.h"
#include "CommandQueue.h"
#include "TrainerMatcher.h"
#include "VirtualSpeedModel.h"

// =====================================================
// LOGGING MACROS
// =====================================================

#if DEBUG_LOG
  #define LOG(x) Serial.print(x)
  #define LOGLN(x) Serial.println(x)
  #define LOGF(...) Serial.printf(__VA_ARGS__)
#else
  #define LOG(x)
  #define LOGLN(x)
  #define LOGF(...)
#endif

// =====================================================
// CONTROL POINT TRANSACTION STATE MACHINE
// =====================================================

enum class CpTxState : uint8_t {
  IDLE,
  WAITING_RESPONSE,
  TIMED_OUT
};

// =====================================================
// GLOBAL MODULES & STATE
// =====================================================

static PowerFilter powerFilter(
  DEFAULT_POWER_SCALE,
  SMOOTH_OUTPUT_POWER,
  POWER_SMOOTHING_SHIFT,
  POWER_STALE_TIMEOUT_MS
);

static CommandQueue commandQueue;
static VirtualSpeedModel virtualSpeedModel;

// Real trainer client state
static volatile bool realConnected = false;
static volatile bool doScan = false;
static volatile bool scanning = false;
static volatile bool doConnectReal = false;
static volatile bool proxyReady = false;

static NimBLEAddress realTrainerAddress;
static NimBLEClient *realClient = nullptr;
static NimBLERemoteCharacteristic *realIndoorBikeDataChr = nullptr;
static NimBLERemoteCharacteristic *realControlPointChr = nullptr;

// Virtual server state
static NimBLEServer *proxyServer = nullptr;

// Cycling Power (0x1818)
static NimBLECharacteristic *virtualCpsMeasurementChr = nullptr;
static NimBLECharacteristic *virtualCpsFeatureChr = nullptr;
static NimBLECharacteristic *virtualCpsSensorLocationChr = nullptr;

// Cycling Speed and Cadence (0x1816) for native Garmin speed & distance
static NimBLECharacteristic *virtualCscMeasurementChr = nullptr;
static NimBLECharacteristic *virtualCscFeatureChr = nullptr;

// Fitness Machine Service (0x1826)
static NimBLECharacteristic *virtualFtmsFeatureChr = nullptr;
static NimBLECharacteristic *virtualIndoorBikeDataChr = nullptr;
static NimBLECharacteristic *virtualSupportedPowerRangeChr = nullptr;
static NimBLECharacteristic *virtualSupportedResistanceRangeChr = nullptr;
static NimBLECharacteristic *virtualControlPointChr = nullptr;
static NimBLECharacteristic *virtualStatusChr = nullptr;

static bool proxyAdvertisingStartedOnce = false;

// Control Point transaction tracking
static volatile CpTxState cpTxState = CpTxState::IDLE;
static uint8_t pendingCpOpcode = 0x00;
static uint16_t pendingCpConnHandle = CommandQueue::NO_CONN_HANDLE;
static unsigned long pendingCpSince = 0;
static int16_t pendingTargetPower = 0;
static int16_t activeTargetPower = 0;
static uint8_t pendingSimCommandData[CommandQueue::MAX_PAYLOAD_LEN];
static size_t pendingSimCommandLen = 0;

// Packet counters and diagnostics
static uint32_t packetsFromTrainer = 0;
static uint32_t packetsToGarmin = 0;
static uint32_t packetsToApp = 0;

static unsigned long lastScanAt = 0;

#if DEBUG_LOG
static unsigned long lastDebugAt = 0;
#endif

// Non-blocking serial buffer
static char serialRxBuf[64];
static size_t serialRxLen = 0;

// =====================================================
// UTILITY FUNCTIONS
// =====================================================

static int16_t readS16(const uint8_t *data, size_t index)
{
  return (int16_t)(data[index] | (data[index + 1] << 8));
}

static uint16_t readU16(const uint8_t *data, size_t index)
{
  return (uint16_t)(data[index] | (data[index + 1] << 8));
}

static void writeS16(uint8_t *data, size_t index, int16_t value)
{
  data[index] = value & 0xff;
  data[index + 1] = (value >> 8) & 0xff;
}

static void writeU16(uint8_t *data, size_t index, uint16_t value)
{
  data[index] = value & 0xff;
  data[index + 1] = (value >> 8) & 0xff;
}

static void writeU24(uint8_t *data, size_t index, uint32_t value)
{
  data[index] = value & 0xff;
  data[index + 1] = (value >> 8) & 0xff;
  data[index + 2] = (value >> 16) & 0xff;
}

static void writeU32(uint8_t *data, size_t index, uint32_t value)
{
  data[index] = value & 0xff;
  data[index + 1] = (value >> 8) & 0xff;
  data[index + 2] = (value >> 16) & 0xff;
  data[index + 3] = (value >> 24) & 0xff;
}

static void printBytes(const char *label, const uint8_t *data, size_t len)
{
#if DEBUG_LOG
  Serial.print(label);
  Serial.print(": ");

  for (size_t i = 0; i < len; i++)
  {
    if (data[i] < 16) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }

  Serial.println();
#endif
}

// =====================================================
// ADVERTISING MANAGEMENT
// =====================================================

static void startProxyAdvertising()
{
  if (!proxyReady) return;

  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();

  if (!adv->isAdvertising())
  {
    static unsigned long lastAdvAttemptAt = 0;
    if (millis() - lastAdvAttemptAt >= 500)
    {
      lastAdvAttemptAt = millis();
      if (adv->start())
      {
        proxyAdvertisingStartedOnce = true;
        Serial.println("[ADV] Advertising proxy avviato");
      }
    }
  }
}

static void stopProxyAdvertising()
{
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();

  if (adv->isAdvertising())
  {
    adv->stop();
    Serial.println("[ADV] Advertising proxy fermato");
  }
}

// =====================================================
// GATT SERVER NOTIFICATIONS
// =====================================================

static void notifyGarminCyclingPower(int16_t watts)
{
  if (virtualCpsMeasurementChr == nullptr) return;

  if (ENABLE_VIRTUAL_SPEED)
  {
    // Cycling Power Measurement (0x2A63):
    // Flags 0x0010 = Instantaneous Power + Wheel Revolution Data
    uint8_t cp[10];
    writeU16(cp, 0, 0x0010);
    writeS16(cp, 2, watts);
    writeU32(cp, 4, virtualSpeedModel.getCumulativeRevs());
    writeU16(cp, 8, virtualSpeedModel.getLastWheelEventTime2048());

    virtualCpsMeasurementChr->setValue(cp, sizeof(cp));
    virtualCpsMeasurementChr->notify(cp, sizeof(cp));
  }
  else
  {
    uint8_t cp[4];
    writeU16(cp, 0, 0x0000);
    writeS16(cp, 2, watts);

    virtualCpsMeasurementChr->setValue(cp, sizeof(cp));
    virtualCpsMeasurementChr->notify(cp, sizeof(cp));
  }

  packetsToGarmin++;
}

static void notifyGarminCyclingSpeed()
{
  if (virtualCscMeasurementChr == nullptr || !ENABLE_CSC_SERVICE || !ENABLE_VIRTUAL_SPEED) return;

  // CSC Measurement (0x2A5B): 7 bytes
  // Byte 0: Flags (0x01 = Wheel Revolution Data Present)
  // Bytes 1..4: Cumulative Wheel Revolutions (uint32)
  // Bytes 5..6: Last Wheel Event Time in 1/1024s resolution (uint16)
  uint8_t csc[7];
  csc[0] = CSC_MEASUREMENT_WHEEL_REV_PRESENT; // 0x01
  writeU32(csc, 1, virtualSpeedModel.getCumulativeRevs());
  writeU16(csc, 5, virtualSpeedModel.getLastWheelEventTime1024());

  virtualCscMeasurementChr->setValue(csc, sizeof(csc));
  virtualCscMeasurementChr->notify(csc, sizeof(csc));
}

static void notifyAppIndoorBikeData(int16_t watts)
{
  if (virtualIndoorBikeDataChr == nullptr) return;

  if (ENABLE_VIRTUAL_SPEED)
  {
    uint8_t ftms[9];
    writeU16(ftms, 0, FTMS_IBD_FLAG_TOTAL_DISTANCE | FTMS_IBD_FLAG_INST_POWER); // 0x0050: speed present, distance present, power present
    uint16_t speed001Kmh = (uint16_t)(virtualSpeedModel.getSpeedKmh() * 100.0f + 0.5f);
    writeU16(ftms, 2, speed001Kmh);
    writeU24(ftms, 4, virtualSpeedModel.getDistanceMeters());
    writeS16(ftms, 7, watts);

    virtualIndoorBikeDataChr->setValue(ftms, sizeof(ftms));
    virtualIndoorBikeDataChr->notify(ftms, sizeof(ftms));
  }
  else
  {
    uint8_t ftms[4];
    writeU16(ftms, 0, FTMS_IBD_FLAG_MORE_DATA | FTMS_IBD_FLAG_INST_POWER); // 0x0041
    writeS16(ftms, 2, watts);

    virtualIndoorBikeDataChr->setValue(ftms, sizeof(ftms));
    virtualIndoorBikeDataChr->notify(ftms, sizeof(ftms));
  }

  packetsToApp++;
}

static void notifyBoth(int16_t watts)
{
  notifyGarminCyclingPower(watts);
  notifyGarminCyclingSpeed();
  notifyAppIndoorBikeData(watts);
}

static void notifyFtmsStatusNewPower(int16_t watts)
{
  if (virtualStatusChr == nullptr) return;

  uint8_t status[3];
  status[0] = FTMS_STATUS_TARGET_POWER_CHANGED;
  writeS16(status, 1, watts);

  virtualStatusChr->setValue(status, sizeof(status));
  virtualStatusChr->notify(status, sizeof(status));
}

static void notifyFtmsStatusStarted()
{
  if (virtualStatusChr == nullptr) return;

  uint8_t status[1] = {FTMS_STATUS_STARTED};
  virtualStatusChr->setValue(status, sizeof(status));
  virtualStatusChr->notify(status, sizeof(status));
}

static void notifyFtmsStatusStopped()
{
  if (virtualStatusChr == nullptr) return;

  uint8_t status[2] = {FTMS_STATUS_STOPPED, 0x01}; // 0x01 = STOP per Bluetooth SIG FTMS spec
  virtualStatusChr->setValue(status, sizeof(status));
  virtualStatusChr->notify(status, sizeof(status));
}

static void notifyFtmsStatusIndoorSimulation(const uint8_t *cmd, size_t len)
{
  if (virtualStatusChr == nullptr || len < 7) return;

  uint8_t status[7];
  status[0] = FTMS_STATUS_INDOOR_BIKE_SIM_CHANGED;
  memcpy(&status[1], &cmd[1], 6);

  virtualStatusChr->setValue(status, sizeof(status));
  virtualStatusChr->notify(status, sizeof(status));
}

static void sendVirtualControlPointResponse(uint8_t requestedOpcode, uint8_t resultCode, uint16_t connHandle = CommandQueue::NO_CONN_HANDLE)
{
  if (virtualControlPointChr == nullptr) return;

  uint8_t resp[3] = {FTMS_CP_OP_RESPONSE_CODE, requestedOpcode, resultCode};

  Serial.printf("[CP RESP] opcode=0x%02X, res=0x%02X, connHandle=%d\n",
    requestedOpcode, resultCode, connHandle);

  virtualControlPointChr->setValue(resp, sizeof(resp));
  if (connHandle != CommandQueue::NO_CONN_HANDLE)
  {
    bool indOk = virtualControlPointChr->indicate(resp, sizeof(resp), connHandle);
    Serial.printf("[CP INDICATE] connHandle=%d, ok=%d\n", connHandle, indOk ? 1 : 0);
  }
  else
  {
    bool indOk = virtualControlPointChr->indicate(resp, sizeof(resp));
    Serial.printf("[CP INDICATE ALL] ok=%d\n", indOk ? 1 : 0);
  }
}

// =====================================================
// WRITE TO REAL CONTROL POINT
// =====================================================

static bool writeRealControlPoint(const uint8_t *data, size_t length)
{
  if (!realConnected || realControlPointChr == nullptr)
  {
    LOGLN("Real Control Point non disponibile");
    return false;
  }

  if (!realControlPointChr->canWrite())
  {
    LOGLN("Real Control Point non scrivibile");
    return false;
  }

  printBytes("Write real CP", data, length);

  bool ok = realControlPointChr->writeValue(
    (uint8_t *)data,
    length,
    REAL_CP_WRITE_WITH_RESPONSE
  );

  LOG("Write real CP result: ");
  LOGLN(ok ? "OK" : "FAILED");

  return ok;
}

// =====================================================
// REAL TRAINER FTMS PARSER
// =====================================================

static bool parseRealIndoorBikeData(const uint8_t *pData, size_t length, int16_t &outPowerW)
{
  if (length < 2) return false;

  uint16_t flags = readU16(pData, 0);
  size_t index = 2;
  bool powerPresent = false;
  int16_t parsedPowerW = 0;

  // More Data bit 0: when clear, Instantaneous Speed is present (uint16)
  if ((flags & FTMS_IBD_FLAG_MORE_DATA) == 0)
  {
    if (index + 2 > length) return false;
    index += 2;
  }

  // Average Speed
  if (flags & FTMS_IBD_FLAG_AVG_SPEED)
  {
    if (index + 2 > length) return false;
    index += 2;
  }

  // Instantaneous Cadence
  if (flags & FTMS_IBD_FLAG_INST_CADENCE)
  {
    if (index + 2 > length) return false;
    index += 2;
  }

  // Average Cadence
  if (flags & FTMS_IBD_FLAG_AVG_CADENCE)
  {
    if (index + 2 > length) return false;
    index += 2;
  }

  // Total Distance
  if (flags & FTMS_IBD_FLAG_TOTAL_DISTANCE)
  {
    if (index + 3 > length) return false;
    index += 3;
  }

  // Resistance Level
  if (flags & FTMS_IBD_FLAG_RESISTANCE_LEVEL)
  {
    if (index + 2 > length) return false;
    index += 2;
  }

  // Instantaneous Power
  if (flags & FTMS_IBD_FLAG_INST_POWER)
  {
    if (index + 2 > length) return false;
    parsedPowerW = readS16(pData, index);
    powerPresent = true;
  }

  if (!powerPresent) return false;

  outPowerW = parsedPowerW;
  return true;
}

// =====================================================
// REAL TRAINER CALLBACKS
// =====================================================

static void realIndoorBikeDataCallback(
  NimBLERemoteCharacteristic *chr,
  uint8_t *pData,
  size_t length,
  bool isNotify
)
{
  int16_t rawPower = 0;
  if (!parseRealIndoorBikeData(pData, length, rawPower)) return;

  packetsFromTrainer++;

  int16_t filteredWatts = powerFilter.update(rawPower, millis());

  if (ENABLE_VIRTUAL_SPEED)
  {
    virtualSpeedModel.update(filteredWatts, millis());
  }

  static bool ftmsStartedNotified = false;
  if (filteredWatts > 0 && !ftmsStartedNotified)
  {
    notifyFtmsStatusStarted();
    ftmsStartedNotified = true;
  }
  else if (filteredWatts == 0 && ftmsStartedNotified && powerFilter.isStale(millis()))
  {
    notifyFtmsStatusStopped();
    ftmsStartedNotified = false;
  }

  notifyBoth(filteredWatts);
}

static void realControlPointCallback(
  NimBLERemoteCharacteristic *chr,
  uint8_t *pData,
  size_t length,
  bool isNotify
)
{
  printBytes("Real CP response", pData, length);

  if (length < 3 || pData[0] != FTMS_CP_OP_RESPONSE_CODE) return;

  uint8_t opcode = pData[1];
  uint8_t result = pData[2];

  // If the command already timed out, do not send a duplicate indication to app
  if (cpTxState == CpTxState::TIMED_OUT)
  {
    LOGLN("Late real CP indication dropped after timeout");
    cpTxState = CpTxState::IDLE;
    return;
  }

  if (cpTxState == CpTxState::WAITING_RESPONSE)
  {
    cpTxState = CpTxState::IDLE;

    // Forward the real trainer's confirmation only to the client that sent the command
    if (virtualControlPointChr != nullptr)
    {
      Serial.printf("[REAL CP RESP] opcode=0x%02X, res=0x%02X, targetConnHandle=%d\n",
        opcode, result, pendingCpConnHandle);

      virtualControlPointChr->setValue(pData, length);
      if (pendingCpConnHandle != CommandQueue::NO_CONN_HANDLE)
      {
        virtualControlPointChr->indicate(pData, length, pendingCpConnHandle);
      }
    }
    pendingCpConnHandle = CommandQueue::NO_CONN_HANDLE;

    // Emit status notifications ONLY after real trainer confirms success
    if (result == FTMS_CP_RES_SUCCESS)
    {
      if (opcode == FTMS_CP_OP_SET_TARGET_POWER)
      {
        activeTargetPower = pendingTargetPower;
        notifyFtmsStatusNewPower(activeTargetPower);
      }
      else if (opcode == FTMS_CP_OP_SET_INDOOR_BIKE_SIM && pendingSimCommandLen >= 7)
      {
        notifyFtmsStatusIndoorSimulation(pendingSimCommandData, pendingSimCommandLen);
      }
      else if (opcode == FTMS_CP_OP_START_RESUME)
      {
        notifyFtmsStatusStarted();
      }
      else if (opcode == FTMS_CP_OP_STOP_PAUSE)
      {
        notifyFtmsStatusStopped();
      }
    }
  }
}

// =====================================================
// VIRTUAL CONTROL POINT CALLBACK
// =====================================================

class VirtualControlPointCallbacks : public NimBLECharacteristicCallbacks
{
  void onWrite(NimBLECharacteristic *chr, NimBLEConnInfo &connInfo) override
  {
    std::string value = chr->getValue();

    if (value.length() < 1)
    {
      return;
    }

    const uint8_t *cmd = (const uint8_t *)value.data();
    size_t len = value.length();
    uint8_t opcode = cmd[0];

    uint16_t clientConnHandle = connInfo.getConnHandle();

    Serial.printf("[CP WRITE] connHandle=%d, opcode=0x%02X, len=%d\n",
      clientConnHandle, opcode, (int)len);

    if (len > CommandQueue::MAX_PAYLOAD_LEN)
    {
      sendVirtualControlPointResponse(opcode, FTMS_CP_RES_INVALID_PARAMETER, clientConnHandle);
      return;
    }

    if (!realConnected || realControlPointChr == nullptr)
    {
      sendVirtualControlPointResponse(opcode, FTMS_CP_RES_OPERATION_FAILED, clientConnHandle);
      return;
    }

    // Immediate handling of control and state transitions to prevent multi-master timeouts
    if (opcode == FTMS_CP_OP_REQUEST_CONTROL)
    {
      sendVirtualControlPointResponse(opcode, FTMS_CP_RES_SUCCESS, clientConnHandle);
      notifyFtmsStatusStarted();
      return;
    }

    if (opcode == FTMS_CP_OP_START_RESUME)
    {
      sendVirtualControlPointResponse(opcode, FTMS_CP_RES_SUCCESS, clientConnHandle);
      notifyFtmsStatusStarted();
      return;
    }

    if (opcode == FTMS_CP_OP_RESET)
    {
      sendVirtualControlPointResponse(opcode, FTMS_CP_RES_SUCCESS, clientConnHandle);
      if (virtualStatusChr != nullptr)
      {
        uint8_t st[1] = {FTMS_STATUS_RESET};
        virtualStatusChr->setValue(st, sizeof(st));
        virtualStatusChr->notify(st, sizeof(st));
      }
      return;
    }

    int16_t targetVal = 0;
    if (opcode == FTMS_CP_OP_SET_TARGET_POWER && len >= 3)
    {
      targetVal = readS16(cmd, 1);
    }

    // Push into thread-safe coalescing command queue
    if (!commandQueue.push(cmd, len, opcode, targetVal, clientConnHandle))
    {
      // Queue is full and could not coalesce
      sendVirtualControlPointResponse(opcode, FTMS_CP_RES_OPERATION_FAILED, clientConnHandle);
    }
  }

  void onSubscribe(NimBLECharacteristic *chr, NimBLEConnInfo &connInfo, uint16_t subValue) override
  {
    Serial.printf("[CP SUBSCRIBE] connHandle=%d, subValue=%d\n",
      connInfo.getConnHandle(), subValue);
  }

  void onStatus(NimBLECharacteristic *chr, NimBLEConnInfo &connInfo, int code) override
  {
    Serial.printf("[CP IND STATUS] connHandle=%d, code=%d\n",
      connInfo.getConnHandle(), code);
  }
};

static VirtualControlPointCallbacks virtualControlPointCallbacks;

// =====================================================
// GENERIC CALLBACKS
// =====================================================

class GenericCharacteristicCallbacks : public NimBLECharacteristicCallbacks
{
  void onSubscribe(NimBLECharacteristic *chr, NimBLEConnInfo &connInfo, uint16_t subValue) override
  {
    Serial.printf("[SUBSCRIBE] chr=%s, connHandle=%d, val=%d\n",
      chr->getUUID().toString().c_str(), connInfo.getConnHandle(), subValue);
  }
};

static GenericCharacteristicCallbacks genericCallbacks;

// =====================================================
// SERVER CALLBACKS
// =====================================================

class ProxyServerCallbacks : public NimBLEServerCallbacks
{
  void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) override
  {
    Serial.printf("[BLE] Connect: connHandle=%d, addr=%s, total=%d\n",
      connInfo.getConnHandle(),
      connInfo.getAddress().toString().c_str(),
      server->getConnectedCount());
  }

  void onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo, int reason) override
  {
    Serial.printf("[BLE] Disconnect: connHandle=%d, addr=%s, reason=0x%04X, remaining=%d\n",
      connInfo.getConnHandle(),
      connInfo.getAddress().toString().c_str(),
      reason,
      server->getConnectedCount());

    if (pendingCpConnHandle == connInfo.getConnHandle())
    {
      pendingCpConnHandle = CommandQueue::NO_CONN_HANDLE;
    }
  }

  void onMTUChange(uint16_t MTU, NimBLEConnInfo &connInfo) override
  {
    LOG("Proxy MTU changed: ");
    LOGLN(MTU);
  }
};

static ProxyServerCallbacks proxyServerCallbacks;

// =====================================================
// REAL CLIENT CALLBACKS
// =====================================================

class RealClientCallbacks : public NimBLEClientCallbacks
{
  void onConnect(NimBLEClient *client) override
  {
    LOGLN("Connesso al trainer reale");
  }

  void onDisconnect(NimBLEClient *client, int reason) override
  {
    LOG("Disconnesso dal trainer reale, reason=");
    LOGLN(reason);

    realConnected = false;
    proxyReady = false;

    realIndoorBikeDataChr = nullptr;
    realControlPointChr = nullptr;

    cpTxState = CpTxState::IDLE;
    commandQueue.clear();
    powerFilter.reset();
    virtualSpeedModel.stop(millis());

    // Notify connected clients with 0 W so Garmin and Zwift do not freeze on old power
    notifyBoth(0);

    stopProxyAdvertising();

    doScan = true;
    lastScanAt = millis() + 3000;
  }

  void onMTUChange(NimBLEClient *client, uint16_t MTU) override
  {
    LOG("Real trainer MTU changed: ");
    LOGLN(MTU);
  }

  bool onConnParamsUpdateRequest(NimBLEClient *client, const ble_gap_upd_params *params) override
  {
    return true;
  }
};

static RealClientCallbacks realClientCallbacks;

// =====================================================
// CONNECT TO REAL TRAINER
// =====================================================

static bool connectToRealTrainer()
{
  LOGLN("Connessione al trainer reale...");

  if (realClient == nullptr)
  {
    realClient = NimBLEDevice::createClient();
  }

  if (realClient == nullptr)
  {
    LOGLN("Impossibile creare realClient");
    return false;
  }

  realClient->setClientCallbacks(&realClientCallbacks, false);
  realClient->setConnectionParams(
    CONN_INTERVAL_MIN,
    CONN_INTERVAL_MAX,
    CONN_LATENCY,
    CONN_TIMEOUT
  );
  realClient->setConnectTimeout(10000);

  // Connect using value-stored address (zero dynamic heap allocation)
  if (!realClient->connect(realTrainerAddress))
  {
    LOGLN("Connessione al trainer reale fallita");

    NimBLEDevice::deleteClient(realClient);
    realClient = nullptr;

    return false;
  }

  NimBLERemoteService *ftmsService = realClient->getService(UUID_FTMS_SERVICE);

  if (ftmsService == nullptr)
  {
    LOGLN("Service FTMS 0x1826 non trovato sul trainer reale");

    realClient->disconnect();
    NimBLEDevice::deleteClient(realClient);
    realClient = nullptr;

    return false;
  }

  LOGLN("Service FTMS reale trovato");

  realIndoorBikeDataChr = ftmsService->getCharacteristic(UUID_INDOOR_BIKE_DATA);

  if (realIndoorBikeDataChr == nullptr)
  {
    LOGLN("Indoor Bike Data 0x2AD2 non trovato sul trainer reale");

    realClient->disconnect();
    NimBLEDevice::deleteClient(realClient);
    realClient = nullptr;

    return false;
  }

  if (!realIndoorBikeDataChr->canNotify())
  {
    LOGLN("Indoor Bike Data reale non supporta notify");

    realClient->disconnect();
    NimBLEDevice::deleteClient(realClient);
    realClient = nullptr;

    return false;
  }

  if (!realIndoorBikeDataChr->subscribe(true, realIndoorBikeDataCallback))
  {
    LOGLN("Subscribe Indoor Bike Data reale fallita");

    realClient->disconnect();
    NimBLEDevice::deleteClient(realClient);
    realClient = nullptr;

    return false;
  }

  LOGLN("Subscribe Indoor Bike Data reale OK");

  realControlPointChr = ftmsService->getCharacteristic(UUID_FTMS_CONTROL_POINT);

  if (realControlPointChr == nullptr)
  {
    LOGLN("Control Point reale 0x2AD9 non trovato");
  }
  else
  {
    LOG("Real CP canWrite=");
    LOGLN(realControlPointChr->canWrite() ? "true" : "false");

    LOG("Real CP canIndicate=");
    LOGLN(realControlPointChr->canIndicate() ? "true" : "false");

    LOG("Real CP canNotify=");
    LOGLN(realControlPointChr->canNotify() ? "true" : "false");

    if (realControlPointChr->canIndicate())
    {
      if (realControlPointChr->subscribe(false, realControlPointCallback))
      {
        LOGLN("Subscribe Real Control Point indications OK");
      }
      else
      {
        LOGLN("Subscribe Real Control Point indications FAILED");
      }
    }
    else if (realControlPointChr->canNotify())
    {
      if (realControlPointChr->subscribe(true, realControlPointCallback))
      {
        LOGLN("Subscribe Real Control Point notifications OK");
      }
      else
      {
        LOGLN("Subscribe Real Control Point notifications FAILED");
      }
    }

    if (realControlPointChr->canWrite())
    {
      uint8_t reqCtrl[1] = {FTMS_CP_OP_REQUEST_CONTROL};
      writeRealControlPoint(reqCtrl, sizeof(reqCtrl));
    }
  }

  realConnected = true;
  proxyReady = true;

  LOGLN("Trainer reale connesso. Proxy pronto per Garmin/Zwift/MyWhoosh.");

  return true;
}

// =====================================================
// SCAN CALLBACKS
// =====================================================

class RealScanCallbacks : public NimBLEScanCallbacks
{
  void onResult(const NimBLEAdvertisedDevice *advertisedDevice) override
  {
    String name = advertisedDevice->getName().c_str();
    String address = advertisedDevice->getAddress().toString().c_str();

#if DEBUG_LOG
    Serial.print("BLE trovato | Nome=");
    Serial.print(name.length() ? name : "(senza nome)");
    Serial.print(" | Address=");
    Serial.print(address);
    Serial.print(" | RSSI=");
    Serial.print(advertisedDevice->getRSSI());

    if (advertisedDevice->haveServiceUUID())
    {
      Serial.print(" | Service UUID=");
      Serial.print(advertisedDevice->getServiceUUID().toString().c_str());
    }

    Serial.println();
#endif

    bool matchByUUID =
      advertisedDevice->haveServiceUUID() &&
      advertisedDevice->isAdvertisingService(UUID_FTMS_SERVICE);

    bool matchesTarget = TrainerMatcher::matches(
      name,
      address,
      matchByUUID,
      TARGET_NAME_CONTAINS,
      TARGET_MAC
    );

    if (matchesTarget)
    {
      LOGLN("TRAINER TARGET TROVATO");

      Serial.print("Trainer matched: ");
      Serial.print(name.length() ? name : "(no name)");
      Serial.print(" | ");
      Serial.println(address);

      NimBLEDevice::getScan()->stop();

      scanning = false;
      doScan = false;

      // Copy address by value (eliminates heap allocation & leaks)
      realTrainerAddress = advertisedDevice->getAddress();
      doConnectReal = true;
    }
  }

  void onScanEnd(const NimBLEScanResults &results, int reason) override
  {
    scanning = false;

    LOG("Scan finito. Count=");
    LOG(results.getCount());
    LOG(" reason=");
    LOGLN(reason);

    if (!realConnected && !doConnectReal)
    {
      doScan = true;
      lastScanAt = millis() + 3000;
    }
  }
};

static RealScanCallbacks realScanCallbacks;

// =====================================================
// SETUP LOCAL GATT SERVER
// =====================================================

static void setupDeviceInformationService()
{
  NimBLEService *dis = proxyServer->createService(UUID_DIS_SERVICE);

  NimBLECharacteristic *manufacturer = dis->createCharacteristic(
    UUID_MANUFACTURER_NAME,
    NIMBLE_PROPERTY::READ
  );

  NimBLECharacteristic *model = dis->createCharacteristic(
    UUID_MODEL_NUMBER,
    NIMBLE_PROPERTY::READ
  );

  manufacturer->setValue(DEVICE_NAME);
  model->setValue("FTMS-CPS Bridge");
}

static void setupCyclingPowerService()
{
  NimBLEService *cps = proxyServer->createService(UUID_CPS_SERVICE);

  virtualCpsFeatureChr = cps->createCharacteristic(
    UUID_CYCLING_POWER_FEATURE,
    NIMBLE_PROPERTY::READ
  );

  virtualCpsSensorLocationChr = cps->createCharacteristic(
    UUID_SENSOR_LOCATION,
    NIMBLE_PROPERTY::READ
  );

  virtualCpsMeasurementChr = cps->createCharacteristic(
    UUID_CYCLING_POWER_MEASUREMENT,
    NIMBLE_PROPERTY::NOTIFY
  );

  virtualCpsMeasurementChr->setCallbacks(&genericCallbacks);

  // Cycling Power Feature 0x2A65:
  // Bit 2 = 1 (Wheel Revolution Data Supported)
  // Bits 20-21 = 0b01 (Not for use in a distributed system)
  // Value: 0x00100004
  uint8_t cpFeature[4];
  writeU32(cpFeature, 0, 0x00100004);
  virtualCpsFeatureChr->setValue(cpFeature, sizeof(cpFeature));

  // Sensor location: SENSOR_LOCATION_OTHER (0x00) prevents erroneous pedal calibration popups
  uint8_t sensorLocation[1] = {CPS_SENSOR_LOCATION};
  virtualCpsSensorLocationChr->setValue(sensorLocation, sizeof(sensorLocation));

  // Initial measurement: instantaneous power 0, wheel revolutions 0
  if (ENABLE_VIRTUAL_SPEED)
  {
    uint8_t cpInitial[10] = {0};
    writeU16(cpInitial, 0, 0x0010); // flags: Wheel Revolution Data present
    virtualCpsMeasurementChr->setValue(cpInitial, sizeof(cpInitial));
  }
  else
  {
    uint8_t cpInitial[4] = {0};
    virtualCpsMeasurementChr->setValue(cpInitial, sizeof(cpInitial));
  }
}

static void setupCyclingSpeedAndCadenceService()
{
  NimBLEService *csc = proxyServer->createService(UUID_CSC_SERVICE);

  virtualCscFeatureChr = csc->createCharacteristic(
    UUID_CSC_FEATURE,
    NIMBLE_PROPERTY::READ
  );

  virtualCscMeasurementChr = csc->createCharacteristic(
    UUID_CSC_MEASUREMENT,
    NIMBLE_PROPERTY::NOTIFY
  );

  virtualCscMeasurementChr->setCallbacks(&genericCallbacks);

  // CSC Feature 0x2A5C: Wheel Revolution Data Supported (bit 0 = 1)
  uint8_t cscFeature[2];
  writeU16(cscFeature, 0, CSC_FEATURE_WHEEL_REV_DATA); // 0x0001
  virtualCscFeatureChr->setValue(cscFeature, sizeof(cscFeature));

  // Sensor Location 0x2A5D: Other (0x00)
  NimBLECharacteristic *cscSensorLocChr = csc->createCharacteristic(
    UUID_SENSOR_LOCATION,
    NIMBLE_PROPERTY::READ
  );
  uint8_t cscLoc[1] = {SENSOR_LOCATION_OTHER};
  cscSensorLocChr->setValue(cscLoc, sizeof(cscLoc));

  // Initial measurement: flags 0x01 (wheel data present), 7 bytes
  uint8_t cscInitial[7] = {0};
  cscInitial[0] = CSC_MEASUREMENT_WHEEL_REV_PRESENT; // 0x01
  virtualCscMeasurementChr->setValue(cscInitial, sizeof(cscInitial));
}

static void setupFitnessMachineService()
{
  NimBLEService *ftms = proxyServer->createService(UUID_FTMS_SERVICE);

  virtualFtmsFeatureChr = ftms->createCharacteristic(
    UUID_FTMS_FEATURE,
    NIMBLE_PROPERTY::READ
  );

  virtualIndoorBikeDataChr = ftms->createCharacteristic(
    UUID_INDOOR_BIKE_DATA,
    NIMBLE_PROPERTY::NOTIFY
  );

  virtualSupportedResistanceRangeChr = ftms->createCharacteristic(
    UUID_SUPPORTED_RESISTANCE_RANGE,
    NIMBLE_PROPERTY::READ
  );

  virtualSupportedPowerRangeChr = ftms->createCharacteristic(
    UUID_SUPPORTED_POWER_RANGE,
    NIMBLE_PROPERTY::READ
  );

  virtualControlPointChr = ftms->createCharacteristic(
    UUID_FTMS_CONTROL_POINT,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE
  );

  virtualStatusChr = ftms->createCharacteristic(
    UUID_FTMS_STATUS,
    NIMBLE_PROPERTY::NOTIFY
  );

  virtualIndoorBikeDataChr->setCallbacks(&genericCallbacks);
  virtualStatusChr->setCallbacks(&genericCallbacks);
  virtualControlPointChr->setCallbacks(&virtualControlPointCallbacks);

  // Fitness Machine Feature: 8 bytes
  uint32_t machineFeatures = 0x00004080; // Power + Resistance supported
  if (ENABLE_VIRTUAL_SPEED)
  {
    machineFeatures |= 0x00000004; // Bit 2: Total Distance Supported
  }
  uint32_t targetFeatures = 0x0000200C;  // Target Power, Resistance, and Simulation supported

  uint8_t featureData[8];
  featureData[0] = machineFeatures & 0xff;
  featureData[1] = (machineFeatures >> 8) & 0xff;
  featureData[2] = (machineFeatures >> 16) & 0xff;
  featureData[3] = (machineFeatures >> 24) & 0xff;

  featureData[4] = targetFeatures & 0xff;
  featureData[5] = (targetFeatures >> 8) & 0xff;
  featureData[6] = (targetFeatures >> 16) & 0xff;
  featureData[7] = (targetFeatures >> 24) & 0xff;

  virtualFtmsFeatureChr->setValue(featureData, sizeof(featureData));

  // Supported Power Range 0x2AD8
  uint8_t powerRange[6];
  writeS16(powerRange, 0, POWER_MIN_W);
  writeS16(powerRange, 2, POWER_MAX_W);
  writeU16(powerRange, 4, POWER_STEP_W);
  virtualSupportedPowerRangeChr->setValue(powerRange, sizeof(powerRange));

  // Supported Resistance Level Range 0x2AD6
  uint8_t resistanceRange[6];
  writeS16(resistanceRange, 0, RESISTANCE_MIN);
  writeS16(resistanceRange, 2, RESISTANCE_MAX);
  writeU16(resistanceRange, 4, RESISTANCE_STEP);
  virtualSupportedResistanceRangeChr->setValue(resistanceRange, sizeof(resistanceRange));

  // Initial Indoor Bike Data
  if (ENABLE_VIRTUAL_SPEED)
  {
    uint8_t indoorInitial[9] = {0};
    writeU16(indoorInitial, 0, FTMS_IBD_FLAG_TOTAL_DISTANCE | FTMS_IBD_FLAG_INST_POWER);
    virtualIndoorBikeDataChr->setValue(indoorInitial, sizeof(indoorInitial));
  }
  else
  {
    uint8_t indoorInitial[4];
    writeU16(indoorInitial, 0, FTMS_IBD_FLAG_MORE_DATA | FTMS_IBD_FLAG_INST_POWER);
    writeS16(indoorInitial, 2, 0);
    virtualIndoorBikeDataChr->setValue(indoorInitial, sizeof(indoorInitial));
  }

  // Initial Control Point response
  uint8_t cpInitial[3] = {FTMS_CP_OP_RESPONSE_CODE, 0x00, FTMS_CP_RES_SUCCESS};
  virtualControlPointChr->setValue(cpInitial, sizeof(cpInitial));

  // Initial Status: Reset (0x01 per Bluetooth SIG FTMS spec)
  uint8_t statusInitial[1] = {FTMS_STATUS_RESET};
  virtualStatusChr->setValue(statusInitial, sizeof(statusInitial));
}

static void setupProxyServer()
{
  proxyServer = NimBLEDevice::createServer();
  proxyServer->setCallbacks(&proxyServerCallbacks);
  proxyServer->advertiseOnDisconnect(false);

  setupDeviceInformationService();
  setupCyclingPowerService();
  if (ENABLE_CSC_SERVICE)
  {
    setupCyclingSpeedAndCadenceService();
  }
  setupFitnessMachineService();

  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->reset();
  // Enable scan response before setting name so the local name goes to scan response,
  // ensuring that all 3 service UUIDs fit into the primary 31-byte advertising packet.
  adv->enableScanResponse(true);
  adv->setName(DEVICE_NAME);
  adv->setAppearance(0x0480); // Generic Cycling

  adv->addServiceUUID(UUID_FTMS_SERVICE);
  adv->addServiceUUID(UUID_CPS_SERVICE);
  if (ENABLE_CSC_SERVICE)
  {
    adv->addServiceUUID(UUID_CSC_SERVICE);
  }

  // FTMS service data for app discovery
  uint8_t ftmsServiceData[3] = {0x01, 0x20, 0x00};
  adv->setServiceData(UUID_FTMS_SERVICE, ftmsServiceData, sizeof(ftmsServiceData));

  adv->setMinInterval(32);
  adv->setMaxInterval(64);

  // Start the GATT server now while no connections are active, registering all services in NimBLE tables
  proxyServer->start();

  Serial.println("Proxy GATT server pronto");
}

// =====================================================
// SCANNER SETUP
// =====================================================

static void setupScanner()
{
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&realScanCallbacks, false);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(99);

  doScan = true;
  lastScanAt = millis();
}

// =====================================================
// COMMAND PROCESSING & STATE MACHINE
// =====================================================

static void processQueuedCommands()
{
  // If an acknowledged command is currently awaiting confirmation from the real trainer
  if (cpTxState == CpTxState::WAITING_RESPONSE)
  {
    if (millis() - pendingCpSince > CP_TIMEOUT_MS)
    {
      LOGLN("Control Point transaction timeout");
      cpTxState = CpTxState::TIMED_OUT;
      sendVirtualControlPointResponse(pendingCpOpcode, FTMS_CP_RES_OPERATION_FAILED, pendingCpConnHandle);
      pendingCpConnHandle = CommandQueue::NO_CONN_HANDLE;
    }
    return;
  }

  // Ready for next command
  QueuedCommand cmd;
  if (!commandQueue.pop(cmd))
  {
    return;
  }

  if (!realConnected || realControlPointChr == nullptr)
  {
    sendVirtualControlPointResponse(cmd.opcode, FTMS_CP_RES_OPERATION_FAILED, cmd.connHandle);
    return;
  }

  if (cmd.opcode == FTMS_CP_OP_SET_TARGET_POWER)
  {
    pendingTargetPower = cmd.targetValue;
  }
  else if (cmd.opcode == FTMS_CP_OP_SET_INDOOR_BIKE_SIM)
  {
    pendingSimCommandLen = cmd.len;
    memcpy(pendingSimCommandData, cmd.data, cmd.len);
  }

  bool ok = writeRealControlPoint(cmd.data, cmd.len);

  if (!ok)
  {
    sendVirtualControlPointResponse(cmd.opcode, FTMS_CP_RES_OPERATION_FAILED, cmd.connHandle);
    return;
  }

  cpTxState = CpTxState::WAITING_RESPONSE;
  pendingCpOpcode = cmd.opcode;
  pendingCpConnHandle = cmd.connHandle;
  pendingCpSince = millis();
}

// =====================================================
// SERIAL STATUS & NON-BLOCKING CLI
// =====================================================

static void printStatus()
{
  Serial.println();
  Serial.println("===== TRAINERBRIDGE STATUS =====");

  Serial.print("realConnected: ");
  Serial.println(realConnected ? "true" : "false");

  Serial.print("proxyReady: ");
  Serial.println(proxyReady ? "true" : "false");

  Serial.print("proxy clients: ");
  Serial.println(proxyServer ? proxyServer->getConnectedCount() : 0);

  Serial.print("trainerMatch: ");
  if (strlen(TARGET_MAC) > 0)
  {
    Serial.print("MAC equals ");
    Serial.println(TARGET_MAC);
  }
  else if (strlen(TARGET_NAME_CONTAINS) > 0)
  {
    Serial.print("name contains \"");
    Serial.print(TARGET_NAME_CONTAINS);
    Serial.println("\" (case-insensitive)");
  }
  else
  {
    Serial.println("first FTMS advertiser");
  }

  Serial.print("realPowerW: ");
  Serial.println(powerFilter.getLastRaw());

  Serial.print("outputPowerW: ");
  Serial.println(powerFilter.getLastOutput());

  Serial.print("virtualSpeedKmh: ");
  Serial.println(virtualSpeedModel.getSpeedKmh(), 2);

  Serial.print("virtualDistanceKm: ");
  Serial.println(virtualSpeedModel.getDistanceKm(), 3);

  Serial.print("cumulativeWheelRevs: ");
  Serial.println(virtualSpeedModel.getCumulativeRevs());

  Serial.print("powerScale: ");
  Serial.println(powerFilter.getScale(), 2);

  Serial.print("activeTargetPower: ");
  Serial.println(activeTargetPower);

  Serial.print("packetsFromTrainer: ");
  Serial.println(packetsFromTrainer);

  Serial.print("packetsToGarmin: ");
  Serial.println(packetsToGarmin);

  Serial.print("packetsToApp: ");
  Serial.println(packetsToApp);

  Serial.print("queuedCommands: ");
  Serial.println(commandQueue.count());

  Serial.print("cpTxState: ");
  Serial.println(cpTxState == CpTxState::IDLE ? "IDLE" :
                 (cpTxState == CpTxState::WAITING_RESPONSE ? "WAITING_RESPONSE" : "TIMED_OUT"));

  Serial.print("proxyAdvertisingStartedOnce: ");
  Serial.println(proxyAdvertisingStartedOnce ? "true" : "false");

  Serial.println("==============================");
  Serial.println();
}

static void executeSerialCommand(const char *rawCmd)
{
  String cmd = String(rawCmd);
  cmd.trim();
  cmd.toLowerCase();

  if (cmd.length() == 0) return;

  if (cmd == "status")
  {
    printStatus();
  }
  else if (cmd == "adv")
  {
    startProxyAdvertising();
  }
  else if (cmd == "noadv")
  {
    stopProxyAdvertising();
  }
  else if (cmd.startsWith("scale"))
  {
    String numStr = cmd.substring(5);
    numStr.trim();
    float s = numStr.toFloat();

    if (s >= 0.1f && s <= 5.0f)
    {
      powerFilter.setScale(s);
      Serial.print("powerScale=");
      Serial.println(powerFilter.getScale(), 2);
    }
    else
    {
      Serial.println("Invalid scale (0.1 - 5.0)");
    }
  }
  else if (cmd.startsWith("p"))
  {
    String wattsStr = cmd.substring(1);
    wattsStr.trim();
    int watts = wattsStr.toInt();

    if (watts > 0 && watts <= POWER_MAX_W && realConnected && realControlPointChr != nullptr)
    {
      uint8_t c[3];
      c[0] = FTMS_CP_OP_SET_TARGET_POWER;
      writeS16(c, 1, (int16_t)watts);

      commandQueue.push(c, sizeof(c), FTMS_CP_OP_SET_TARGET_POWER, (int16_t)watts);
      Serial.print("Target power queued: ");
      Serial.println(watts);
    }
  }
  else
  {
    Serial.println("Comandi: status, adv, noadv, scale 1.28, p 150");
  }
}

static void handleSerialNonBlocking()
{
  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();

    if (c == '\r') continue;

    if (c == '\n')
    {
      if (serialRxLen > 0)
      {
        serialRxBuf[serialRxLen] = '\0';
        executeSerialCommand(serialRxBuf);
        serialRxLen = 0;
      }
    }
    else
    {
      if (serialRxLen < sizeof(serialRxBuf) - 1)
      {
        serialRxBuf[serialRxLen++] = c;
      }
    }
  }
}

// =====================================================
// SETUP & LOOP
// =====================================================

void setup()
{
  // Disable brownout detector to prevent resets caused by microsecond voltage dips when USB RF initializes
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("TrainerBridge boot");

  Serial.print("Virtual speed model self-check: ");
  Serial.println(VirtualSpeedModel::selfCheck() ? "OK" : "FAILED");

  Serial.print("Trainer matcher self-check: ");
  Serial.println(TrainerMatcher::selfCheck() ? "OK" : "FAILED");

  NimBLEDevice::init(DEVICE_NAME);

  // Set transmission power to P9 (+9 dBm): maximum range and link robustness
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(185);

  setupProxyServer();
  setupScanner();

  Serial.println("Setup completato");
}

void loop()
{
  handleSerialNonBlocking();

  // Process queued app commands outside BLE callback
  processQueuedCommands();

  // Connect to real trainer when matched
  if (doConnectReal)
  {
    doConnectReal = false;

    if (connectToRealTrainer())
    {
      Serial.println("Trainer reale connesso. Proxy visibile a Garmin/Zwift/MyWhoosh.");
    }
    else
    {
      Serial.println("Connect real trainer fallita. Riprovo scan tra 3 secondi.");

      realConnected = false;
      proxyReady = false;

      stopProxyAdvertising();

      doScan = true;
      lastScanAt = millis() + 3000;
    }
  }

  // Periodic trainer scan
  if (!realConnected && doScan && !scanning && millis() >= lastScanAt)
  {
    Serial.println("Avvio scan trainer reale...");

    stopProxyAdvertising();

    NimBLEScan *scan = NimBLEDevice::getScan();
    scan->clearResults();

    scanning = true;
    doScan = false;

    scan->start(8000, false, true);
  }

  // Keep proxy advertising active whenever fewer than 2 clients are connected
  if (realConnected && proxyReady && proxyServer != nullptr)
  {
    if (proxyServer->getConnectedCount() < 2)
    {
      startProxyAdvertising();
    }
    else
    {
      stopProxyAdvertising();
    }
  }

  // Power stale watchdog
  if (realConnected && powerFilter.getLastOutput() != 0 && powerFilter.isStale(millis()))
  {
    powerFilter.reset();
    virtualSpeedModel.stop(millis());
    notifyBoth(0);
  }

#if DEBUG_LOG
  if (millis() - lastDebugAt > 2000)
  {
    lastDebugAt = millis();

    Serial.print("Live | real=");
    Serial.print(realConnected ? "Y" : "N");

    Serial.print(" ready=");
    Serial.print(proxyReady ? "Y" : "N");

    Serial.print(" clients=");
    Serial.print(proxyServer ? proxyServer->getConnectedCount() : 0);

    Serial.print(" power=");
    Serial.print(powerFilter.getLastOutput());

    Serial.print(" target=");
    Serial.print(activeTargetPower);

    Serial.print(" in=");
    Serial.print(packetsFromTrainer);

    Serial.print(" garmin=");
    Serial.print(packetsToGarmin);

    Serial.print(" app=");
    Serial.println(packetsToApp);
  }
#endif

  delay(1);
}
