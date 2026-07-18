/**
 * SPDX-License-Identifier: GPL-3.0-only
 *
 * TrainerBridge BLE trainer proxy - Stable queued-control version
 *
 * Real trainer:
 *   Van Rysel FTMS 0x1826
 *   Indoor Bike Data 0x2AD2
 *   Fitness Machine Control Point 0x2AD9
 *
 * Virtual proxy exposed by ESP32:
 *   1. Fitness Machine Service 0x1826 for Zwift/MyWhoosh/TrainerDay
 *   2. Cycling Power Service 0x1818 for Garmin
 *
 * Garmin receives:
 *   - Cycling Power Measurement 0x2A63
 *   - Power and virtual wheel revolutions
 *   - No cadence
 *
 * App receives:
 *   - FTMS Indoor Bike Data 0x2AD2
 *   - FTMS Control Point 0x2AD9
 *
 * Main stability fix:
 *   App writes to virtual Control Point -> command is queued.
 *   loop() forwards queued command to real trainer.
 *   We do NOT write to the real trainer inside the BLE server callback.
 */

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "Config.h"

// =====================================================
// UUID
// =====================================================

static NimBLEUUID UUID_FTMS_SERVICE("1826");
static NimBLEUUID UUID_CPS_SERVICE("1818");
static NimBLEUUID UUID_DIS_SERVICE("180A");

// FTMS
static NimBLEUUID UUID_FTMS_FEATURE("2ACC");
static NimBLEUUID UUID_INDOOR_BIKE_DATA("2AD2");
static NimBLEUUID UUID_SUPPORTED_RESISTANCE_RANGE("2AD6");
static NimBLEUUID UUID_SUPPORTED_POWER_RANGE("2AD8");
static NimBLEUUID UUID_FTMS_CONTROL_POINT("2AD9");
static NimBLEUUID UUID_FTMS_STATUS("2ADA");

// Cycling Power
static NimBLEUUID UUID_CYCLING_POWER_MEASUREMENT("2A63");
static NimBLEUUID UUID_CYCLING_POWER_FEATURE("2A65");
static NimBLEUUID UUID_SENSOR_LOCATION("2A5D");

// Device Information
static NimBLEUUID UUID_MANUFACTURER_NAME("2A29");
static NimBLEUUID UUID_MODEL_NUMBER("2A24");

// =====================================================
// REAL TRAINER CLIENT STATE
// =====================================================

static bool realConnected = false;
static bool doScan = false;
static bool scanning = false;
static bool doConnectReal = false;

static NimBLEAdvertisedDevice *realDevice = nullptr;
static NimBLEClient *realClient = nullptr;

static NimBLERemoteCharacteristic *realIndoorBikeDataChr = nullptr;
static NimBLERemoteCharacteristic *realControlPointChr = nullptr;

// =====================================================
// VIRTUAL SERVER STATE
// =====================================================

static NimBLEServer *proxyServer = nullptr;

static NimBLECharacteristic *virtualCpsMeasurementChr = nullptr;
static NimBLECharacteristic *virtualCpsFeatureChr = nullptr;
static NimBLECharacteristic *virtualCpsSensorLocationChr = nullptr;

static NimBLECharacteristic *virtualFtmsFeatureChr = nullptr;
static NimBLECharacteristic *virtualIndoorBikeDataChr = nullptr;
static NimBLECharacteristic *virtualSupportedPowerRangeChr = nullptr;
static NimBLECharacteristic *virtualSupportedResistanceRangeChr = nullptr;
static NimBLECharacteristic *virtualControlPointChr = nullptr;
static NimBLECharacteristic *virtualStatusChr = nullptr;

static bool proxyAdvertisingStartedOnce = false;
static bool proxyReady = false;

// =====================================================
// LIVE DATA
// =====================================================

static int16_t realPowerW = 0;
static int16_t outputPowerW = 0;
static float powerScale = DEFAULT_POWER_SCALE;
static int32_t smoothedOutputPowerQ8 = 0;
static bool hasSmoothedOutputPower = false;
static uint16_t realSpeedRaw = 0; // 0.01 km/h
static int16_t realResistance = 0;

static float virtualSpeedMps = 0.0f;
static float virtualWheelRemainder = 0.0f;
static uint32_t virtualCumulativeWheelRevs = 0;
static uint16_t virtualLastWheelEventTime = 0;
static unsigned long lastVirtualWheelUpdateAt = 0;

static uint32_t packetsFromTrainer = 0;
static uint32_t packetsToGarmin = 0;
static uint32_t packetsToApp = 0;

static unsigned long lastScanAt = 0;
static unsigned long lastTrainerPacketAt = 0;

#if DEBUG_LOG
static unsigned long lastDebugAt = 0;
#endif

// Pending FTMS control command response
static bool pendingCpResponse = false;
static uint8_t pendingCpOpcode = 0x00;
static unsigned long pendingCpSince = 0;

static int16_t pendingTargetPower = 0;
static int16_t activeTargetPower = 0;

// Queued app command.
// This avoids writing to the real trainer inside a BLE callback.
static bool queuedAppCommand = false;
static uint8_t queuedAppCommandData[20];
static size_t queuedAppCommandLen = 0;
static uint8_t queuedAppCommandOpcode = 0x00;
static int16_t queuedTargetPower = 0;

// =====================================================
// UTILITY
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

static String lowerString(String s)
{
  s.toLowerCase();
  return s;
}

static bool stringContainsIgnoreCase(String source, String needle)
{
  if (needle.length() == 0) return false;

  source.toLowerCase();
  needle.toLowerCase();

  return source.indexOf(needle) >= 0;
}

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

static bool addressesMatch(const String &a, const String &b)
{
  return lowerString(a) == lowerString(b);
}

static bool matchesTrainerAdvertisement(
  const String &name,
  const String &address,
  bool advertisesFtms,
  const char *targetNameContains,
  const char *targetMac
)
{
  if (strlen(targetMac) > 0)
  {
    return addressesMatch(address, String(targetMac));
  }

  if (strlen(targetNameContains) > 0)
  {
    return stringContainsIgnoreCase(name, targetNameContains);
  }

  return advertisesFtms;
}

static bool trainerMatcherSelfCheck()
{
  return matchesTrainerAdvertisement("VanRysel D500", "AA:BB", false, "RYSEL", "") &&
         matchesTrainerAdvertisement("Van Rysel D500", "AA:BB", false, "RYSEL", "") &&
         matchesTrainerAdvertisement("Varysel D500", "AA:BB", false, "RYSEL", "") &&
         !matchesTrainerAdvertisement("Other trainer", "AA:BB", true, "RYSEL", "") &&
         matchesTrainerAdvertisement("Other", "AA:BB", false, "VANRYSEL", "aa:bb") &&
         !matchesTrainerAdvertisement("VanRysel D500", "CC:DD", true, "VANRYSEL", "aa:bb") &&
         matchesTrainerAdvertisement("Other", "AA:BB", true, "", "");
}

static int16_t getOutputPowerW()
{
  int16_t watts = (int16_t)(realPowerW * powerScale);

  if (!SMOOTH_OUTPUT_POWER || watts <= 0)
  {
    smoothedOutputPowerQ8 = ((int32_t)watts) << 8;
    hasSmoothedOutputPower = watts > 0;
    return watts;
  }

  int32_t wattsQ8 = ((int32_t)watts) << 8;

  if (!hasSmoothedOutputPower)
  {
    smoothedOutputPowerQ8 = wattsQ8;
    hasSmoothedOutputPower = true;
    return watts;
  }

  smoothedOutputPowerQ8 += (wattsQ8 - smoothedOutputPowerQ8) / (1 << POWER_SMOOTHING_SHIFT);

  return (int16_t)((smoothedOutputPowerQ8 + 128) >> 8);
}

static float getVirtualSpeedMps(int16_t watts)
{
  if (watts <= VIRTUAL_STOP_POWER_W) return 0.0f;

  const float aerodynamicFactor = 0.5f * VIRTUAL_AIR_DENSITY_KG_M3 * VIRTUAL_CDA;
  const float rollingForce = VIRTUAL_CRR * VIRTUAL_TOTAL_MASS_KG * VIRTUAL_GRAVITY_M_S2;
  const float wheelPower = watts * VIRTUAL_DRIVETRAIN_EFFICIENCY;
  float speed = 8.0f;

  for (uint8_t i = 0; i < 6; i++)
  {
    float speedSquared = speed * speed;
    float error = aerodynamicFactor * speedSquared * speed + rollingForce * speed - wheelPower;
    float derivative = 3.0f * aerodynamicFactor * speedSquared + rollingForce;

    speed -= error / derivative;
  }

  return speed > 0.0f ? speed : 0.0f;
}

static void updateVirtualWheel(int16_t watts, unsigned long now)
{
  if (lastVirtualWheelUpdateAt != 0 && virtualSpeedMps > 0.0f)
  {
    unsigned long elapsedMs = now - lastVirtualWheelUpdateAt;

    virtualWheelRemainder +=
      virtualSpeedMps * (elapsedMs / 1000.0f) / VIRTUAL_WHEEL_CIRCUMFERENCE_M;

    uint32_t completedRevolutions = (uint32_t)virtualWheelRemainder;

    if (completedRevolutions > 0)
    {
      virtualCumulativeWheelRevs += completedRevolutions;
      virtualWheelRemainder -= completedRevolutions;

      unsigned long msSinceLastRevolution = (unsigned long)(
        virtualWheelRemainder * VIRTUAL_WHEEL_CIRCUMFERENCE_M /
        virtualSpeedMps * 1000.0f
      );

      virtualLastWheelEventTime = (uint16_t)(
        ((uint64_t)(now - msSinceLastRevolution) * 2048ULL) / 1000ULL
      );
    }
  }

  virtualSpeedMps = getVirtualSpeedMps(watts);
  lastVirtualWheelUpdateAt = now;
}

static void stopVirtualWheel(unsigned long now)
{
  virtualSpeedMps = 0.0f;
  lastVirtualWheelUpdateAt = now;
}

static bool virtualSpeedModelSelfCheck()
{
  float speedAt150Kmh = getVirtualSpeedMps(150) * 3.6f;

  return getVirtualSpeedMps(0) == 0.0f &&
         speedAt150Kmh >= 29.5f &&
         speedAt150Kmh <= 30.0f;
}

// =====================================================
// ADVERTISING
// =====================================================

static void startProxyAdvertising()
{
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();

  if (!adv->isAdvertising())
  {
    adv->start();
    proxyAdvertisingStartedOnce = true;
    LOGLN("Advertising proxy avviato");
  }
}

static void stopProxyAdvertising()
{
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();

  if (adv->isAdvertising())
  {
    adv->stop();
    LOGLN("Advertising proxy fermato");
  }
}

// =====================================================
// LOCAL NOTIFICATIONS
// =====================================================

static void notifyGarminCyclingPower()
{
  if (virtualCpsMeasurementChr == nullptr) return;

  // Cycling Power Measurement 0x2A63:
  // flags 0x0010 = Instantaneous Power + Wheel Revolution Data.
  // No cadence.
  uint8_t cp[10];

  writeU16(cp, 0, 0x0010);
  writeS16(cp, 2, outputPowerW);
  writeU32(cp, 4, virtualCumulativeWheelRevs);
  writeU16(cp, 8, virtualLastWheelEventTime);

  virtualCpsMeasurementChr->setValue(cp, sizeof(cp));
  virtualCpsMeasurementChr->notify();

  packetsToGarmin++;
}

static void notifyAppIndoorBikeData()
{
  if (virtualIndoorBikeDataChr == nullptr) return;

  // Indoor Bike Data 0x2AD2.
  // flags 0x0040:
  // - bit 0 = 0 -> instantaneous speed present
  // - bit 6 = 1 -> instantaneous power present
  // No cadence.
  uint8_t ftms[6];

  writeU16(ftms, 0, 0x0040);
  writeU16(ftms, 2, realSpeedRaw);
  writeS16(ftms, 4, outputPowerW);

  virtualIndoorBikeDataChr->setValue(ftms, sizeof(ftms));
  virtualIndoorBikeDataChr->notify();

  packetsToApp++;
}

static void notifyBothImmediately()
{
  outputPowerW = getOutputPowerW();
  updateVirtualWheel(outputPowerW, millis());

  notifyGarminCyclingPower();
  notifyAppIndoorBikeData();
}

static void notifyFtmsStatusNewPower(int16_t watts)
{
  if (virtualStatusChr == nullptr) return;

  uint8_t status[3];

  // Best-effort status for new target power.
  status[0] = 0x08;
  writeS16(status, 1, watts);

  virtualStatusChr->setValue(status, sizeof(status));
  virtualStatusChr->notify();
}

static void notifyFtmsStatusStarted()
{
  if (virtualStatusChr == nullptr) return;

  uint8_t status[1] = {0x04};

  virtualStatusChr->setValue(status, sizeof(status));
  virtualStatusChr->notify();
}

static void notifyFtmsStatusStopped()
{
  if (virtualStatusChr == nullptr) return;

  uint8_t status[1] = {0x02};

  virtualStatusChr->setValue(status, sizeof(status));
  virtualStatusChr->notify();
}

static void notifyFtmsStatusIndoorSimulation(const uint8_t *cmd, size_t len)
{
  if (virtualStatusChr == nullptr) return;
  if (len < 7) return;

  uint8_t status[7];

  status[0] = 0x12;
  status[1] = cmd[1];
  status[2] = cmd[2];
  status[3] = cmd[3];
  status[4] = cmd[4];
  status[5] = cmd[5];
  status[6] = cmd[6];

  virtualStatusChr->setValue(status, sizeof(status));
  virtualStatusChr->notify();
}

// =====================================================
// REAL TRAINER FTMS PARSER
// =====================================================

static bool parseRealIndoorBikeData(const uint8_t *pData, size_t length)
{
  if (length < 2) return false;

  uint16_t flags = readU16(pData, 0);
  size_t index = 2;
  uint16_t parsedSpeedRaw = realSpeedRaw;
  int16_t parsedResistance = realResistance;
  int16_t parsedPowerW = realPowerW;
  bool powerPresent = false;

  bool speedPresent = ((flags & 0x0001) == 0);

  if (speedPresent)
  {
    if (index + 2 > length) return false;

    parsedSpeedRaw = readU16(pData, index);
    index += 2;
  }

  if (flags & 0x0002)
  {
    if (index + 2 > length) return false;
    index += 2;
  }

  if (flags & 0x0004)
  {
    if (index + 2 > length) return false;
    index += 2;
  }

  if (flags & 0x0008)
  {
    if (index + 2 > length) return false;
    index += 2;
  }

  if (flags & 0x0010)
  {
    if (index + 3 > length) return false;
    index += 3;
  }

  if (flags & 0x0020)
  {
    if (index + 2 > length) return false;

    parsedResistance = readS16(pData, index);
    index += 2;
  }

  if (flags & 0x0040)
  {
    if (index + 2 > length) return false;

    parsedPowerW = readS16(pData, index);
    powerPresent = true;
  }

  if (!powerPresent) return false;

  realSpeedRaw = parsedSpeedRaw;
  realResistance = parsedResistance;
  realPowerW = parsedPowerW;

  return true;
}

// =====================================================
// CONTROL POINT RESPONSE
// =====================================================

static void sendVirtualControlPointResponse(uint8_t requestedOpcode, uint8_t resultCode)
{
  if (virtualControlPointChr == nullptr) return;

  uint8_t resp[3] = {0x80, requestedOpcode, resultCode};

  printBytes("Virtual CP response", resp, sizeof(resp));

  virtualControlPointChr->setValue(resp, sizeof(resp));
  virtualControlPointChr->indicate();

  pendingCpResponse = false;
}

static void forwardRealControlPointResponseToApp(uint8_t *pData, size_t length)
{
  if (virtualControlPointChr == nullptr) return;
  if (length < 3) return;

  printBytes("Forward real CP response to app", pData, length);

  virtualControlPointChr->setValue(pData, length);
  virtualControlPointChr->indicate();

  if (pData[0] == 0x80)
  {
    uint8_t opcode = pData[1];
    uint8_t result = pData[2];

    if (result == 0x01)
    {
      if (opcode == 0x05)
      {
        activeTargetPower = pendingTargetPower;
        notifyFtmsStatusNewPower(activeTargetPower);
      }
      else if (opcode == 0x07)
      {
        notifyFtmsStatusStarted();
      }
      else if (opcode == 0x08)
      {
        notifyFtmsStatusStopped();
      }
    }
  }

  pendingCpResponse = false;
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
  if (!parseRealIndoorBikeData(pData, length)) return;

  packetsFromTrainer++;
  lastTrainerPacketAt = millis();

  // Low latency: forward immediately.
  notifyBothImmediately();
}

static void realControlPointCallback(
  NimBLERemoteCharacteristic *chr,
  uint8_t *pData,
  size_t length,
  bool isNotify
)
{
  printBytes("Real CP response", pData, length);

  forwardRealControlPointResponseToApp(pData, length);
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
// QUEUED COMMAND PROCESSING
// =====================================================

static void processQueuedAppCommand()
{
  if (!queuedAppCommand)
  {
    return;
  }

  if (!realConnected || realControlPointChr == nullptr)
  {
    sendVirtualControlPointResponse(queuedAppCommandOpcode, 0x04);
    queuedAppCommand = false;
    return;
  }

  if (pendingCpResponse)
  {
    return;
  }

  uint8_t opcode = queuedAppCommandOpcode;

  if (opcode == 0x05 && queuedAppCommandLen >= 3)
  {
    pendingTargetPower = queuedTargetPower;
  }
  else if (opcode == 0x11 && queuedAppCommandLen >= 7)
  {
    notifyFtmsStatusIndoorSimulation(queuedAppCommandData, queuedAppCommandLen);
  }

  bool ok = writeRealControlPoint(queuedAppCommandData, queuedAppCommandLen);

  if (!ok)
  {
    sendVirtualControlPointResponse(opcode, 0x04);
    queuedAppCommand = false;
    return;
  }

  pendingCpResponse = true;
  pendingCpOpcode = opcode;
  pendingCpSince = millis();

  queuedAppCommand = false;
}

// =====================================================
// VIRTUAL CONTROL POINT CALLBACK
// =====================================================

class VirtualControlPointCallbacks : public NimBLECharacteristicCallbacks
{
  void onWrite(NimBLECharacteristic *chr, NimBLEConnInfo &connInfo)
  {
    std::string value = chr->getValue();

    if (value.length() < 1)
    {
      return;
    }

    const uint8_t *cmd = (const uint8_t *)value.data();
    size_t len = value.length();
    uint8_t opcode = cmd[0];

    if (len > sizeof(queuedAppCommandData))
    {
      sendVirtualControlPointResponse(opcode, 0x03);
      return;
    }

    if (queuedAppCommand || pendingCpResponse)
    {
      sendVirtualControlPointResponse(opcode, 0x04);
      return;
    }

    if (!realConnected || realControlPointChr == nullptr)
    {
      sendVirtualControlPointResponse(opcode, 0x04);
      return;
    }

    memcpy(queuedAppCommandData, cmd, len);

    queuedAppCommandLen = len;
    queuedAppCommandOpcode = opcode;

    if (opcode == 0x05 && len >= 3)
    {
      queuedTargetPower = readS16(cmd, 1);
    }
    else
    {
      queuedTargetPower = 0;
    }

    queuedAppCommand = true;
  }

  void onSubscribe(NimBLECharacteristic *chr, NimBLEConnInfo &connInfo, uint16_t subValue)
  {
    LOG("Virtual Control Point subscribe: ");
    LOGLN(subValue);
  }

  void onStatus(NimBLECharacteristic *chr, NimBLEConnInfo &connInfo, int code)
  {
    LOG("Virtual Control Point indication status code: ");
    LOGLN(code);
  }
};

static VirtualControlPointCallbacks virtualControlPointCallbacks;

// =====================================================
// GENERIC CALLBACKS
// =====================================================

class GenericCharacteristicCallbacks : public NimBLECharacteristicCallbacks
{
  void onSubscribe(NimBLECharacteristic *chr, NimBLEConnInfo &connInfo, uint16_t subValue)
  {
    LOG("Subscribe ");
    LOG(chr->getUUID().toString().c_str());
    LOG(" = ");
    LOGLN(subValue);
  }
};

static GenericCharacteristicCallbacks genericCallbacks;

// =====================================================
// SERVER CALLBACKS
// =====================================================

class ProxyServerCallbacks : public NimBLEServerCallbacks
{
  void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo)
  {
    LOGLN("Client connesso al proxy");

    server->updateConnParams(
      connInfo.getConnHandle(),
      CONN_INTERVAL_MIN,
      CONN_INTERVAL_MAX,
      CONN_LATENCY,
      CONN_TIMEOUT
    );

    // Continue advertising for Garmin + app.
    if (server->getConnectedCount() < 2)
    {
      startProxyAdvertising();
    }
  }

  void onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo, int reason)
  {
    LOG("Client disconnesso dal proxy, reason=");
    LOGLN(reason);

    startProxyAdvertising();
  }

  void onMTUChange(uint16_t MTU, NimBLEConnInfo &connInfo)
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
  void onConnect(NimBLEClient *client)
  {
    LOGLN("Connesso al Van Rysel reale");
  }

  void onDisconnect(NimBLEClient *client, int reason)
  {
    LOG("Disconnesso dal Van Rysel reale, reason=");
    LOGLN(reason);

    realConnected = false;
    proxyReady = false;

    realIndoorBikeDataChr = nullptr;
    realControlPointChr = nullptr;

    pendingCpResponse = false;
    queuedAppCommand = false;
    hasSmoothedOutputPower = false;
    realPowerW = 0;
    outputPowerW = 0;
    realSpeedRaw = 0;
    realResistance = 0;
    lastTrainerPacketAt = 0;
    stopVirtualWheel(millis());

    stopProxyAdvertising();

    doScan = true;
    lastScanAt = millis() + 3000;
  }

  void onMTUChange(NimBLEClient *client, uint16_t MTU)
  {
    LOG("Real trainer MTU changed: ");
    LOGLN(MTU);
  }

  bool onConnParamsUpdateRequest(NimBLEClient *client, const ble_gap_upd_params *params)
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
  if (realDevice == nullptr)
  {
    LOGLN("realDevice null");
    return false;
  }

  LOGLN("Connessione al Van Rysel reale...");

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

  if (!realClient->connect(realDevice))
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
  void onResult(const NimBLEAdvertisedDevice *advertisedDevice)
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

    bool matchesTarget = matchesTrainerAdvertisement(
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

      if (realDevice != nullptr)
      {
        delete realDevice;
        realDevice = nullptr;
      }

      realDevice = new NimBLEAdvertisedDevice(*advertisedDevice);

      doConnectReal = true;
    }
  }

  void onScanEnd(const NimBLEScanResults &results, int reason)
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

  // Wheel Revolution Data supported; not a distributed power sensor.
  uint8_t cpFeature[4];
  writeU32(cpFeature, 0, 0x00100004);
  virtualCpsFeatureChr->setValue(cpFeature, sizeof(cpFeature));

  // Sensor location.
  uint8_t sensorLocation[1] = {0x0D};
  virtualCpsSensorLocationChr->setValue(sensorLocation, sizeof(sensorLocation));

  // Initial measurement: wheel data present, all values at zero.
  uint8_t cpInitial[10] = {0};
  writeU16(cpInitial, 0, 0x0010);
  virtualCpsMeasurementChr->setValue(cpInitial, sizeof(cpInitial));

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

  // Fitness Machine Feature, 8 bytes.
  // Pragmatic indoor-bike feature flags.
  uint32_t machineFeatures = 0x00004080;
  uint32_t targetFeatures = 0x0000200C;

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

  // Supported Power Range 0x2AD8:
  // sint16 min, sint16 max, uint16 increment.
  uint8_t powerRange[6];

  writeS16(powerRange, 0, POWER_MIN_W);
  writeS16(powerRange, 2, POWER_MAX_W);
  writeU16(powerRange, 4, POWER_STEP_W);

  virtualSupportedPowerRangeChr->setValue(powerRange, sizeof(powerRange));

  // Supported Resistance Level Range 0x2AD6:
  // sint16 min, sint16 max, uint16 increment.
  uint8_t resistanceRange[6];

  writeS16(resistanceRange, 0, RESISTANCE_MIN);
  writeS16(resistanceRange, 2, RESISTANCE_MAX);
  writeU16(resistanceRange, 4, RESISTANCE_STEP);

  virtualSupportedResistanceRangeChr->setValue(resistanceRange, sizeof(resistanceRange));

  // Initial Indoor Bike Data:
  // flags 0x0040, speed 0, power 0.
  uint8_t indoorInitial[6];

  writeU16(indoorInitial, 0, 0x0040);
  writeU16(indoorInitial, 2, 0);
  writeS16(indoorInitial, 4, 0);

  virtualIndoorBikeDataChr->setValue(indoorInitial, sizeof(indoorInitial));

  // Initial Control Point response
  uint8_t cpInitial[3] = {0x80, 0x00, 0x01};
  virtualControlPointChr->setValue(cpInitial, sizeof(cpInitial));

  // Initial Status idle
  uint8_t statusInitial[1] = {0x00};
  virtualStatusChr->setValue(statusInitial, sizeof(statusInitial));

}

static void setupProxyServer()
{
  proxyServer = NimBLEDevice::createServer();
  proxyServer->setCallbacks(&proxyServerCallbacks);
  proxyServer->advertiseOnDisconnect(true);

  setupDeviceInformationService();
  setupCyclingPowerService();
  setupFitnessMachineService();

  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();

  adv->reset();
  adv->setName(DEVICE_NAME);

  // Best-effort cycling/trainer-ish appearance.
  adv->setAppearance(0x0480);

  adv->addServiceUUID(UUID_FTMS_SERVICE);
  adv->addServiceUUID(UUID_CPS_SERVICE);

  // FTMS service data, useful for app discovery.
  uint8_t ftmsServiceData[3] = {0x01, 0x20, 0x00};
  adv->setServiceData(UUID_FTMS_SERVICE, ftmsServiceData, sizeof(ftmsServiceData));

  adv->enableScanResponse(true);
  adv->setMinInterval(32);
  adv->setMaxInterval(64);

  Serial.println("Proxy GATT server pronto");

  // Critical fix:
  // Start advertising once at boot, then stop it.
  // This initializes GAP advertising before using ESP32 as BLE client.
  startProxyAdvertising();

  delay(300);

  stopProxyAdvertising();

  Serial.println("Advertising proxy inizializzato e fermato in attesa del rullo reale");
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
// SERIAL STATUS
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
  Serial.println(realPowerW);

  Serial.print("outputPowerW: ");
  Serial.println(outputPowerW);

  Serial.print("speedRaw 0.01kmh: ");
  Serial.println(realSpeedRaw);

  Serial.print("virtualSpeedKmh: ");
  Serial.println(virtualSpeedMps * 3.6f, 2);

  Serial.print("virtualDistanceKm: ");
  Serial.println(
    (virtualCumulativeWheelRevs + virtualWheelRemainder) *
      VIRTUAL_WHEEL_CIRCUMFERENCE_M / 1000.0f,
    3
  );

  Serial.print("virtualWheelRevs: ");
  Serial.println(virtualCumulativeWheelRevs);

  Serial.print("activeTargetPower: ");
  Serial.println(activeTargetPower);

  Serial.print("packetsFromTrainer: ");
  Serial.println(packetsFromTrainer);

  Serial.print("packetsToGarmin: ");
  Serial.println(packetsToGarmin);

  Serial.print("packetsToApp: ");
  Serial.println(packetsToApp);

  Serial.print("pendingCpResponse: ");
  Serial.println(pendingCpResponse ? "true" : "false");

  Serial.print("queuedAppCommand: ");
  Serial.println(queuedAppCommand ? "true" : "false");

  Serial.print("proxyAdvertisingStartedOnce: ");
  Serial.println(proxyAdvertisingStartedOnce ? "true" : "false");

  Serial.println("==============================");
  Serial.println();
}

static void handleSerial()
{
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');

  cmd.trim();
  cmd.toLowerCase();

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
    float s = cmd.substring(5).toFloat();

    if (s > 0.1 && s < 5.0)
    {
      powerScale = s;

      Serial.print("powerScale=");
      Serial.println(powerScale);
    }
  }
  else if (cmd.startsWith("p"))
  {
    int watts = cmd.substring(1).toInt();

    if (watts > 0 && watts <= POWER_MAX_W && realConnected && realControlPointChr != nullptr)
    {
      uint8_t c[3];

      c[0] = 0x05;
      writeS16(c, 1, watts);

      pendingTargetPower = watts;

      writeRealControlPoint(c, sizeof(c));
    }
  }
  else
  {
    Serial.println("Comandi: status, adv, noadv, scale1.28, p150");
  }
}

// =====================================================
// SETUP / LOOP
// =====================================================

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("TrainerBridge boot");

  Serial.print("Virtual speed model self-check: ");
  Serial.println(virtualSpeedModelSelfCheck() ? "OK" : "FAILED");

  Serial.print("Trainer matcher self-check: ");
  Serial.println(trainerMatcherSelfCheck() ? "OK" : "FAILED");

  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(185);

  setupProxyServer();
  setupScanner();

  Serial.println("Setup completato");
}

void loop()
{
  handleSerial();

  // Process app command outside BLE callback.
  processQueuedAppCommand();

  if (doConnectReal)
  {
    doConnectReal = false;

    // Critical:
    // Re-enable advertising BEFORE connecting as BLE client to the real trainer.
    startProxyAdvertising();

    delay(100);

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

  if (!realConnected && doScan && !scanning && millis() >= lastScanAt)
  {
    Serial.println("Avvio scan Van Rysel reale...");

    stopProxyAdvertising();

    NimBLEScan *scan = NimBLEDevice::getScan();

    scan->clearResults();

    scanning = true;
    doScan = false;

    scan->start(8000, false, true);
  }

  if (realConnected && outputPowerW != 0 && lastTrainerPacketAt > 0 && millis() - lastTrainerPacketAt > POWER_STALE_TIMEOUT_MS)
  {
    realPowerW = 0;
    outputPowerW = 0;
    hasSmoothedOutputPower = false;
    stopVirtualWheel(millis());
    notifyGarminCyclingPower();
    notifyAppIndoorBikeData();
  }

  // Timeout Control Point response.
  // Avoid leaving Zwift/MyWhoosh hanging forever.
  if (pendingCpResponse && millis() - pendingCpSince > 1200)
  {
    sendVirtualControlPointResponse(pendingCpOpcode, 0x04);
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
    Serial.print(outputPowerW);

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
