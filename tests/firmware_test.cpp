// Host regression tests: runs the real sketch and helper modules against Arduino/NimBLE
// test doubles. Tests share the firmware's global state and run in order, as one session:
// boot, scan, connect, ride, then repeated trainer disconnects.
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <vector>
#include "../TrainerBridge/TrainerBridge.ino"

// Unlike assert(), stays active under NDEBUG and names the failing test.
static const char* currentTest = "";
#define CHECK(cond) do { if(!(cond)) { \
  std::fprintf(stderr,"FAIL %s\n  %s:%d: CHECK(%s)\n",currentTest,__FILE__,__LINE__,#cond); \
  std::exit(1); } } while(0)

using Bytes = std::vector<uint8_t>;
static constexpr uint16_t APP = 11;    // e.g. Zwift
static constexpr uint16_t GARMIN = 22; // second subscriber

// ---------- helpers: drive firmware callbacks through the public NimBLE interfaces ----------

static NimBLEScan* scanner() { return NimBLEDevice::getScan(); }
static NimBLEAdvertisedDevice trainer;

static void advertise(const NimBLEAdvertisedDevice& device) {
  static_cast<NimBLEScanCallbacks*>(&realScanCallbacks)->onResult(&device);
}
static void completeScan() {
  static_cast<NimBLEScanCallbacks*>(&realScanCallbacks)->onScanEnd(NimBLEScanResults{},0);
}
static void disconnectTrainer() {
  static_cast<NimBLEClientCallbacks*>(&realClientCallbacks)->onDisconnect(realClient,19);
}
static void disconnectApp(uint16_t handle) {
  NimBLEConnInfo info(handle);
  static_cast<NimBLEServerCallbacks*>(&proxyServerCallbacks)->onDisconnect(proxyServer,info,19);
}
static void writeCp(const Bytes& data,uint16_t handle=APP) {
  virtualControlPointChr->setValue(data.data(),data.size());
  NimBLEConnInfo info(handle);
  static_cast<NimBLECharacteristicCallbacks*>(&virtualControlPointCallbacks)->onWrite(virtualControlPointChr,info);
}
static void reply(uint8_t opcode,uint8_t result=FTMS_CP_RES_SUCCESS) {
  uint8_t bytes[]={FTMS_CP_OP_RESPONSE_CODE,opcode,result};
  realControlPointCallback(nullptr,bytes,sizeof(bytes),false);
}
static void clearDeliveries() {
  for(auto* c:{virtualCpsMeasurementChr,virtualCscMeasurementChr,virtualIndoorBikeDataChr,virtualStatusChr,virtualControlPointChr}) {
    c->notifications.clear(); c->indications.clear();
  }
}
static size_t cpIndications() { return virtualControlPointChr->indications.size(); }
static bool lastCpIs(uint8_t opcode,uint8_t result,uint16_t handle) {
  if(virtualControlPointChr->indications.empty()) return false;
  const auto& d=virtualControlPointChr->indications.back();
  return d.handle==handle && d.bytes==Bytes{FTMS_CP_OP_RESPONSE_CODE,opcode,result};
}
static const Bytes& lastTrainerWrite() { return realControlPointChr->writes.back(); }

// ---------- tests ----------

static void moduleSelfChecks() {
  CHECK(TrainerMatcher::selfCheck());
  CHECK(StatusLed::selfCheck());
  CHECK(VirtualSpeedModel::selfCheck());
}

static void bootStartsAnEightSecondScan() {
  setup();
  CHECK(NimBLEDevice::initCalls==1);
  loop();
  CHECK(scanning && scanner()->starts==1 && scanner()->duration==8000 && scanner()->clears==1);
}

static void ignoresOtherDevicesAndRetriesAfterThreeSeconds() {
  NimBLEAdvertisedDevice other; other.name="Other trainer";
  advertise(other);
  CHECK(!doConnectReal && scanning);
  completeScan();
  CHECK(doScan && !scanning);
  const auto retry=lastScanAt;
  testNow=retry-1; loop(); CHECK(scanner()->starts==1);
  testNow=retry;   loop(); CHECK(scanner()->starts==2);
}

static void connectsToMatchingTrainer() {
  advertise(trainer);
  CHECK(doConnectReal && !doScan && !scanning);
  CHECK(realTrainerAddress.toString()==trainer.address.toString());
  completeScan(); CHECK(!doScan); // The stop after a match must not schedule another scan.
  loop();
  CHECK(realConnected && proxyReady && realClient);
  CHECK(realIndoorBikeDataChr->callback==realIndoorBikeDataCallback);
  CHECK(realControlPointChr->callback==realControlPointCallback);
}

static void requestsTrainerControlOnConnect() {
  CHECK((realControlPointChr->writes==std::vector<Bytes>{{FTMS_CP_OP_REQUEST_CONTROL}}));
  reply(FTMS_CP_OP_REQUEST_CONTROL); // Not an app transaction: nothing is forwarded.
  CHECK(cpIndications()==0 && cpTxState==CpTxState::IDLE);
}

static void pausesAdvertisingAtTwoClients() {
  proxyServer->connectedCount=2; loop(); CHECK(!NimBLEDevice::advertising.active);
  proxyServer->connectedCount=1; testNow+=500; loop(); CHECK(NimBLEDevice::advertising.active);
}

static void sendsIdenticalTelemetryToEverySubscriber() {
  clearDeliveries();
  uint8_t telemetry[]={0x40,0,0,0,200,0}; // Speed and instantaneous power.
  realIndoorBikeDataCallback(nullptr,telemetry,sizeof(telemetry),true);
  for(auto* c:{virtualCpsMeasurementChr,virtualCscMeasurementChr,virtualIndoorBikeDataChr}) {
    CHECK(c->notifications.size()==2);
    CHECK(c->notifications[0].handle==APP && c->notifications[1].handle==GARMIN);
    CHECK(c->notifications[0].bytes==c->notifications[1].bytes);
  }
  CHECK(readS16(virtualCpsMeasurementChr->notifications[0].bytes.data(),2)==200);
  CHECK(readS16(virtualIndoorBikeDataChr->notifications[0].bytes.data(),7)==200);
}

static void rejectsTruncatedTelemetry() {
  uint8_t telemetry[]={0x40,0,0,0,200,0};
  int16_t parsed=0;
  CHECK(!parseRealIndoorBikeData(telemetry,5,parsed));
  CHECK(!parseRealIndoorBikeData(telemetry,1,parsed));
}

static void answersControlAndResetLocally() {
  clearDeliveries();
  writeCp({FTMS_CP_OP_REQUEST_CONTROL});
  CHECK(lastCpIs(FTMS_CP_OP_REQUEST_CONTROL,FTMS_CP_RES_SUCCESS,APP));
  writeCp({FTMS_CP_OP_RESET});
  CHECK(lastCpIs(FTMS_CP_OP_RESET,FTMS_CP_RES_SUCCESS,APP));
  CHECK(commandQueue.isEmpty());
}

static void startIsAnsweredLocallyAndForwardedToTrainer() {
  writeCp({FTMS_CP_OP_START_RESUME});
  CHECK(lastCpIs(FTMS_CP_OP_START_RESUME,FTMS_CP_RES_SUCCESS,APP));
  const auto indications=cpIndications();
  processQueuedCommands();
  CHECK(lastTrainerWrite()==Bytes{FTMS_CP_OP_START_RESUME});
  reply(FTMS_CP_OP_START_RESUME);
  CHECK(cpIndications()==indications && cpTxState==CpTxState::IDLE); // App already has its answer.
}

static void rejectsEmptyAndOversizedWrites() {
  const auto indications=cpIndications();
  writeCp({});
  CHECK(cpIndications()==indications);
  writeCp(Bytes(21,FTMS_CP_OP_SET_TARGET_POWER),GARMIN);
  CHECK(lastCpIs(FTMS_CP_OP_SET_TARGET_POWER,FTMS_CP_RES_INVALID_PARAMETER,GARMIN));
}

static void forwardsErgTargetAndRoutesResponseToSender() {
  writeCp({FTMS_CP_OP_SET_TARGET_POWER,250,0},GARMIN);
  processQueuedCommands();
  CHECK(cpTxState==CpTxState::WAITING_RESPONSE);
  CHECK((lastTrainerWrite()==Bytes{FTMS_CP_OP_SET_TARGET_POWER,250,0}));
  const auto indications=cpIndications();
  reply(FTMS_CP_OP_START_RESUME); // Wrong opcode: does not complete the transaction.
  CHECK(cpTxState==CpTxState::WAITING_RESPONSE && cpIndications()==indications);
  const auto statuses=virtualStatusChr->notifications.size();
  reply(FTMS_CP_OP_SET_TARGET_POWER);
  CHECK(lastCpIs(FTMS_CP_OP_SET_TARGET_POWER,FTMS_CP_RES_SUCCESS,GARMIN));
  CHECK(cpTxState==CpTxState::IDLE && activeTargetPower==250);
  CHECK(virtualStatusChr->notifications.size()==statuses+2);
}

static void forwardsSimulationParameters() {
  writeCp({FTMS_CP_OP_SET_INDOOR_BIKE_SIM,0,0,1,0,4,32});
  processQueuedCommands(); reply(FTMS_CP_OP_SET_INDOOR_BIKE_SIM);
  CHECK(lastCpIs(FTMS_CP_OP_SET_INDOOR_BIKE_SIM,FTMS_CP_RES_SUCCESS,APP));
  CHECK(virtualStatusChr->notifications.back().bytes.size()==7);
}

static void handlesResponseArrivingBeforeWriteReturns() {
  realControlPointChr->afterWrite=[]{ reply(FTMS_CP_OP_SET_TARGET_POWER); };
  writeCp({FTMS_CP_OP_SET_TARGET_POWER,220,0});
  processQueuedCommands();
  CHECK(lastCpIs(FTMS_CP_OP_SET_TARGET_POWER,FTMS_CP_RES_SUCCESS,APP));
  CHECK(cpTxState==CpTxState::IDLE && activeTargetPower==220);
}

static void forwardsTrainerFailureWithoutStatusChange() {
  writeCp({FTMS_CP_OP_SET_TARGET_POWER,100,0});
  processQueuedCommands();
  const auto statuses=virtualStatusChr->notifications.size();
  reply(FTMS_CP_OP_SET_TARGET_POWER,FTMS_CP_RES_OPERATION_FAILED);
  CHECK(lastCpIs(FTMS_CP_OP_SET_TARGET_POWER,FTMS_CP_RES_OPERATION_FAILED,APP));
  CHECK(virtualStatusChr->notifications.size()==statuses && activeTargetPower==220);
}

static void failsWhenTrainerWriteFails() {
  realControlPointChr->writeOk=false;
  writeCp({FTMS_CP_OP_STOP_PAUSE,1}); processQueuedCommands();
  CHECK(lastCpIs(FTMS_CP_OP_STOP_PAUSE,FTMS_CP_RES_OPERATION_FAILED,APP));
  CHECK(cpTxState==CpTxState::IDLE);
  realControlPointChr->writeOk=true;
}

static void timesOutAndDropsLateResponse() {
  writeCp({FTMS_CP_OP_STOP_PAUSE,1}); processQueuedCommands();
  testNow+=CP_TIMEOUT_MS+1; processQueuedCommands();
  CHECK(lastCpIs(FTMS_CP_OP_STOP_PAUSE,FTMS_CP_RES_OPERATION_FAILED,APP));
  const auto indications=cpIndications();
  reply(FTMS_CP_OP_STOP_PAUSE);
  CHECK(cpIndications()==indications && cpTxState==CpTxState::IDLE);
}

static void departedAppGetsNoResponseAndOthersGetNoBroadcast() {
  const auto indications=cpIndications();
  writeCp({FTMS_CP_OP_SET_TARGET_POWER,150,0}); processQueuedCommands();
  disconnectApp(APP);
  reply(FTMS_CP_OP_SET_TARGET_POWER);
  CHECK(cpIndications()==indications);

  writeCp({FTMS_CP_OP_SET_TARGET_POWER,160,0}); processQueuedCommands();
  disconnectApp(APP);
  testNow+=CP_TIMEOUT_MS+1; processQueuedCommands();
  CHECK(cpIndications()==indications && cpTxState==CpTxState::TIMED_OUT);
  reply(FTMS_CP_OP_SET_TARGET_POWER);
  CHECK(cpTxState==CpTxState::IDLE);
}

static void survivesTwentyTrainerReconnects() {
  auto* client=realClient;
  for(int cycle=0;cycle<20;++cycle) {
    writeCp({FTMS_CP_OP_SET_TARGET_POWER,180,0});
    processQueuedCommands();
    CHECK(cpTxState==CpTxState::WAITING_RESPONSE);
    writeCp({FTMS_CP_OP_STOP_PAUSE,1},GARMIN);
    CHECK(!commandQueue.isEmpty());
    clearDeliveries();
    disconnectTrainer();
    CHECK(!realConnected && !proxyReady && !realIndoorBikeDataChr && !realControlPointChr);
    CHECK(commandQueue.isEmpty() && cpTxState==CpTxState::IDLE);

    // In-flight and queued commands fail immediately, each to its sender.
    CHECK(cpIndications()==2);
    CHECK((virtualControlPointChr->indications[0].bytes==Bytes{FTMS_CP_OP_RESPONSE_CODE,FTMS_CP_OP_SET_TARGET_POWER,FTMS_CP_RES_OPERATION_FAILED}));
    CHECK(virtualControlPointChr->indications[0].handle==APP);
    CHECK(lastCpIs(FTMS_CP_OP_STOP_PAUSE,FTMS_CP_RES_OPERATION_FAILED,GARMIN));

    // Riders see zero power instead of a frozen value.
    CHECK(powerFilter.getLastOutput()==0 && virtualSpeedModel.getSpeedMps()==0);
    CHECK(readS16(virtualCpsMeasurementChr->notifications.back().bytes.data(),2)==0);
    CHECK(readS16(virtualIndoorBikeDataChr->notifications.back().bytes.data(),7)==0);
    CHECK(!NimBLEDevice::advertising.active && doScan);

    writeCp({FTMS_CP_OP_SET_TARGET_POWER,200,0});
    CHECK(lastCpIs(FTMS_CP_OP_SET_TARGET_POWER,FTMS_CP_RES_OPERATION_FAILED,APP));

    const auto starts=scanner()->starts;
    testNow=lastScanAt-1; loop(); CHECK(scanner()->starts==starts);
    testNow=lastScanAt;   loop(); CHECK(scanner()->starts==starts+1);
    advertise(trainer);
    loop();
    CHECK(realConnected && proxyReady && realClient==client);
    CHECK(NimBLEDevice::initCalls==1);
  }
}

static void failedConnectOrSubscribeReturnsToScanning() {
  disconnectTrainer(); realClient->connectOk=false; doConnectReal=true;
  loop(); CHECK(!realClient && !realConnected && doScan);
  realClient=NimBLEDevice::createClient();
  realClient->service.data.subscribeOk=false; doConnectReal=true;
  loop(); CHECK(!realClient && !proxyReady && doScan);
}

static void queueCoalescesPerClient() {
  CommandQueue queue;
  uint8_t command[]={FTMS_CP_OP_SET_TARGET_POWER,1,0};
  CHECK(queue.push(command,3,command[0],1,APP));
  command[1]=2; CHECK(queue.push(command,3,command[0],2,APP));
  command[1]=3; CHECK(queue.push(command,3,command[0],3,GARMIN));
  QueuedCommand out{};
  CHECK(queue.count()==2 && queue.pop(out));
  CHECK(out.targetValue==2 && out.connHandle==APP);
  CHECK(queue.pop(out) && out.targetValue==3 && out.connHandle==GARMIN);
}

static void queueEnforcesBoundsAndCapacity() {
  CommandQueue queue;
  uint8_t command[]={FTMS_CP_OP_STOP_PAUSE,1};
  CHECK(!queue.push(nullptr,2,command[0]));
  CHECK(!queue.push(command,21,command[0]));
  for(size_t i=0;i<CommandQueue::CAPACITY;++i) CHECK(queue.push(command,2,command[0]));
  CHECK(queue.isFull() && !queue.push(command,2,command[0]));
  QueuedCommand out{};
  queue.clear(); CHECK(queue.isEmpty() && !queue.pop(out));
}

int main() {
  const std::vector<std::pair<const char*,void(*)()>> tests={
#define T(fn) {#fn,fn}
    T(moduleSelfChecks),
    T(bootStartsAnEightSecondScan),
    T(ignoresOtherDevicesAndRetriesAfterThreeSeconds),
    T(connectsToMatchingTrainer),
    T(requestsTrainerControlOnConnect),
    T(pausesAdvertisingAtTwoClients),
    T(sendsIdenticalTelemetryToEverySubscriber),
    T(rejectsTruncatedTelemetry),
    T(answersControlAndResetLocally),
    T(startIsAnsweredLocallyAndForwardedToTrainer),
    T(rejectsEmptyAndOversizedWrites),
    T(forwardsErgTargetAndRoutesResponseToSender),
    T(forwardsSimulationParameters),
    T(handlesResponseArrivingBeforeWriteReturns),
    T(forwardsTrainerFailureWithoutStatusChange),
    T(failsWhenTrainerWriteFails),
    T(timesOutAndDropsLateResponse),
    T(departedAppGetsNoResponseAndOthersGetNoBroadcast),
    T(survivesTwentyTrainerReconnects),
    T(failedConnectOrSubscribeReturnsToScanning),
    T(queueCoalescesPerClient),
    T(queueEnforcesBoundsAndCapacity),
#undef T
  };
  for(const auto& [name,fn]:tests) {
    currentTest=name;
    fn();
    std::printf("ok   %s\n",name);
  }
  std::printf("PASS: %zu tests\n",tests.size());
}
