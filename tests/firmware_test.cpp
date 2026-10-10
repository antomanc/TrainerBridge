#include <cassert>
#include <iostream>
#include "../TrainerBridge/TrainerBridge.ino"

// Call private overrides through the public NimBLE callback interface.
static void disconnectTrainer() {
  static_cast<NimBLEClientCallbacks*>(&realClientCallbacks)->onDisconnect(realClient,19);
}
static void writeCp(std::initializer_list<uint8_t> bytes,uint16_t handle=11) {
  std::vector<uint8_t> data(bytes);
  virtualControlPointChr->setValue(data.data(),data.size());
  NimBLEConnInfo info(handle);
  static_cast<NimBLECharacteristicCallbacks*>(&virtualControlPointCallbacks)->onWrite(virtualControlPointChr,info);
}
static void clearDeliveries() {
  for(auto* c:{virtualCpsMeasurementChr,virtualCscMeasurementChr,virtualIndoorBikeDataChr,virtualStatusChr,virtualControlPointChr}) {
    c->notifications.clear(); c->indications.clear();
  }
}
static void reply(uint8_t opcode,uint8_t result=FTMS_CP_RES_SUCCESS) {
  uint8_t bytes[]={FTMS_CP_OP_RESPONSE_CODE,opcode,result};
  realControlPointCallback(nullptr,bytes,sizeof(bytes),false);
}
static void assertCp(uint8_t opcode,uint8_t result,uint16_t handle) {
  const auto& d=virtualControlPointChr->indications.back();
  assert(d.handle==handle);
  assert((d.bytes==std::vector<uint8_t>{FTMS_CP_OP_RESPONSE_CODE,opcode,result}));
}
static void completeScan() {
  static_cast<NimBLEScanCallbacks*>(&realScanCallbacks)->onScanEnd(NimBLEScanResults{},0);
}
int main() {
  assert(TrainerMatcher::selfCheck());
  assert(StatusLed::selfCheck());
  assert(VirtualSpeedModel::selfCheck());
  setup();
  assert(NimBLEDevice::initCalls==1);
  loop();
  auto* scan=NimBLEDevice::getScan();
  assert(scanning && scan->starts==1 && scan->duration==8000 && scan->clears==1);
  NimBLEAdvertisedDevice other; other.name="Other trainer";
  static_cast<NimBLEScanCallbacks*>(&realScanCallbacks)->onResult(&other);
  assert(!doConnectReal && scanning);
  completeScan();
  assert(doScan && !scanning);
  const auto retry=lastScanAt;
  testNow=retry-1; loop(); assert(scan->starts==1);
  testNow=retry; loop(); assert(scan->starts==2);
  NimBLEAdvertisedDevice trainer;
  static_cast<NimBLEScanCallbacks*>(&realScanCallbacks)->onResult(&trainer);
  assert(doConnectReal && !doScan && !scanning);
  assert(realTrainerAddress.toString()==trainer.address.toString());
  completeScan(); assert(!doScan); // Stop completion must not schedule another scan after match.
  loop();
  assert(realConnected && proxyReady && realClient);
  assert(realIndoorBikeDataChr->callback==realIndoorBikeDataCallback);
  assert(realControlPointChr->callback==realControlPointCallback);
  // The bridge takes control of the trainer once connected; the reply is not forwarded.
  assert((realControlPointChr->writes==std::vector<std::vector<uint8_t>>{{FTMS_CP_OP_REQUEST_CONTROL}}));
  reply(FTMS_CP_OP_REQUEST_CONTROL);
  assert(virtualControlPointChr->indications.empty() && cpTxState==CpTxState::IDLE);
  proxyServer->connectedCount=2; loop(); assert(!NimBLEDevice::advertising.active);
  proxyServer->connectedCount=1; testNow+=500; loop(); assert(NimBLEDevice::advertising.active);

  clearDeliveries();
  uint8_t telemetry[]={0x40,0,0,0,200,0}; // Speed and instantaneous power.
  realIndoorBikeDataCallback(nullptr,telemetry,sizeof(telemetry),true);
  for(auto* c:{virtualCpsMeasurementChr,virtualCscMeasurementChr,virtualIndoorBikeDataChr}) {
    assert(c->notifications.size()==2);
    assert(c->notifications[0].handle==11 && c->notifications[1].handle==22);
    assert(c->notifications[0].bytes==c->notifications[1].bytes);
  }
  assert(readS16(virtualCpsMeasurementChr->notifications[0].bytes.data(),2)==200);
  assert(readS16(virtualIndoorBikeDataChr->notifications[0].bytes.data(),7)==200);
  int16_t parsed=0;
  assert(!parseRealIndoorBikeData(telemetry,5,parsed));
  assert(!parseRealIndoorBikeData(telemetry,1,parsed));

  clearDeliveries();
  writeCp({FTMS_CP_OP_REQUEST_CONTROL});
  assertCp(FTMS_CP_OP_REQUEST_CONTROL,FTMS_CP_RES_SUCCESS,11);
  assert(commandQueue.isEmpty());
  writeCp({FTMS_CP_OP_START_RESUME});
  assertCp(FTMS_CP_OP_START_RESUME,FTMS_CP_RES_SUCCESS,11);
  writeCp({FTMS_CP_OP_RESET});
  assertCp(FTMS_CP_OP_RESET,FTMS_CP_RES_SUCCESS,11);
  writeCp({});
  std::vector<uint8_t> oversized(21,FTMS_CP_OP_SET_TARGET_POWER);
  virtualControlPointChr->setValue(oversized.data(),oversized.size());
  NimBLEConnInfo info(22);
  static_cast<NimBLECharacteristicCallbacks*>(&virtualControlPointCallbacks)->onWrite(virtualControlPointChr,info);
  assertCp(FTMS_CP_OP_SET_TARGET_POWER,FTMS_CP_RES_INVALID_PARAMETER,22);
  writeCp({FTMS_CP_OP_SET_TARGET_POWER,250,0},22);
  processQueuedCommands();
  assert(cpTxState==CpTxState::WAITING_RESPONSE);
  assert(realControlPointChr->writes.back()==std::vector<uint8_t>({FTMS_CP_OP_SET_TARGET_POWER,250,0}));
  auto before=virtualControlPointChr->indications.size();
  reply(FTMS_CP_OP_START_RESUME); // Mismatched opcode must not complete the transaction.
  assert(cpTxState==CpTxState::WAITING_RESPONSE && virtualControlPointChr->indications.size()==before);
  auto statuses=virtualStatusChr->notifications.size();
  reply(FTMS_CP_OP_SET_TARGET_POWER);
  assertCp(FTMS_CP_OP_SET_TARGET_POWER,FTMS_CP_RES_SUCCESS,22);
  assert(cpTxState==CpTxState::IDLE && activeTargetPower==250);
  assert(virtualStatusChr->notifications.size()==statuses+2);
  writeCp({FTMS_CP_OP_SET_INDOOR_BIKE_SIM,0,0,1,0,4,32});
  processQueuedCommands(); reply(FTMS_CP_OP_SET_INDOOR_BIKE_SIM);
  assertCp(FTMS_CP_OP_SET_INDOOR_BIKE_SIM,FTMS_CP_RES_SUCCESS,11);
  assert(virtualStatusChr->notifications.back().bytes.size()==7);
  writeCp({FTMS_CP_OP_SET_TARGET_POWER,100,0});
  processQueuedCommands(); statuses=virtualStatusChr->notifications.size();
  reply(FTMS_CP_OP_SET_TARGET_POWER,FTMS_CP_RES_OPERATION_FAILED);
  assertCp(FTMS_CP_OP_SET_TARGET_POWER,FTMS_CP_RES_OPERATION_FAILED,11);
  assert(virtualStatusChr->notifications.size()==statuses && activeTargetPower==250);
  realControlPointChr->writeOk=false;
  writeCp({FTMS_CP_OP_STOP_PAUSE,1}); processQueuedCommands();
  assertCp(FTMS_CP_OP_STOP_PAUSE,FTMS_CP_RES_OPERATION_FAILED,11);
  realControlPointChr->writeOk=true;
  writeCp({FTMS_CP_OP_STOP_PAUSE,1}); processQueuedCommands();
  testNow+=CP_TIMEOUT_MS+1; processQueuedCommands();
  assertCp(FTMS_CP_OP_STOP_PAUSE,FTMS_CP_RES_OPERATION_FAILED,11);
  const auto indications=virtualControlPointChr->indications.size();
  reply(FTMS_CP_OP_STOP_PAUSE);
  assert(virtualControlPointChr->indications.size()==indications && cpTxState==CpTxState::IDLE);

  // A departing app must not receive the pending trainer response.
  writeCp({FTMS_CP_OP_SET_TARGET_POWER,150,0}); processQueuedCommands();
  NimBLEConnInfo departing(11);
  static_cast<NimBLEServerCallbacks*>(&proxyServerCallbacks)->onDisconnect(proxyServer,departing,19);
  reply(FTMS_CP_OP_SET_TARGET_POWER);
  assert(virtualControlPointChr->indications.size()==indications);
  // A departing app's timeout is not broadcast to the remaining clients.
  writeCp({FTMS_CP_OP_SET_TARGET_POWER,160,0}); processQueuedCommands();
  static_cast<NimBLEServerCallbacks*>(&proxyServerCallbacks)->onDisconnect(proxyServer,departing,19);
  testNow+=CP_TIMEOUT_MS+1; processQueuedCommands();
  assert(virtualControlPointChr->indications.size()==indications && cpTxState==CpTxState::TIMED_OUT);
  reply(FTMS_CP_OP_SET_TARGET_POWER);

  // Repeat disconnect -> delayed scan -> rediscovery -> reconnect, keeping the same host/client.
  auto* client=realClient;
  for(int cycle=0;cycle<20;++cycle) {
    writeCp({FTMS_CP_OP_SET_TARGET_POWER,180,0});
    processQueuedCommands();
    assert(cpTxState==CpTxState::WAITING_RESPONSE);
    writeCp({FTMS_CP_OP_STOP_PAUSE,1},22);
    assert(!commandQueue.isEmpty());
    clearDeliveries();
    disconnectTrainer();
    // In-flight and queued commands fail immediately, each to its originating client.
    assert(virtualControlPointChr->indications.size()==2);
    assert((virtualControlPointChr->indications[0].bytes==std::vector<uint8_t>{FTMS_CP_OP_RESPONSE_CODE,FTMS_CP_OP_SET_TARGET_POWER,FTMS_CP_RES_OPERATION_FAILED}));
    assert(virtualControlPointChr->indications[0].handle==11);
    assertCp(FTMS_CP_OP_STOP_PAUSE,FTMS_CP_RES_OPERATION_FAILED,22);
    assert(!realConnected && !proxyReady && !realIndoorBikeDataChr && !realControlPointChr);
    assert(commandQueue.isEmpty() && cpTxState==CpTxState::IDLE);
    assert(powerFilter.getLastOutput()==0 && virtualSpeedModel.getSpeedMps()==0);
    assert(!NimBLEDevice::advertising.active && doScan);
    assert(readS16(virtualCpsMeasurementChr->notifications.back().bytes.data(),2)==0);
    assert(readS16(virtualIndoorBikeDataChr->notifications.back().bytes.data(),7)==0);
    writeCp({FTMS_CP_OP_SET_TARGET_POWER,200,0});
    assertCp(FTMS_CP_OP_SET_TARGET_POWER,FTMS_CP_RES_OPERATION_FAILED,11);
    const auto starts=scan->starts;
    testNow=lastScanAt-1; loop(); assert(scan->starts==starts);
    testNow=lastScanAt; loop(); assert(scan->starts==starts+1);
    static_cast<NimBLEScanCallbacks*>(&realScanCallbacks)->onResult(&trainer);
    loop(); assert(realConnected && proxyReady && realClient==client);
    assert(NimBLEDevice::initCalls==1);
  }
  CommandQueue queue;
  uint8_t command[]={FTMS_CP_OP_SET_TARGET_POWER,1,0};
  assert(queue.push(command,3,command[0],1,11));
  command[1]=2; assert(queue.push(command,3,command[0],2,11));
  command[1]=3; assert(queue.push(command,3,command[0],3,22));
  QueuedCommand out{}; assert(queue.count()==2 && queue.pop(out));
  assert(out.targetValue==2 && out.connHandle==11); // Coalesces only within one client.
  assert(queue.pop(out) && out.targetValue==3 && out.connHandle==22);
  assert(!queue.push(nullptr,3,command[0]));
  assert(!queue.push(command,21,command[0]));
  // Failed reconnect and failed telemetry subscription return to the scan retry path.
  disconnectTrainer(); realClient->connectOk=false; doConnectReal=true;
  loop(); assert(!realClient && !realConnected && doScan);
  realClient=NimBLEDevice::createClient();
  realClient->service.data.subscribeOk=false; doConnectReal=true;
  loop(); assert(!realClient && !proxyReady && doScan);
  assert(queue.push(command,3,FTMS_CP_OP_STOP_PAUSE));
  for(size_t i=1;i<CommandQueue::CAPACITY;++i) assert(queue.push(command,3,FTMS_CP_OP_STOP_PAUSE));
  assert(queue.isFull() && !queue.push(command,3,FTMS_CP_OP_STOP_PAUSE));
  queue.clear(); assert(queue.isEmpty() && !queue.pop(out));
  std::cout << "PASS: self-checks, scanning, 20 reconnect cycles, multi-client routing and FTMS transactions\n";
}
