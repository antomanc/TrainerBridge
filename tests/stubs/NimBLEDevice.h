#pragma once
// Behavioral doubles only: the real Arduino build validates upstream API compatibility.
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <cstdint>
constexpr int ESP_PWR_LVL_P9=9;
namespace NIMBLE_PROPERTY { constexpr int READ=1, WRITE=2, NOTIFY=4, INDICATE=8; }
class NimBLEUUID {
  std::string value;
public:
  NimBLEUUID(uint16_t v): value(std::to_string(v)) {}
  NimBLEUUID(const char* v): value(v) {}
  std::string toString() const {return value;}
};
class NimBLEAddress {
  std::string value;
public:
  NimBLEAddress(const char* v=""): value(v) {}
  std::string toString() const {return value;}
};
class NimBLEConnInfo {
  uint16_t handle;
public:
  explicit NimBLEConnInfo(uint16_t h=1): handle(h) {}
  uint16_t getConnHandle() const { return handle; }
  NimBLEAddress getAddress() const { return NimBLEAddress("aa:bb"); }
};
class NimBLECharacteristic;
struct NimBLECharacteristicCallbacks {
  virtual ~NimBLECharacteristicCallbacks()=default;
  virtual void onWrite(NimBLECharacteristic*,NimBLEConnInfo&) {}
  virtual void onSubscribe(NimBLECharacteristic*,NimBLEConnInfo&,uint16_t) {}
  virtual void onStatus(NimBLECharacteristic*,NimBLEConnInfo&,int) {}
};
struct Delivery { std::vector<uint8_t> bytes; uint16_t handle; };
class NimBLECharacteristic {
  std::string value;
public:
  std::vector<Delivery> notifications, indications;
  NimBLECharacteristicCallbacks* callbacks=nullptr;
  std::vector<uint16_t> subscribers{11,22};
  void setCallbacks(NimBLECharacteristicCallbacks* c) {callbacks=c;}
  void setValue(const uint8_t* d,size_t n) {value.assign(reinterpret_cast<const char*>(d),n);}
  void setValue(const char* d) {value=d;}
  std::string getValue() const {return value;}
  NimBLEUUID getUUID() const {return NimBLEUUID(uint16_t{0});}
  bool notify(const uint8_t* d,size_t n) {
    for(auto h:subscribers) notifications.push_back({{d,d+n},h});
    return true;
  }
  bool indicate(const uint8_t* d,size_t n,uint16_t h=0xffff) {
    if(h==0xffff) {for(auto s:subscribers) indications.push_back({{d,d+n},s});}
    else indications.push_back({{d,d+n},h});
    return true;
  }
};
struct NimBLEService {
  std::vector<std::unique_ptr<NimBLECharacteristic>> characteristics;
  NimBLECharacteristic* createCharacteristic(const NimBLEUUID&,int) {
    characteristics.emplace_back(new NimBLECharacteristic); return characteristics.back().get();
  }
};
class NimBLEServer;
struct NimBLEServerCallbacks {
  virtual ~NimBLEServerCallbacks()=default;
  virtual void onConnect(NimBLEServer*,NimBLEConnInfo&) {}
  virtual void onDisconnect(NimBLEServer*,NimBLEConnInfo&,int) {}
  virtual void onMTUChange(uint16_t,NimBLEConnInfo&) {}
};
struct NimBLEServer {
  int connectedCount=0;
  std::vector<std::unique_ptr<NimBLEService>> services;
  void setCallbacks(NimBLEServerCallbacks*) {}
  void advertiseOnDisconnect(bool) {}
  void start() {}
  int getConnectedCount() {return connectedCount;}
  NimBLEService* createService(const NimBLEUUID&) {services.emplace_back(new NimBLEService);return services.back().get();}
};
struct NimBLEAdvertising {
  bool active=false;
  bool isAdvertising() const {return active;}
  bool start() {active=true;return true;}
  void stop() {active=false;}
  void reset() {active=false;}
  void enableScanResponse(bool) {}
  void setName(const char*) {}
  void setAppearance(int) {}
  void addServiceUUID(const NimBLEUUID&) {}
  void setServiceData(const NimBLEUUID&,const uint8_t*,size_t) {}
  void setMinInterval(int) {}
  void setMaxInterval(int) {}
};
struct NimBLEAdvertisedDevice {
  std::string name="Van Rysel D100";
  NimBLEAddress address{"aa:bb:cc:dd:ee:ff"};
  std::string getName() const {return name;}
  NimBLEAddress getAddress() const {return address;}
  int getRSSI() const {return -40;}
  bool haveServiceUUID() const {return true;}
  bool isAdvertisingService(const NimBLEUUID&) const {return true;}
  NimBLEUUID getServiceUUID() const {return NimBLEUUID(0x1826);}
};
struct NimBLEScanResults {int getCount() const {return 0;}};
struct NimBLEScanCallbacks {
  virtual ~NimBLEScanCallbacks()=default;
  virtual void onResult(const NimBLEAdvertisedDevice*) {}
  virtual void onScanEnd(const NimBLEScanResults&,int) {}
};
struct NimBLEScan {
  NimBLEScanCallbacks* callbacks=nullptr;
  unsigned starts=0,clears=0;
  uint32_t duration=0;
  bool active=false;
  void setScanCallbacks(NimBLEScanCallbacks* c,bool) {callbacks=c;}
  void setActiveScan(bool) {}
  void setInterval(int) {}
  void setWindow(int) {}
  void clearResults() {++clears;}
  bool start(uint32_t ms,bool,bool) {++starts;duration=ms;active=true;return true;}
  void stop() {active=false;}
};
class NimBLERemoteCharacteristic;
using RemoteCallback=void(*)(NimBLERemoteCharacteristic*,uint8_t*,size_t,bool);
struct NimBLERemoteCharacteristic {
  bool writable=true,notifiable=true,indicatable=true,subscribeOk=true,writeOk=true;
  RemoteCallback callback=nullptr;
  std::vector<std::vector<uint8_t>> writes;
  bool canWrite() const {return writable;}
  bool canNotify() const {return notifiable;}
  bool canIndicate() const {return indicatable;}
  bool subscribe(bool,RemoteCallback c) {callback=c;return subscribeOk;}
  bool writeValue(uint8_t* d,size_t n,bool) {writes.push_back({d,d+n});return writeOk;}
};
struct NimBLERemoteService {
  NimBLERemoteCharacteristic data,control;
  NimBLERemoteCharacteristic* getCharacteristic(const NimBLEUUID& id) {return id.toString()=="2AD2"?&data:&control;}
};
struct ble_gap_upd_params {};
class NimBLEClient;
struct NimBLEClientCallbacks {
  virtual ~NimBLEClientCallbacks()=default;
  virtual void onConnect(NimBLEClient*) {}
  virtual void onDisconnect(NimBLEClient*,int) {}
  virtual void onMTUChange(NimBLEClient*,uint16_t) {}
  virtual bool onConnParamsUpdateRequest(NimBLEClient*,const ble_gap_upd_params*) {return true;}
};
struct NimBLEClient {
  NimBLEClientCallbacks* callbacks=nullptr;
  NimBLERemoteService service;
  bool connectOk=true;
  void setClientCallbacks(NimBLEClientCallbacks* c,bool) {callbacks=c;}
  void setConnectionParams(int,int,int,int) {}
  void setConnectTimeout(int) {}
  bool connect(NimBLEAddress) {return connectOk;}
  NimBLERemoteService* getService(const NimBLEUUID&) {return &service;}
  void disconnect() {if(callbacks) callbacks->onDisconnect(this,0);}
};
struct NimBLEDevice {
  inline static NimBLEAdvertising advertising;
  inline static NimBLEScan scan;
  inline static NimBLEServer server;
  inline static unsigned initCalls=0;
  static void init(const char*) {++initCalls;}
  static void setPower(int) {}
  static void setMTU(int) {}
  static NimBLEAdvertising* getAdvertising() {return &advertising;}
  static NimBLEScan* getScan() {return &scan;}
  static NimBLEServer* createServer() {return &server;}
  static NimBLEClient* createClient() {return new NimBLEClient;}
  static void deleteClient(NimBLEClient* c) {delete c;}
};
