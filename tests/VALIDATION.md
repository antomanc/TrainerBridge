# Stable patch upgrade validation

## Dependency and API review

- [ESP32 Arduino 3.3.11](https://github.com/espressif/arduino-esp32/releases/tag/3.3.11)
  and [3.3.12](https://github.com/espressif/arduino-esp32/releases/tag/3.3.12):
  reviewed both intervening release notes, including ESP-IDF changes. The sketch
  uses Arduino timing, Serial, GPIO, FreeRTOS critical sections and the classic
  ESP32 brownout register. It uses NimBLE-Arduino, not the core's separate BLE
  library, so the core BLE fixes are not proof of a TrainerBridge behavior change.
- [NimBLE-Arduino 2.5.1](https://github.com/h2zero/NimBLE-Arduino/releases/tag/2.5.1)
  and its [2.5.0 comparison](https://github.com/h2zero/NimBLE-Arduino/compare/2.5.0...2.5.1):
  reviewed scan timer lifecycle changes and the public signatures used by this
  sketch. Scan callbacks/start, client callbacks/connect/subscriptions/writes,
  server callbacks, broadcast `notify(data, length)` and connection-specific
  `indicate(data, length, handle)` remain compatible. No firmware migration needed.
- The scan-response timer fix initializes the callout lazily and tears it down
  before the host event queue. TrainerBridge calls `NimBLEDevice::init` once and
  never calls `deinit`; ordinary reconnect reuses the host/client. This does not
  reproduce or establish exposure to the upstream reinitialization crash.

## Reproducible software checks

```bash
bash tests/run.sh
arduino-cli compile --warnings all --fqbn esp32:esp32:esp32 TrainerBridge
```

`tests/run.sh` includes the actual sketch and links the actual helper modules.
The stubs only provide hardware/stack boundaries; they are not a NimBLE emulator.
The Arduino compile is the compatibility check against the real dependencies.

Host assertions cover:

- Existing matcher, LED and virtual-speed self-checks.
- Nonmatching/matching advertisements, address copying, scan completion, 8-second
  scans and 3-second retry boundaries; no extra retry on a matched scan stopping.
- Twenty disconnect/rediscovery/reconnect cycles, without another host init,
  including queue/filter cleanup, remote-pointer clearing, zero-power telemetry,
  failed reconnect and failed telemetry subscription.
- Two subscriber handles receive identical CPS/CSC/FTMS payloads; advertising
  pauses at two clients and resumes below two after its existing throttle.
- FTMS Request Control, Start and Reset responses; queued ERG and simulation
  writes; response routing to the originating handle; successful/failed trainer
  replies; failed writes; timeout and late-response suppression; a departing app's
  pending response; empty/oversized writes; truncated telemetry; queue capacity,
  payload bounds and existing coalescing behavior.

## Results (2026-10-07)

- Host regression harness: PASS, including twenty simulated reconnect cycles.
- Arduino CLI 1.5.2-rc.1, ESP32 Arduino 3.3.12 and NimBLE-Arduino 2.5.1:
  the exact compile command above passed with no warnings. Flash: 627,988 /
  1,310,720 bytes (47%); static RAM: 37,172 / 327,680 bytes (11%).
- AddressSanitizer and UndefinedBehaviorSanitizer: PASS for the host harness with
  `ASAN_OPTIONS=detect_leaks=0`. LeakSanitizer cannot inspect processes in this
  execution environment; neither this run nor the mocks validate physical leaks.
- Hardware checks below: not executed.

## Existing behavior outside this compatibility change

These are not claims of complete FTMS correctness: the queue coalesces by opcode
across clients, retaining the latest handle; pending trainer commands are cleared
on trainer disconnect without an immediate failed indication. The initial Request
Control attempt occurs before `realConnected` is set, and the write helper declines
it. The response state machine does not compare a received opcode with the pending
opcode. These pre-existing semantics are unchanged; concurrent controllers,
out-of-order/late replies and trainer control acquisition require separate work
if physical tests expose failures. This patch does not change protocol policy.

## Physical checks still required

No physical ESP32, trainer, Garmin or app is available in the software harness.

1. Flash the build on the supported standard ESP32 and D100. Confirm serial module
   self-checks, discovery by name/MAC, active scan responses and LED states.
2. Power-cycle the trainer at least 20 times with Garmin and an app still paired.
   Check zero power on loss, scan retry after three seconds, rediscovery, fresh
   telemetry, recovered advertising, and no crash/watchdog/heap growth.
3. Connect Garmin and an app concurrently. Verify CPS, CSC and FTMS subscriptions,
   power/speed/distance delivery, disconnect/rejoin and continued delivery to the
   remaining subscriber. Exercise subscription changes and MTU negotiation.
4. Exercise Request Control, ERG target power, simulation grade, Start/Stop and
   Reset against the real trainer. Verify resistance changes and response/status
   delivery only as intended; disconnect each side during a pending transaction,
   provoke timeout/late replies, and repeat reconnect plus control acquisition.
5. The upstream init/deinit timer fix is not exercised by this firmware's normal
   reconnect. A separate upstream reproduction is needed to validate that fix on
   hardware; the host harness cannot certify the NimBLE timer or event queue.
