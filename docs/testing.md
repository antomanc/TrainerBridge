# Testing

## Host tests

```bash
make test       # g++ with C++17, nothing else
make sanitize   # same, with AddressSanitizer + UndefinedBehaviorSanitizer (what CI runs)
make check      # sanitize, then the real firmware build with all warnings
```

`tests/run.sh` compiles the real `TrainerBridge.ino` and helper modules against the doubles in
`tests/stubs/`. The doubles only stand in for the hardware and the Bluetooth stack: they record
notifications, indications and trainer writes per connection, and let a test trigger NimBLE
callbacks. They are not a NimBLE emulator. CI compiles the firmware against the real pinned
ESP32 core and NimBLE-Arduino, which is the API compatibility check.

`tests/firmware_test.cpp` runs named tests in order, as one session. Covered:

- Module self-checks (trainer matcher, LED, speed model).
- Scanning: ignoring other devices, 8 s scans, the 3 s retry, no extra scan after a match.
- Connecting: subscriptions, Request Control sent to the trainer, its reply not forwarded.
- Two clients: identical CPS, CSC and FTMS telemetry; advertising pauses at two clients.
- Control Point: local Request Control, Reset and Start answers, Start forwarded to the
  trainer; ERG and simulation forwarded, with the response sent only to the sender;
  mismatched, failed, timed-out and late responses; a response that arrives before the
  trainer write returns; empty and oversized writes; failures not broadcast to other clients.
- 20 trainer disconnect/reconnect cycles without re-initialising NimBLE: queued and in-flight
  commands fail to their senders, 0 W is sent, and the client is reused.
- Failed connection and failed subscription return to scanning.
- Command queue: per-client coalescing, payload bounds and capacity.

Not covered: the radio, NimBLE and FreeRTOS internals, real concurrency between the
Bluetooth host task and `loop()`, timing on the device and memory over a long ride.

## Hardware checklist

Run this after changes to Bluetooth, Control Point or connection handling, and after a core or
NimBLE upgrade. It needs an ESP32, a trainer, a Garmin and an app.

1. **Boot**: flash, open `make monitor`. The module self-checks pass, the trainer is found by
   name or MAC, and the LED blinks then turns solid.
2. **Reconnects**: with the Garmin and an app paired, power-cycle the trainer at least 20
   times. Each time: 0 W on loss, rediscovery, fresh telemetry, advertising back, and no
   crash, watchdog reset or heap growth (`status`).
3. **Two clients**: Garmin and app together. Power, speed and distance reach both; one
   disconnecting and rejoining doesn't interrupt the other.
4. **Control**: ERG target, simulation grade, Start/Stop and Reset against the real trainer.
   Resistance changes, and only the sending app gets the response. Disconnect each side while
   a command is pending, and repeat after a trainer reconnect.

## History

- **2026-10-07**: ESP32 Arduino 3.3.10 → 3.3.12 and NimBLE-Arduino 2.5.0 → 2.5.1. No API
  changes affect the firmware. NimBLE 2.5.1's scan-response timer fix concerns host
  re-initialisation, which TrainerBridge never does (it calls `NimBLEDevice::init` once).
  Hardware checklist not yet run on these versions.
