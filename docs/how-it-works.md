# How it works

## Architecture

```text
Real trainer (FTMS 0x1826)
        │
   [BLE client]  one connection, owned by the ESP32
        ▼
  TrainerBridge (ESP32)
   ├── Power filter      Q8 EMA smoothing, calibration scale, stale-data watchdog
   ├── Speed model       flat-road physics, flywheel coasting
   └── Command queue     FreeRTOS-safe, one trainer transaction at a time
        │
   [BLE server]  several clients at once
        ├── FTMS (0x1826) ──► Zwift / MyWhoosh / TrainerDay (ERG, simulation)
        │                └──► Garmin Edge (Smart Trainer)
        ├── CSC  (0x1816) ──► Garmin speed sensor
        └── CPS  (0x1818) ──► Garmin power meter
```

The ESP32 scans for the trainer (by MAC, then by name, otherwise the first FTMS device),
subscribes to its Indoor Bike Data and Control Point, and takes control of it. It then
advertises its own three services. Advertising pauses at two connected clients and resumes
when one leaves.

## Bluetooth services

1. **Fitness Machine Service (FTMS `0x1826`)**
   - `0x2AD2` Indoor Bike Data: instantaneous power, speed (0.01 km/h) and total distance
     (1 m, `uint24`).
   - `0x2ACC` Fitness Machine Feature: power, resistance and total distance.
   - `0x2AD9` Control Point: target power, simulation parameters, start/stop.
   - `0x2ADA` Fitness Machine Status: reports target and simulation changes once the trainer
     has accepted them.
2. **Cycling Speed and Cadence (CSC `0x1816`)**
   - `0x2A5B` CSC Measurement: cumulative wheel revolutions and last wheel event time
     (1/1024 s), what Garmin expects from a wheel speed sensor.
   - `0x2A5C` CSC Feature and `0x2A5D` Sensor Location.
3. **Cycling Power Service (CPS `0x1818`)**
   - `0x2A63` Cycling Power Measurement: instantaneous power in watts.

## Control Point

App commands go through a queue and reach the trainer from the main loop, never from a
Bluetooth callback (a blocking write there can deadlock the NimBLE host task).

- **Request Control** and **Reset** are answered by the bridge; it requested control of the
  trainer itself when it connected. **Start/Resume** is answered right away and also
  forwarded to the trainer.
- **Target power, simulation, resistance, stop**: queued and sent one at a time. The app
  gets the trainer's own response, only on its connection. Repeated updates from the same
  app are merged while queued, so a slider drag sends only the latest value.
- A command fails with *Operation Failed* when the trainer doesn't answer within 1.2 s, when
  the trainer disconnects, or when the queue is full. A late answer after a timeout is
  dropped.
- Responses with no recipient (serial console commands, or an app that left) are never
  broadcast to other clients.

## Virtual speed and distance

On a flat road with no wind, power at the wheel balances rolling resistance and
aerodynamic drag:

$$P_{\text{wheel}} = \left(C_{\text{rr}} \cdot m \cdot g\right) v + \left(\frac{1}{2} \cdot \rho \cdot C_d A\right) v^3$$

| | | |
|---|---|---|
| Total mass $m$ | 85 kg | rider 75 kg + bike 10 kg |
| Rolling resistance $C_{\text{rr}}$ | 0.004 | performance road tyre on smooth asphalt |
| Drag area $C_d A$ | 0.32 m² | endurance position on the hoods |
| Air density $\rho$ | 1.225 kg/m³ | sea level |
| Drivetrain efficiency | 98% | |
| Coasting | 1.5 m/s² | smooth slow-down when you stop pedalling, so Garmin doesn't auto-pause |

| Power | Speed |
|---|---|
| 100 W | ≈ 26.0 km/h |
| 150 W | ≈ 30.5 km/h |
| 200 W | ≈ 34.0 km/h |
| 250 W | ≈ 36.9 km/h |
| 300 W | ≈ 39.4 km/h |

All constants are in [`Config.h`](../TrainerBridge/Config.h).

## Status LED

| LED | Meaning |
|---|---|
| Blinking (1 Hz) | Searching for or connecting to the trainer |
| Solid | Trainer connected, ready for apps and Garmin |
| Off | No power, or the LED is disabled in `Config.h` |

Set `STATUS_LED_BLINK_WHILE_SCANNING = false` to keep it off until connected.

## Source layout

```text
TrainerBridge/
├── TrainerBridge.ino          Entry point: GATT services, scanning, connection and Control Point state
├── Config.h                   User settings and physics constants
├── FtmsConstants.h            UUIDs, opcodes and flags
├── PowerFilter.h/.cpp         Power smoothing, scaling and stale watchdog
├── CommandQueue.h/.cpp        Critical-section-protected coalescing command queue
├── TrainerMatcher.h/.cpp      Trainer MAC/name matching (with boot self-check)
├── VirtualSpeedModel.h/.cpp   Speed/distance model and CSC wheel event timing
└── StatusLed.h/.cpp           Non-blocking status LED
tests/
├── run.sh                     Builds and runs the host tests (SANITIZE=1 for ASan/UBSan)
├── firmware_test.cpp          Named scenario tests against the real sketch
└── stubs/                     Arduino, NimBLE and FreeRTOS test doubles
```
