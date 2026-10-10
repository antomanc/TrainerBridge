# TrainerBridge

**Turn an ESP32 into a Bluetooth bridge that lets Zwift and your Garmin use the same smart trainer at the same time.**

Many trainers accept only one Bluetooth connection at a time, so you pick: ERG mode in the app, or power
and speed on your Garmin. TrainerBridge connects to the trainer once and re-publishes it as a
new device that **several apps and bike computers can pair with at once**. Apps keep full control
(ERG, simulation grade), and Garmin gets power, speed and distance without GPS or extra sensors.

```bash
make deps upload   # pinned ESP32 core + NimBLE, then build and flash
```

[![CI](https://github.com/antomanc/TrainerBridge/actions/workflows/ci.yml/badge.svg)](https://github.com/antomanc/TrainerBridge/actions)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)

> Tested on a standard ESP32 DevKit with the Decathlon Van Rysel D100. Other FTMS trainers
> should work: pair them by advertised name or MAC address.

## Why

Riding indoors with an app *and* a bike computer usually means compromises:
- the trainer drops the Garmin as soon as Zwift connects, or the other way round
- no speed or distance on the Garmin indoors, because there's no GPS and no wheel sensor
- ride data split between the app and your Garmin history
- a separate power meter or speed sensor just to record the ride twice

TrainerBridge sits in between and gives everyone what they need from one trainer connection.

## What you get

| | |
|---|---|
| **Apps** | Zwift, MyWhoosh, TrainerDay and any FTMS app see a controllable trainer: ERG target power, simulation grade, start/stop, routed to the real trainer |
| **Garmin** | Edge and watches pair it as a **Smart Trainer**, or as separate **Power** and **Speed** sensors |
| **Speed & distance** | Physics-based virtual speed on a flat road (CdA, rolling resistance, 85 kg rider + bike), with flywheel coasting so Garmin doesn't auto-pause |
| **Multi-client** | App and Garmin connected together; each gets the same telemetry, and control responses go only to the client that sent the command |
| **Recovery** | Trainer power-cycled? Clients get 0 W immediately, and the bridge keeps scanning and reconnects on its own |
| **Status LED** | Blinks while searching, solid once the trainer is connected. No serial monitor needed |
| **Diagnostics** | Serial console with live status, power calibration and manual ERG commands |

## Quick start

You need an ESP32 (the classic ESP32, e.g. a DevKit V1), a USB cable and
[Arduino CLI](https://arduino.github.io/arduino-cli/) (or the Arduino IDE with
ESP32 Arduino core **3.3.12** and NimBLE-Arduino **2.5.1**).

```bash
git clone https://github.com/antomanc/TrainerBridge && cd TrainerBridge
make deps                          # install the pinned core and library (once)
make upload PORT=/dev/ttyUSB0      # build with all warnings and flash
make monitor                       # optional: serial console at 115200 baud
```

Turn on the trainer: the LED blinks while the bridge looks for it and stays on once connected.
Then pair your devices with **TrainerBridge**:

**Garmin Edge or watch**: *Settings > Sensors > Add Sensor > Smart Trainer*, pick
TrainerBridge. You get power, speed and distance over one connection. If asked for a wheel
size, use **2096 mm**. (Alternatively, add it as a *Speed/Cadence* sensor and a *Power* sensor.)

**Zwift, MyWhoosh, TrainerDay**: in the pairing screen, choose TrainerBridge for both
**Controllable** and **Power Source**.

## Configure

Everything lives in [`TrainerBridge/Config.h`](TrainerBridge/Config.h). Defaults work for a
Van Rysel D100. The settings you are most likely to change:

| Setting | Default | |
|---|---|---|
| `TARGET_NAME_CONTAINS` | `"RYSEL"` | Case-insensitive part of the trainer's Bluetooth name |
| `TARGET_MAC` | `""` | Exact trainer MAC. Takes priority over the name; leave both empty to use the first FTMS trainer found |
| `DEVICE_NAME` | `"TrainerBridge"` | Name apps and Garmin see |
| `DEFAULT_POWER_SCALE` | `1.00` | Multiply the trainer's watts, if it reads high or low |
| `ENABLE_VIRTUAL_SPEED` | `true` | Physics-based speed and distance |
| `ENABLE_CSC_SERVICE` | `true` | Speed sensor service for Garmin |
| `VIRTUAL_WHEEL_CIRCUMFERENCE_M` | `2.096` | Match the wheel size set on your Garmin (700x25c) |
| `STATUS_LED_PIN` | `LED_BUILTIN` or `2` | Status LED GPIO; `STATUS_LED_ACTIVE_HIGH` sets polarity |
| `DEBUG_LOG` | `0` | Set to `1` for verbose serial logging while debugging |

Rider mass, CdA, rolling resistance and coasting are in the same file.
[→ How it works](docs/how-it-works.md) explains the model and the Bluetooth services.

## Serial console

Open the serial port at 115200 baud (`make monitor`):

```
status        Connection, power, virtual speed, distance and control queue state
scale 1.05    Change the power calibration until reboot
p 200         Send an ERG target of 200 W to the trainer (testing)
adv / noadv   Start or stop advertising to apps and Garmin
```

## Design

- **One trainer connection, many clients.** The ESP32 is the only device that talks to the
  trainer, so apps and Garmin never fight over it.
- **The trainer has the last word.** ERG and simulation commands are queued, sent one at a
  time, and the app gets the trainer's real answer, not an optimistic one. Commands that time
  out or hit a disconnect fail right away instead of leaving the app waiting.
- **Answers go to whoever asked.** Control responses are sent only to the client that sent
  the command, so the Garmin never sees an app's traffic (Garmin drops sensors that send
  unexpected indications).
- **Never blocks the Bluetooth stack.** Writes to the trainer happen in the main loop, not in
  Bluetooth callbacks. State shared between the two is locked.
- **Fails safe.** If the trainer disappears, every client sees 0 W immediately, not a frozen
  number.

## Development

```bash
make test       # host tests: the real sketch against Arduino/NimBLE test doubles (g++ only)
make check      # tests with AddressSanitizer + UBSan, then the real firmware build
```

The host tests run the actual firmware code through scanning, reconnecting 20 times, two
clients at once, and FTMS success, failure, timeout and late-response cases. They can't test the
radio, so hardware changes still need a ride on a real trainer:
[→ docs/testing.md](docs/testing.md). See [CONTRIBUTING.md](CONTRIBUTING.md) and
[CHANGELOG.md](CHANGELOG.md).

## License

[GPL-3.0](LICENSE) © 2026 Antonio Mancini

*TrainerBridge is an independent project, not affiliated with or endorsed by Garmin,
Decathlon, Van Rysel, Zwift, MyWhoosh or TrainerDay.*
