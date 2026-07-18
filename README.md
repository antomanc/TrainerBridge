# TrainerBridge

TrainerBridge turns an ESP32 into a Bluetooth Low Energy bridge between an FTMS indoor trainer, training apps, and Garmin bike computers.

- Zwift, MyWhoosh, TrainerDay, and other compatible apps receive FTMS data and can control the trainer.
- Garmin and other bike computers receive Cycling Power Service (CPS) power and virtual wheel-revolution data.
- Both connections can be used at the same time.

The firmware has been tested with a standard ESP32 and a Van Rysel D100 FTMS trainer. Other FTMS trainers can be selected by advertised name or MAC address.

## How it works

```text
FTMS trainer -> TrainerBridge
                  |-> FTMS -> Zwift / MyWhoosh / TrainerDay
                  \-> CPS  -> Garmin
```

TrainerBridge forwards measured trainer power with a small, responsive EMA filter. FTMS control commands from an app are queued and forwarded to the real trainer outside BLE callbacks for stability.

Virtual distance is calculated from measured power using a fixed flat-road model rather than trainer flywheel speed. The default model uses 75 kg total mass, CdA 0.35, Crr 0.004, 97% drivetrain efficiency, and a 2105 mm wheel circumference. At 150 W it produces approximately 29.8 km/h, or 5 km in 10 minutes. The goal is repeatable indoor statistics, not matching the distance calculated by Zwift or MyWhoosh.

## Requirements

- ESP32 Arduino core 3.3.10
- [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino) 2.5.0
- Arduino CLI or Arduino IDE

## Configuration

Edit [`TrainerBridge/Config.h`](TrainerBridge/Config.h):

- `DEVICE_NAME`: BLE name exposed to apps and bike computers.
- `TARGET_NAME_CONTAINS`: case-insensitive substring of the trainer's advertised name. The default `RYSEL` matches names such as `VANRYSEL D100`, `VAN RYSEL D100`, and `VARYSEL D100`.
- `TARGET_MAC`: optional exact trainer MAC address. A non-empty MAC takes priority over name matching.
- `DEFAULT_POWER_SCALE`: measured-power calibration; `1.00` leaves trainer watts unchanged.
- `VIRTUAL_*`: rider, road, and wheel values used by the virtual-distance model.
- `DEBUG_LOG`: set to `1` temporarily for verbose BLE logs.

If both trainer selectors are empty, TrainerBridge connects to the first device advertising FTMS.

## Build and upload

Install the pinned dependencies and compile:

```bash
arduino-cli core update-index \
  --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core install esp32:esp32@3.3.10 \
  --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli lib install NimBLE-Arduino@2.5.0
arduino-cli compile --warnings all --fqbn esp32:esp32:esp32 TrainerBridge
```

Upload to a connected ESP32:

```bash
arduino-cli upload --port /dev/ttyUSB0 \
  --fqbn esp32:esp32:esp32 TrainerBridge
```

Change the serial port or FQBN when using a different board. GitHub Actions compiles every push and pull request with the same dependency versions.

## Pairing

1. Power on the trainer and ESP32.
2. Remove an older proxy pairing after changing firmware or GATT features.
3. Pair `TrainerBridge` with Garmin as a power sensor.
4. Set wheel circumference to 2105 mm if Garmin offers the setting.
5. Select `TrainerBridge` as the controllable trainer in the FTMS app.

Power reporting and trainer control are independent of virtual-distance support. A bike computer must consume the wheel-revolution fields included in CPS to record that distance.

## Serial diagnostics

Open a serial monitor at 115200 baud and send `status`. It reports the selected trainer-matching rule, BLE connection state, measured and output power, virtual speed and distance, packet counters, and command-queue state.

At boot, TrainerBridge also runs self-checks for the trainer matcher and virtual-speed model. Both should report `OK`.

## Project structure

```text
TrainerBridge/Config.h           Configuration
TrainerBridge/TrainerBridge.ino  Firmware
.github/workflows/compile.yml    Automated compile check
README.md                        Documentation
LICENSE                          GNU GPL v3
```

## License

TrainerBridge is licensed under the GNU General Public License v3.0. See [LICENSE](LICENSE).

TrainerBridge is not affiliated with or endorsed by Garmin, Zwift, MyWhoosh, TrainerDay, Decathlon, Van Rysel, or any other equipment manufacturer or subscription service.
