# TrainerBridge

TrainerBridge turns an ESP32 into a high-performance Bluetooth Low Energy (BLE) proxy between an indoor smart trainer (such as the Van Rysel D100), cycling training apps (Zwift, MyWhoosh, TrainerDay), and Garmin bike computers / watches.

- **Cycling Apps** (Zwift, MyWhoosh, TrainerDay) receive FTMS telemetry and control trainer resistance via standard FTMS Control Point commands.
- **Garmin Edge & Watches** receive live Power (Watts) and native simulated flat-terrain Speed (km/h) and Distance (km) without requiring GPS or external sensors.
- Simultaneous multi-client connections: Ride on Zwift while simultaneously recording on your Garmin Edge.

Tested and validated on a standard ESP32 with the Decathlon Van Rysel D100 FTMS smart trainer. Other FTMS trainers can be paired by advertised name or MAC address.

---

## Architecture & How It Works

```text
Real Trainer (FTMS 0x1826) 
        │
   [BLE Client]
        ▼
  TrainerBridge (ESP32)
   ├── Physics Engine (CdA 0.32, Crr 0.004, 85 kg mass, flywheel coasting)
   ├── Power Filter (Q8 EMA smoothing, scale calibration, stale watchdog)
   └── Coalescing Queue (Thread-safe FreeRTOS ring buffer)
        │
   [BLE Server / Proxy]
        ├── FTMS (0x1826) ────► Zwift / MyWhoosh / TrainerDay (Interactive ERG / Sim)
        │                 └──► Garmin Edge (Unified Smart Trainer mode)
        ├── CSC  (0x1816) ────► Garmin Speed & Cadence (1/1024s wheel event time)
        └── CPS  (0x1818) ────► Garmin Cycling Power (Watts)
```

### Tri-Service GATT Implementation
1. **Fitness Machine Service (FTMS `0x1826`)**:
   - `0x2AD2` (Indoor Bike Data): Transmits Instantaneous Power, Instantaneous Speed ($0.01\text{ km/h}$ resolution), and Total Distance ($1\text{ m}$ resolution, `uint24`).
   - `0x2ACC` (Fitness Machine Feature): Declares support for Power, Resistance, and Total Distance.
   - `0x2AD9` (Control Point): Receives app commands (target power, simulation grade, start/stop) and queues them for deterministic delivery to the real trainer.
2. **Cycling Speed and Cadence (CSC `0x1816`)**:
   - `0x2A5B` (CSC Measurement): Broadcasts cumulative wheel revolutions and high-resolution wheel event timing ($1/1024\text{ s}$ resolution) for native Garmin speed sensor pairing.
   - `0x2A5C` (CSC Feature) & `0x2A5D` (Sensor Location): Full Bluetooth SIG compliance.
3. **Cycling Power Service (CPS `0x1818`)**:
   - `0x2A63` (Power Measurement): Broadcasts Instantaneous Power in Watts for standalone power meter profiles.

---

## Physics Engine (Virtual Speed & Distance)

On flat road ($0\%$ grade) with no wind, the power delivered to the wheels is opposed by aerodynamic drag and rolling resistance:

$$P_{\text{wheel}} = \left(C_{\text{rr}} \cdot m_{\text{total}} \cdot g\right) v + \left(\frac{1}{2} \cdot \rho \cdot C_d A\right) v^3$$

* **Total mass ($m$)**: $85\text{ kg}$ (rider $75\text{ kg}$ + bike $10\text{ kg}$)
* **Rolling resistance ($C_{\text{rr}}$)**: $0.004$ (standard performance road tire on smooth asphalt)
* **Aerodynamic drag ($C_d A$)**: $0.32\text{ m}^2$ (standard endurance road position on the hoods)
* **Air density ($\rho$)**: $1.225\text{ kg/m}^3$ (sea level)
* **Drivetrain efficiency ($\eta$)**: $0.98$ ($98\%$)
* **Coasting Deceleration**: $1.5\text{ m/s}^2$ smooth flywheel coasting when pedaling ceases, preventing sudden auto-pauses on Garmin.

**Speed Benchmarks:**
* $100\text{ W} \to \approx 26.0\text{ km/h}$
* $150\text{ W} \to \approx 30.5\text{ km/h}$
* $200\text{ W} \to \approx 34.0\text{ km/h}$
* $250\text{ W} \to \approx 36.9\text{ km/h}$
* $300\text{ W} \to \approx 39.4\text{ km/h}$

---

## Requirements

- ESP32 Arduino core **3.3.10**
- [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino) **2.5.0**
- Arduino CLI or Arduino IDE

---

## Configuration

Edit [`TrainerBridge/Config.h`](TrainerBridge/Config.h):

- `DEVICE_NAME`: BLE name exposed to apps and bike computers (default: `"TrainerBridge"`).
- `TARGET_NAME_CONTAINS`: case-insensitive substring of the trainer's advertised name (default: `"RYSEL"`).
- `TARGET_MAC`: optional exact trainer MAC address (takes priority over name matching).
- `DEFAULT_POWER_SCALE`: measured-power multiplier (default: `1.00`).
- `ENABLE_VIRTUAL_SPEED`: enables physics-based speed and distance calculation.
- `ENABLE_CSC_SERVICE`: enables the standard BLE Speed Sensor GATT service.
- `VIRTUAL_WHEEL_CIRCUMFERENCE_M`: virtual wheel circumference (default: `2.096f` for $700\times 25\text{c}$).
- `DEBUG_LOG`: set to `1` temporarily for verbose serial debugging.

---

## Build and Flash

Install dependencies and compile with zero warnings:

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
arduino-cli upload --port /dev/ttyUSB0 --fqbn esp32:esp32:esp32 TrainerBridge
```

---

## Pairing Guide

### Connecting to Garmin Edge or Watch

You can pair using either of the following methods:

#### Method 1: Smart Trainer Mode (Recommended)
1. Power on your trainer and the ESP32.
2. On your Garmin, navigate to **Settings > Sensors > Add Sensor > Smart Trainer**.
3. Select **TrainerBridge** and add it.
4. Garmin receives Power, Speed, and Distance simultaneously over a single unified Bluetooth channel.
5. If prompted for a wheel size in the sensor details, set it to **2096 mm**.

#### Method 2: Speed + Power Sensor Mode
1. On your Garmin, navigate to **Settings > Sensors > Add Sensor > Speed/Cadence** (or **Search All**).
2. Select **TrainerBridge** and add it. Set wheel size to **2096 mm**.
3. Add **TrainerBridge** under **Power** sensors if not already paired.
4. Both services communicate concurrently over the same physical Bluetooth link.

### Connecting to Cycling Apps (Zwift / MyWhoosh / TrainerDay)
1. In the app's pairing screen, search for **Controllable** and **Power Source**.
2. Select **TrainerBridge**.
3. Resistance, target power, and simulation gradients are automatically routed to the real trainer.

---

## Serial Diagnostics & CLI

Open the serial monitor at **115200 baud**. Commands:

- `status`: prints connection status, power, virtual speed, distance, wheel revs, and queue state.
- `scale <value>`: adjusts the live power calibration multiplier (e.g., `scale 1.05`).
- `p <watts>`: queues a manual ERG target power command for testing (e.g., `p 200`).
- `adv` / `noadv`: manually starts or stops proxy BLE advertising.

---

## Project Structure

```text
TrainerBridge/
├── Config.h              User parameters & physics constants
├── FtmsConstants.h       BLE UUIDs, opcodes, and bitfield flags
├── PowerFilter.h/.cpp    Q8 EMA power filter & zero-lag coasting
├── CommandQueue.h/.cpp   Thread-safe FreeRTOS coalescing command queue
├── TrainerMatcher.h/.cpp Trainer MAC/name matching with boot self-check
├── VirtualSpeedModel.h/.cpp Flat-road aerodynamic engine & dual-resolution event timers
└── TrainerBridge.ino     Firmware entrypoint, GATT services & BLE state machines
```

---

## License

TrainerBridge is licensed under the GNU General Public License v3.0. See [LICENSE](LICENSE).

*TrainerBridge is an independent open-source project and is not affiliated with or endorsed by Garmin, Decathlon, Van Rysel, Zwift, MyWhoosh, or TrainerDay.*
