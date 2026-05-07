# Smart Health Band

Firmware for the Smart Health Band prototype built on `nrf52dk_nrf52832` with Zephyr RTOS. The application combines motion sensing, temperature sensing, biometric acquisition, local haptic feedback, and Bluetooth Low Energy notifications in a single wearable-oriented firmware image.

## What The Firmware Does

- Reads body or skin-adjacent temperature from the TMP117 over I2C.
- Uses the BMI270 for wake-on-motion plus tap and double-tap gesture detection.
- Runs a MAX32664 session for heart rate, SpO2, and blood-pressure-style output.
- Publishes temperature, event, and vitals data over a custom BLE GATT service.
- Drives local UI feedback with LEDs and a haptic motor.
- Stores MAX32664 calibration data in flash using NVS so later boots can skip full recalibration when valid data already exists.

## Hardware Summary

| Item | Value |
|---|---|
| MCU / board target | `nrf52dk_nrf52832` |
| SDK / RTOS | nRF Connect SDK 2.8.x / Zephyr |
| Debug output | Segger RTT |
| Build system | CMake + `west` |

### Key Pins

| Signal | Pin |
|---|---|
| I2C SCL | P0.00 |
| I2C SDA | P0.04 |
| BMI270 INT1 | P0.31 |
| BMI270 INT2 | P0.30 |
| TMP117 ALERT | P0.11 |
| TMP LED | P0.08 |
| BMI / vitals LED | P0.13 |
| MAX32664 RSTN | P0.21 |
| MAX32664 MFIO | P0.07 |
| Haptic motor gate | P0.06 |

### I2C Devices

| Device | Address |
|---|---|
| TMP117 | `0x48` |
| BMI270 | `0x68` |
| MAX32664D | `0x55` |

## Firmware Architecture

### `src/main.c`

Coordinates startup, launches BLE and sensor modules, starts the MAX32664 session, and runs the main polling loop. Temperature is sampled on the poll interval, while BMI270 gesture wakes and MAX32664 vitals events are handled asynchronously through the power-event layer.

### `src/ble.c`

Defines the custom BLE service and three text-based characteristics:

- Temperature: formatted as strings such as `32.6 C`
- Event: formatted as `TAP_SINGLE`, `TAP_DOUBLE`, or `ALERT_ACKNOWLEDGED`
- Vitals: formatted as `HR: 75.0 bpm | SpO2: 98.5% | BP: 120/80 mmHg`

All three characteristics support `READ` and `NOTIFY`.

### `src/bmi270.c`

Configures the BMI270 through Zephyr's sensor API, arms the any-motion interrupt, and classifies impulsive motion into tap or double-tap events. The IRQ path is intentionally filtered so general hand movement does not get treated as a tap.

### `src/max32664.c`

Handles hub reset, runtime-mode entry, firmware version probe, calibration persistence, first-boot calibration, and ongoing vitals estimation. If estimation stops unexpectedly, the module attempts to recover and restart the session.

### `src/tmp117.c`

Performs raw temperature reads and converts the TMP117 register value into Zephyr `sensor_value` temperature output.

### `src/board.c`

Owns the local user-feedback outputs used by the firmware today:

- TMP LED pulse on temperature sample
- BMI LED pulse on vitals update
- Haptic alert start/stop
- Haptic feedback pulse for double-tap confirmation

### `src/power.c`

Provides the wake/event bridge between interrupt context, worker threads, and the main loop.

## BLE Service

Service UUID: `4fafc201-1fb5-459e-8fcc-c5c9c331914b`

| Characteristic | UUID |
|---|---|
| Temperature | `4fafc202-1fb5-459e-8fcc-c5c9c331914b` |
| Event | `4fafc203-1fb5-459e-8fcc-c5c9c331914b` |
| Vitals | `4fafc204-1fb5-459e-8fcc-c5c9c331914b` |

The device advertises as `SmartHealth Band`.

## Runtime Flow

1. Initialize board GPIO, wake-event handling, and BLE.
2. Probe the TMP117 and publish the first temperature value.
3. Initialize the BMI270 and enable any-motion wake on INT1.
4. Initialize the MAX32664 and start either calibration or estimation, depending on whether a saved calibration vector is available.
5. Enter the main loop and maintain a combined status snapshot over RTT while BLE notifications are sent as sensor events arrive.

Gesture behavior in the current code:

- Single tap with no active alert sends `TAP_SINGLE`.
- Single tap during an active alert stops the alert and sends `ALERT_ACKNOWLEDGED`.
- Double tap triggers a short haptic feedback pulse and sends `TAP_DOUBLE`.

## Repository Layout

```text
.
├── boards/
├── datasheets/
├── dts/
├── include/
├── src/
├── .gitignore
├── CMakeLists.txt
├── Kconfig
├── prj.conf
└── README.md
```

- `boards/`, `dts/`, `include/`, and `src/` contain the firmware source and hardware description.
- `datasheets/` contains hardware reference material and project pin notes.
- `Refrence_Code/` is local reference material and is intentionally excluded from version control.

## Build

```bash
west build -b nrf52dk_nrf52832
```

For a pristine rebuild:

```bash
west build -p always -b nrf52dk_nrf52832
```

## Flash

```bash
west flash
```
