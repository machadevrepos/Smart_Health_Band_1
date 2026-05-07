# Smart Health Band - `pinout.md`

This file is a firmware-facing hardware reference for the **Smart Health Band** PCB.
It is written to help an AI coding system understand the board clearly before generating drivers, board definitions, and application firmware.

It combines:
- the user-provided PCB wiring description,
- the uploaded schematic,
- and the sensor datasheets / hub documentation.

---

## 1. Board Summary

### Main MCU
- **Module:** `BMD-350` (based on Nordic `nRF52832`) 
- MCU GPIO names below use the Nordic style such as `P0.00`, `P0.04`, etc.
- `P0.21` is the module reset pin (`RESET_N`) on the BMD-350 pinout. fileciteturn0file4L1-L14

### Main functional blocks on the PCB
- `TMP117` temperature sensor on main I2C bus
- `BMI270` IMU on main I2C bus
- `MAX32664D` biometric hub on main I2C bus through logic-level shifters
- `MAX30101` optical sensor on the **internal sensor I2C bus of the MAX32664D**
- vibration motor driver stage
- 3 status LEDs:
  - temperature LED
  - BMI LED
  - heart / biometric LED
- battery input, charger, boost, 3.3V LDO, and 1.8V LDO

### Important architecture note
The biometric section is **not** a plain direct-to-MCU sensor.
The host MCU talks to `MAX32664D`; the `MAX32664D` talks to `MAX30101` over its own sensor-side I2C bus. This is the intended architecture for MAX32664 Version D. fileciteturn0file2L1-L8 fileciteturn0file2L14-L16

---

## 2. Power Domains

The schematic has multiple voltage rails. These should be modeled clearly in firmware comments and board docs because they affect logic levels and bring-up.

### `BAT+`
- Li-ion battery rail
- feeds charger / boost converter

### `5V`
- generated from battery through boost stage
- used by:
  - `AMS1117-3.3` regulator input
  - `AP2125 1.8V` regulator input
  - `MAX30101 VLED+` rail

### `3V3`
Main logic rail for the MCU and most board peripherals.
Used by:
- `BMD-350`
- `TMP117`
- `BMI270`
- LEDs and pull-ups on main bus
- motor driver gate control source domain

### `1V8`
Used by the biometric hub / low-voltage side.
Used by:
- `MAX32664D VDD`
- `MAX30101 VDD`
- low-voltage side of the I2C level shifters
- 32.768 kHz crystal domain for MAX32664D

### Important voltage note for firmware engineers
- Main MCU GPIO and main shared I2C are on **3.3V domain**.
- `MAX32664D` host interface is on **1.8V domain**, but the board includes **I2C logic-level shifters** between the MCU bus and the hub bus.
- `MAX30101` uses **1.8V logic** and **5V LED supply** as expected from its datasheet. fileciteturn0file5L1-L8

---

## 3. Main I2C Bus Topology

## MCU-side main I2C pins
- **SCL:** `P0.00`
- **SDA:** `P0.04`

The BMD-350 pinout confirms:
- `P0.00` is also `XTAL1`
- `P0.04` is GPIO / analog-capable and can be used for I2C
- Nordic peripherals such as I2C may be mapped to GPIO pins in firmware. fileciteturn0file4L1-L14

### Important note about `P0.00`
- `P0.00` is normally multiplexed with the 32.768 kHz crystal function on the module.
- In this design, the external crystal for the MCU is **not used**, so `P0.00` is intentionally repurposed as the I2C SCL pin.
- This must be reflected in firmware pin configuration.

### Devices present on the main 3.3V I2C side
- `TMP117`
- `BMI270`
- level-shifter high side for `MAX32664D`

### Devices **not** directly present on the main 3.3V I2C side
- `MAX30101`

### Pull-ups
The schematic shows 4.7k pull-ups on the main I2C lines. This is consistent with I2C operation and should be assumed present in hardware.

---

## 4. I2C Domains and Level Shifting

This board has **two different I2C voltage domains**:

### Domain A: `3V3` main bus
Signals:
- `SCL`
- `SDA`

Connected to:
- MCU
- TMP117
- BMI270
- high side of I2C level shifters

### Domain B: `1V8` hub-side bus
Signals:
- `SCL_L`
- `SDA_L`

Connected to:
- `MAX32664D` slave I2C interface (`SLAVE_SCL`, `SLAVE_SDA`)
- low side of I2C level shifters

### Firmware implication
Firmware should still use **one MCU I2C peripheral** on `P0.00/P0.04`; the level shifting is transparent at software level.
But the documentation must clearly state that the hub is behind a **1.8V translated interface**.

---

## 5. Exact MCU Pin Allocation

| MCU Pin | Direction in firmware | Net / Function | Connected hardware | Notes |
|---|---|---|---|---|
| `P0.00` | I2C SCL | `SCL` | Main I2C bus | Shared with XTAL1 function, used as I2C here |
| `P0.04` | I2C SDA | `SDA` | Main I2C bus | Shared bus for TMP117, BMI270, MAX32664D (via shifter) |
| `P0.11` | GPIO input | `TEMP_ALERT` | TMP117 ALERT pin | Alert / data-ready style line |
| `P0.08` | GPIO output | `TMP_LED` | Temperature status LED | LED polarity should be verified on board bring-up |
| `P0.06` | GPIO output | `MCU_GPIO` | Vibration motor transistor gate | **Inverted / non-standard drive behavior** |
| `P0.31` | GPIO input / interrupt | `INT1` | BMI270 INT1 | Use interrupt-capable GPIO config |
| `P0.30` | GPIO input / interrupt | `INT2` | BMI270 INT2 | Use interrupt-capable GPIO config |
| `P0.13` | GPIO output | `BMI_LED` | BMI status LED | LED polarity should be verified on board bring-up |
| `P0.07` | GPIO bidirectional / control | `MFIO` | MAX32664D MFIO | Used for mode control / status / interrupt behavior |
| `P0.12` | GPIO output | `HEART_LED` | Biometric status LED | For MAX32664D + MAX30101 subsystem |
| `P0.21` | Reset line, not normal GPIO | `RSTN` | MCU reset line and MAX32664D reset line | Shared hardware reset domain from schematic |

---

## 6. Reset and Boot-Control Signals

## MCU reset
- `BMD-350 P0.21` is the module reset pin (`RESET_N`). fileciteturn0file4L1-L14
- In the schematic, there is a reset network on net `RSTN` with pull-up and reset switch.

## MAX32664D reset
- `MAX32664D RSTN` is also tied to the `RSTN` net in the schematic.
- This means the hub reset is **not on a separate normal GPIO** from the MCU; it is tied into the board reset network.

### Firmware implication
- There is **no dedicated independent MCU GPIO listed by the user for MAX32664 reset toggling**.
- Assume the hub is reset by hardware reset / board reset unless later hardware tests show otherwise.
- The only direct hub control GPIO exposed to firmware is `MFIO` on `P0.07`.

## MAX32664D MFIO
- `MFIO` is connected to `P0.07`
- According to the MAX32664 user guide, `MFIO` is used for:
  - bootloader/application mode selection during reset
  - host wake/interrupt style interaction in application mode. fileciteturn0file2L14-L16

### Important implementation note
For this board, firmware should treat:
- `P0.07` = `MAX32664_MFIO_PIN`
- `RSTN` = shared reset domain, not a free GPIO unless proven otherwise

---

## 7. TMP117 Temperature Sensor

### Connections
- `SCL` -> main I2C SCL -> `P0.00`
- `SDA` -> main I2C SDA -> `P0.04`
- `ALERT` -> `P0.11`
- supply -> `3V3`
- ground -> `GND`

### Addressing note
TMP117 address depends on the `ADD0` strap. The datasheet supports selecting the address by tying `ADD0` to GND, V+, SDA, or SCL. fileciteturn0file8L1-L8

### Schematic-specific note
In the schematic, `ADD0` is tied through a resistor to `GND`, so the device should be treated as the default GND-strapped address variant unless hardware inspection shows otherwise.

### Firmware notes
- Use the main I2C peripheral.
- Configure `P0.11` as the alert/data-ready input.
- Temperature LED is on `P0.08`.

### AI-facing summary
```text
TMP117 is a direct 3.3V I2C peripheral on the MCU main bus.
The ALERT pin is wired to P0.11.
Its LED indicator is on P0.08.
```

---

## 8. BMI270 IMU

### Connections
- `SCL` -> main I2C SCL -> `P0.00`
- `SDA` -> main I2C SDA -> `P0.04`
- `INT1` -> `P0.31`
- `INT2` -> `P0.30`
- `CSB` tied high for I2C mode in schematic context
- supply -> `3V3`
- I/O supply -> `3V3`

### BMI270 voltage note
BMI270 supports I2C and wide supply ranges, including:
- `VDD`: 1.71V to 3.6V
- `VDDIO`: 1.2V to 3.6V. fileciteturn0file8L1-L8

### Interrupt notes
- `INT1` = MCU `P0.31`
- `INT2` = MCU `P0.30`
- These should be configured as interrupt inputs.

### Important firmware note
BMI270 requires configuration-file initialization after reset / POR before normal operation. Bosch documents an initialization sequence involving config upload and checking `INTERNAL_STATUS`. fileciteturn0file8L1-L8

### AI-facing summary
```text
BMI270 is a direct 3.3V I2C peripheral on the MCU main bus.
INT1 is on P0.31.
INT2 is on P0.30.
BMI status LED is on P0.13.
```

---

## 9. MAX32664D + MAX30101 Biometric Subsystem

This subsystem must be documented as a **single logical firmware block**.

## 9.1 MAX32664D hub-side architecture
The MAX32664 Version D is intended for:
- finger-based heart rate,
- SpO2,
- and estimated blood pressure,
using a MAX30101/30102-style optical front end over the sensor-side I2C bus. fileciteturn0file2L1-L8 fileciteturn0file3L1-L8

## 9.2 MAX32664D host-side board connections
- host I2C to MCU through level shifters
- `MFIO` -> `P0.07`
- `RSTN` -> shared board reset net
- supply -> `1V8`
- 32.768 kHz crystal fitted to hub

### I2C address note
The schematic labels the hub as address `0x55`.
The quick-start guide/user guide often show `0xAA` write and `0xAB` read, which are the **8-bit bus addresses** corresponding to the **7-bit address `0x55`**. fileciteturn0file1L1-L8

### MFIO behavior
MAX32664 MFIO is used by the host for boot/application mode and host interaction. The user guide explicitly describes `RSTN` + `MFIO` as the two control lines used by the host. fileciteturn0file2L14-L16

## 9.3 MAX30101 board-side architecture
The `MAX30101` is **not connected directly to the MCU**.
It is connected to the hub's internal sensor bus:
- `MAX32664 SENSOR_SCL` <-> `MAX30101 SCL`
- `MAX32664 SENSOR_SDA` <-> `MAX30101 SDA`
- `MAX30101 INT` -> `MAX32664 HR_INT`

### MAX30101 power rails
- logic supply `VDD` = `1V8`
- LED supply `VLED+` = `5V`
- grounds `GND` / `PGND`

This matches the MAX30101 requirements of 1.8V logic with separate LED supply. fileciteturn0file5L1-L8

### Important host-firmware rule
**Do not scan for MAX30101 on the MCU main I2C bus.**
It is not a direct host peripheral in this design.

## 9.4 Biometric LED
- `HEART_LED` -> `P0.12`
- Use this as the subsystem status indicator for the hub + optical sensor.

### AI-facing summary
```text
Treat MAX32664D + MAX30101 as one subsystem.
The MCU talks only to MAX32664D on the main I2C bus.
MAX30101 is behind the hub on the hub's internal sensor I2C bus.
MFIO is on P0.07.
Hub reset is tied into shared RSTN hardware reset.
Subsystem LED is on P0.12.
```

---

## 10. Vibration Motor Driver

### Schematic topology
The vibration motor is not driven directly by the MCU pin.
The control stage is:
- transistor `Q4 = DMG2307L`
- `Source` tied to `3V3`
- `Gate` driven from MCU net `MCU_GPIO` through resistor
- `Drain` connected to motor node
- diode present for flyback handling

### MCU connection
- `P0.06` -> net `MCU_GPIO` -> transistor gate

### Very important logic note
Because the transistor is arranged in a **high-side / non-standard orientation**, motor logic must be treated as **inverted until verified on hardware**.

### Firmware recommendation
Do not hardcode plain active-high behavior.
Use abstraction:

```c
#define MOTOR_CTRL_PIN        NRF_GPIO_PIN_MAP(0, 6)
#define MOTOR_ACTIVE_STATE    0   // verify on hardware
#define MOTOR_INACTIVE_STATE  1   // verify on hardware
```

### AI-facing summary
```text
P0.06 controls the motor transistor gate.
Motor logic is likely inverted because the transistor source is tied to 3.3V.
Firmware must abstract motor on/off polarity.
```

---

## 11. LED Mapping

The schematic shows three indicator LEDs with series resistors.
Exact polarity should still be verified during bring-up.

| LED Net | MCU Pin | Purpose |
|---|---|---|
| `TMP_LED` | `P0.08` | Temperature subsystem indicator |
| `HEART_LED` | `P0.12` | Biometric subsystem indicator |
| `BMI_LED` | `P0.13` | IMU subsystem indicator |

### Firmware note
Because LED drive polarity is not guaranteed only from the partial netlist, firmware should use symbolic macros:

```c
#define LED_ON_STATE   1   // verify on hardware
#define LED_OFF_STATE  0   // verify on hardware
```

If testing shows the LEDs are low-side sunk instead, flip these macros only once in board config.

---

## 12. Interrupt / Alert Mapping

| Source | Net | MCU pin | Notes |
|---|---|---|---|
| TMP117 | `TEMP_ALERT` | `P0.11` | Alert / threshold / data-ready style pin |
| BMI270 | `INT1` | `P0.31` | Main motion / feature interrupt |
| BMI270 | `INT2` | `P0.30` | Secondary motion / feature interrupt |
| MAX30101 -> MAX32664 | `HR_INT` internal to hub block | not direct to MCU | Handled by MAX32664D |
| MAX32664D | `MFIO` | `P0.07` | Control / mode / host interaction pin |

---

## 13. Reserved / Special Pins from Schematic Context

These are important for AI-generated firmware so they are **not accidentally reused**.

### `P0.21`
- board reset pin / `RESET_N`
- tied to shared `RSTN` hardware reset domain
- **do not use as a normal GPIO**

### `P0.00`
- used as I2C SCL
- also multiplexed with crystal function on MCU module
- safe in this design only because external crystal is intentionally not used

### UART test pads / connector nets
The schematic also exposes:
- `TX`
- `RX`
- `SWDIO`
- `SWCLK`
- `RSTN`
- `5V`
- `GND`

These are for programming / debug / factory access and should be treated as support interfaces, not application GPIOs.

---

## 14. Board-Level Firmware Naming Recommendation

```c
// =========================
// Main I2C bus
// =========================
#define PIN_I2C_SCL              NRF_GPIO_PIN_MAP(0, 0)
#define PIN_I2C_SDA              NRF_GPIO_PIN_MAP(0, 4)

// =========================
// TMP117
// =========================
#define PIN_TMP117_ALERT         NRF_GPIO_PIN_MAP(0, 11)
#define PIN_LED_TEMP             NRF_GPIO_PIN_MAP(0, 8)

// =========================
// Motor
// =========================
#define PIN_MOTOR_CTRL           NRF_GPIO_PIN_MAP(0, 6)
#define MOTOR_ACTIVE_STATE       0   // verify on hardware
#define MOTOR_INACTIVE_STATE     1   // verify on hardware

// =========================
// BMI270
// =========================
#define PIN_BMI270_INT1          NRF_GPIO_PIN_MAP(0, 31)
#define PIN_BMI270_INT2          NRF_GPIO_PIN_MAP(0, 30)
#define PIN_LED_BMI              NRF_GPIO_PIN_MAP(0, 13)

// =========================
// MAX32664D biometric hub
// =========================
#define PIN_MAX32664_MFIO        NRF_GPIO_PIN_MAP(0, 7)
#define PIN_LED_HEART            NRF_GPIO_PIN_MAP(0, 12)
#define MAX32664_I2C_ADDR_7BIT   0x55

// =========================
// Shared reset line
// =========================
// P0.21 is RESET_N on BMD-350; do not use as normal GPIO.
```

---

## 15. Per-Device Communication Summary for AI Firmware Generation

### TMP117
- bus: main MCU I2C
- level: 3.3V
- direct MCU peripheral: yes
- interrupt: yes, `P0.11`
- LED: yes, `P0.08`

### BMI270
- bus: main MCU I2C
- level: 3.3V
- direct MCU peripheral: yes
- interrupts: yes, `P0.31`, `P0.30`
- LED: yes, `P0.13`

### MAX32664D
- bus: main MCU I2C through level shifter
- level at hub side: 1.8V
- direct MCU peripheral: yes
- direct control pin: `MFIO` on `P0.07`
- reset pin: tied to shared `RSTN`, not a separate normal GPIO
- LED: yes, `P0.12`

### MAX30101
- bus: internal sensor I2C behind MAX32664D
- level: 1.8V logic, 5V LED supply
- direct MCU peripheral: no
- direct MCU interrupt: no
- host-visible through: MAX32664D only

---

## 16. Bring-Up Order Recommended for AI / Firmware Agent

1. Configure all board GPIO defaults first:
   - LEDs = outputs, safe off state
   - motor control = output, safe off state
   - TMP117 alert = input
   - BMI270 INT1/INT2 = inputs with interrupt capability
   - MAX32664 MFIO = controlled GPIO

2. Bring up main I2C on:
   - `P0.00` SCL
   - `P0.04` SDA

3. Probe / initialize in this order:
   - TMP117
   - BMI270
   - MAX32664D

4. Do **not** expect MAX30101 on main bus scan.

5. Implement biometric hub control using:
   - main I2C address `0x55`
   - `MFIO` handling
   - shared-reset-aware initialization logic

6. Add board abstraction macros for:
   - LED polarity
   - motor polarity

7. Only after raw comms are stable, add:
   - TMP117 alert handling
   - BMI270 interrupts / gestures
   - MAX32664 biometric streaming
   - motor haptics

---

## 17. Critical Assumptions Explicitly Captured

These assumptions are intentionally written here so an AI code generator does not make wrong hardware guesses.

1. `P0.00` is intentionally used as I2C SCL because MCU external crystal is not used.
2. `MAX30101` is **not** on the MCU main I2C bus.
3. `MAX32664D` is the only host-visible biometric device.
4. The board uses **I2C level shifting** between the 3.3V MCU bus and 1.8V hub bus.
5. `MAX32664D RSTN` is tied to shared board reset and is **not** currently documented as a separate MCU GPIO.
6. Motor drive logic must be treated as **inverted until confirmed on hardware**.
7. LED polarity should also be verified on hardware and abstracted in software.

---

## 18. One-Paragraph Summary

This board uses a BMD-350 (nRF52832-based) MCU. The main host I2C bus is on `P0.00` (SCL) and `P0.04` (SDA). `TMP117` is directly connected on this bus with `ALERT` on `P0.11` and LED on `P0.08`. `BMI270` is directly connected on this bus with `INT1` on `P0.31`, `INT2` on `P0.30`, and LED on `P0.13`. The biometric subsystem is built from `MAX32664D + MAX30101`, where the MCU talks only to `MAX32664D` at 7-bit I2C address `0x55`; `MAX30101` is behind the hub on the hub's internal sensor I2C bus. `MAX32664D MFIO` is connected to `P0.07`, while hub `RSTN` is tied into the shared board reset net rather than a separate MCU GPIO. The biometric LED is on `P0.12`. The motor control signal is `P0.06`, but the transistor stage is arranged in a way that likely inverts motor on/off logic, so firmware must abstract motor polarity.
