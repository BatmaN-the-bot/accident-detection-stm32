# accident-detection-stm32
This project implements a fully autonomous, embedded vehicle safety system capable of preventing drunk driving, detecting high-impact collisions and rollovers in real time, and automatically dispatching emergency alerts with precise GPS coordinates to emergency services and pre-configured contacts — all without any human intervention.
# 🚨 Automatic Accident Detection, Prevention & Emergency Response (AADR)

## Project Overview

A complete embedded C firmware for a vehicle safety system that:
- **Prevents** drunk driving by locking the engine
- **Detects** crashes using high-G force and rollover sensors
- **Responds** automatically by sending GPS-tagged SMS and making an emergency call

---

## Hardware Bill of Materials

| Component | Part Number | Purpose | Interface |
|-----------|-------------|---------|-----------|
| **MCU** | STM32F103C8T6 (Blue Pill) | Main controller | — |
| **Gas Sensor** | MQ-3 | Alcohol/BAC detection | ADC (PA0) |
| **IMU** | MPU-6050 | Acceleration + gyroscope | I2C (PB6/PB7) |
| **GPS** | u-blox NEO-6M or NEO-8M | Location coordinates | UART1 (PA10) |
| **GSM** | SIM800L or SIM900A | SMS + voice call | UART2 (PA2/PA3) |
| **Buzzer** | 5V passive buzzer | Alerts and SOS pattern | GPIO (PA8) |
| **Button** | Momentary NO switch | Cancel false alarm | GPIO (PB0) |
| **Relay** | 5V single channel relay | Engine ignition control | GPIO (PB1) |
| **LED** | Onboard PC13 | Status indicator | GPIO (PC13) |
| **LCD** | 16x2 I2C (optional) | Status display | I2C (shared) |

### Power Requirements
```
Car 12V battery
    └─ LM7805 → 5V → SIM800L, MQ-3, Relay, Buzzer
              → AMS1117-3.3 → 3.3V → STM32, MPU-6050, NEO-6M
```

---

## Pin Connections (STM32F103C8T6)

```
STM32 Blue Pill         Component
─────────────────────────────────────────────────────
PA0  (ADC1_CH0)     ──► MQ-3 Analog Output (AO)
PA1  (GPIO IN)      ──► MQ-3 Digital Output (DO)  [optional]
PB6  (I2C1_SCL)     ──► MPU-6050 SCL
PB7  (I2C1_SDA)     ──► MPU-6050 SDA
PA10 (USART1_RX)    ──► NEO-6M GPS TX
PA9  (USART1_TX)    ──► NEO-6M GPS RX  [for config cmds]
PA3  (USART2_RX)    ──► SIM800L TX
PA2  (USART2_TX)    ──► SIM800L RX
PB10 (USART3_TX)    ──► USB-Serial adapter (debug)
PA8  (GPIO OUT)     ──► Buzzer (+)
PB0  (GPIO IN)      ──► Reset button (other end to GND)
PB1  (GPIO OUT)     ──► Relay IN
PC13 (GPIO OUT)     ──► Status LED (onboard)
3.3V                ──► MPU-6050 VCC, NEO-6M VCC
5V                  ──► MQ-3 VCC (5V rail), SIM800L (4V via diode)
GND                 ──► All GND lines (common ground)
```

---

## Wiring Diagram (Text)

```
                  ┌─────────────────────────┐
                  │    STM32F103C8T6        │
    MQ-3 AO ─────►│PA0        PB6 SCL ─────►│──── MPU-6050
    GPS TX  ─────►│PA10       PB7 SDA ─────►│──── MPU-6050
    GSM TX  ─────►│PA3                      │
                  │PA2 ───────────────────► GSM RX
                  │PA8 ───────────────────► Buzzer
                  │PB0 ◄─────────── Button ─┘ (to GND)
                  │PB1 ───────────────────► Relay IN
                  └─────────────────────────┘

  Car Battery (12V) → LM7805 (5V) → AMS1117 (3.3V)
  Engine Start Wire → NC Relay Contact → Ignition Circuit
```

---

## File Structure

```
accident_detection/
├── config.h        ← All constants, pin defs, data structures
├── main.c          ← State machine + system orchestration
├── mpu6050.c/h     ← IMU driver (I2C, crash detection, angles)
├── gps.c/h         ← NMEA parser, GPS fix, coordinate formatting
├── gsm.c/h         ← AT commands, SMS sending, voice call
├── alcohol.c/h     ← MQ-3 ADC reading, BAC level classification
└── drivers.h       ← Combined header shortcut
```

---

## System State Machine

```
  Power On
      │
      ▼
  [INIT] ──── Init failure ──────────────────► [ERROR] (LED SOS loop)
      │
      ▼
  [WARMUP] ─── 30 seconds MQ-3 warmup
      │
      ▼
  [ALCOHOL_TEST] ─── Pass (sober) ──────────► [MONITORING] ─────┐
      │                                              │            │
      └── Fail x3 (drunk) ─► [LOCKOUT]        IMU 100Hz         │
                                (forever)      GPS every 5s      │
                                                    │            │
                                               Crash detected    │
                                                    │            │
                                                    ▼            │
                                           [ALERT_PENDING]       │
                                           7-second countdown    │
                                                    │            │
                                         Cancel button? ─► Yes──┘
                                                    │
                                                   No
                                                    │
                                                    ▼
                                            [EMERGENCY]
                                         Get GPS → Send SMS
                                         → Make call → SOS beeps
                                                    │
                                                    └──────────► Back to [MONITORING]
```

---

## Detection Thresholds (Tunable in config.h)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MQ3_VOLTAGE_THRESHOLD` | 0.8V | Alcohol detection voltage |
| `CRASH_GFORCE_THRESHOLD` | 4.0g | Moderate crash (with cancel window) |
| `CRASH_GFORCE_SEVERE` | 8.0g | Severe crash (immediate response) |
| `ROLLOVER_ANGLE_DEG` | 60° | Static roll angle for rollover |
| `ROLLOVER_GYRO_DPS` | 100°/s | Dynamic rollover angular velocity |
| `ALERT_DELAY_MS` | 7000ms | Cancel window duration |

---

## Setup Instructions

### 1. Configure Emergency Contacts
Edit `config.h`:
```c
#define EMERGENCY_NUMBER_1  "+91XXXXXXXXXX"  // Emergency services
#define EMERGENCY_NUMBER_2  "+91XXXXXXXXXX"  // Family contact
#define EMERGENCY_NUMBER_3  "+91XXXXXXXXXX"  // Friend/emergency contact
#define EMERGENCY_CALL_NUMBER "+91XXXXXXXXXX" // Number to auto-call
```

### 2. Calibrate MQ-3 Sensor (Important!)
The `MQ3_R0_KOHM` value in `config.h` (default: 76.63 kΩ) represents the sensor
resistance in clean air. To calibrate:
1. Let sensor warm up for 24 hours in clean air
2. Read the ADC value and compute: `R0 = RL * (Vcc/Vout - 1)`
3. Update `MQ3_R0_KOHM` with your measured value

### 3. Build for STM32 (STM32CubeIDE)
1. Create new STM32F103C8T6 project in STM32CubeIDE
2. Enable: I2C1, USART1, USART2, USART3, ADC1 in CubeMX
3. Add all .c files to project
4. `#define PLATFORM_STM32` is already default in config.h
5. Build and flash via ST-Link

### 4. Build for Arduino (fallback)
1. Change `config.h` to `#define PLATFORM_ARDUINO`
2. Install: Wire.h (built-in), ensure Serial1/Serial2 available (Mega)
3. Upload via Arduino IDE

---

## Sensor Upgrade Options

| Need | Upgrade | Reason |
|------|---------|--------|
| Better alcohol accuracy | MQ-303A | More sensitive to ethanol specifically |
| Higher G-range | ADXL377 (±200g) | For very high-speed crashes |
| Better GPS | u-blox M8N | Faster fix, better accuracy |
| Cellular + GPS combo | SIM7600G | 4G LTE + built-in GPS (replaces both modules) |
| IMU with temp comp | ICM-42688-P | Higher precision, better temperature stability |
| Data logging | Add SD card (SPI) | Store crash data for analysis |
| Display | SSD1306 OLED (I2C) | Compact status display |

---

## SMS Format Example

```
ACCIDENT ALERT! Vehicle AADR-001
COLLISION DETECTED
Location: maps.google.com/?q=12.971599,77.594566
Impact: 6.2g
Speed before: 87km/h
SEND HELP NOW!
```

---

## Safety Notes

1. **Engine Relay Wiring**: Wire relay in series with ignition circuit, NOT the starter
   motor direct. Use a normally-open (NO) contact rated for your vehicle's load.

2. **MQ-3 Power**: The MQ-3 heater draws ~900mA at 5V — use a dedicated 5V supply,
   not the MCU's 5V rail if it's underpowered.

3. **SIM800L Power**: SIM800L can draw up to 2A during transmission. Use a dedicated
   3.7-4.2V supply with a large filter capacitor (1000µF) at the module.

4. **False Positive Tuning**: If road bumps trigger alerts, raise `CRASH_GFORCE_THRESHOLD`
   slightly or increase `IMU_MOVING_AVG_SIZE` for more smoothing.

5. **Testing**: Always test the cancel button BEFORE deployment to verify it works.
   Never test by causing a real crash.



