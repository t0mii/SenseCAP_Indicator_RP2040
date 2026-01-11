# SenseCAP Indicator RP2040 - Extended Sensor Firmware

Arduino firmware for the RP2040 coprocessor on the SenseCAP Indicator D1S with extended Grove sensor support.

## Supported Sensors

### Built-in Sensors
- **SCD4x** (I2C 0x62): CO2, Temperature, Humidity
- **SGP40**: TVOC Index (with VOC algorithm)

### Extended Grove Sensors (I2C)
| Sensor | Address | Measurements |
|--------|---------|--------------|
| **Grove HM3301 (HM330X)** | 0x40 | PM1.0, PM2.5, PM10 (ug/m³) |
| **Grove Multichannel Gas V2** | 0x08 | NO2, C2H5OH, VOC, CO |
| **Grove AHT20** | 0x38 | External Temperature, Humidity |

## Wiring

Connect Grove sensors to the I2C Grove port on the SenseCAP Indicator. All sensors share the same I2C bus (SDA: GPIO20, SCL: GPIO21).

## Communication Protocol

Sensor data is sent to the ESP32 via UART using COBS (Consistent Overhead Byte Stuffing) encoding.

### Packet Types
| Type | Code | Data |
|------|------|------|
| CO2 | 0xB2 | float |
| Internal Temp | 0xB3 | float |
| Internal Humidity | 0xB4 | float |
| TVOC Index | 0xB5 | float |
| PM1.0 | 0xB6 | float |
| PM2.5 | 0xB7 | float |
| PM10 | 0xB8 | float |
| NO2 (GM102B) | 0xB9 | float |
| C2H5OH (GM302B) | 0xBA | float |
| VOC (GM502B) | 0xBB | float |
| CO (GM702B) | 0xBC | float |
| External Temp | 0xBD | float |
| External Humidity | 0xBE | float |

## Building

### Requirements
- Arduino IDE or arduino-cli
- Board: Raspberry Pi Pico (RP2040)
- Libraries:
  - Sensirion I2C SGP40
  - Sensirion I2C SCD4x
  - Sensirion Gas Index Algorithm
  - Seeed Grove HM330X
  - Seeed Multichannel Gas Sensor V2
  - AHT20
  - PacketSerial

### Flashing
1. Hold BOOTSEL button on RP2040
2. Connect USB
3. Copy `.uf2` file to the mounted drive

Or use Arduino IDE to upload directly.

## License

Based on original [SenseCAP Indicator RP2040](https://github.com/Seeed-Solution/SenseCAP_Indicator_RP2040) by Seeed Studio.
