# ESP32-C6 MQTT BMP180

Simple ESP-IDF project for ESP32-C6 that reads a BMP180 pressure sensor over I2C and publishes temperature and pressure via MQTT.

## Features

- ESP32-C6
- BMP180 over I2C (GPIO0 = SDA, GPIO1 = SCL)
- Proper conversion-ready polling (no fixed delay hack)
- Temperature (°C) and Pressure (hPa)
- MQTT publish
- Home Assistant auto-discovery

## Wiring

| BMP180 | ESP32-C6 |
|--------|----------|
| VCC    | 3.3V     |
| GND    | GND      |
| SDA    | GPIO0    |
| SCL    | GPIO1    |

I2C address: `0x77`

## Build

Requires ESP-IDF v5.x or newer.

```bash
idf.py set-target esp32c6
idf.py menuconfig
idf.py build
idf.py flash monitor
