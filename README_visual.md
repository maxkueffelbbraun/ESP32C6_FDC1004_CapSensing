# ESP32-C6 FDC1004 Visual Firmware (main_visual.cpp)

This document describes the visual variant of the firmware in src/main_visual.cpp.

The visual firmware is based on the original working project, with one major addition:
- A browser-based live chart of FDC1004 capacitance data

The original firmware remains available and unchanged in src/main.cpp.

## What This Variant Does

- Runs on ESP32-C6 using ESP-IDF in PlatformIO
- Configures and reads an FDC1004 over I2C
- Measures two differential channels:
  - CH1 - CH3
  - CH2 - CH4
- Converts raw FDC1004 values to pF
- Starts Wi-Fi in Access Point mode
- Hosts an HTTP dashboard page on the ESP32
- Draws a rolling, live chart in the browser:
  - X axis: moving time window (last 60 seconds)
  - Y axis: capacitance (pF)
  - Dynamic scaling from current min/max sample values
  - Clamped to absolute bounds: -15 pF to +15 pF
- Stores samples in a ring buffer for CSV export
- Blinks WS2812 LED (green) as heartbeat

## Files Involved

- src/main_visual.cpp: firmware source for visual variant
- src/main.cpp: original firmware source (kept as-is)
- src/CMakeLists.txt: selects which source file builds per environment
- platformio.ini: defines environments

## PlatformIO Environments

1. Original firmware
- Environment name: esp32-c6-devkitc-1
- Source selected: src/main.cpp

2. Visual firmware
- Environment name: esp32-c6-devkitc-1-visual
- Source selected: src/main_visual.cpp

Important: With ESP-IDF, source selection is controlled in src/CMakeLists.txt (not by src_filter/build_src_filter).

## Hardware and Pinout

- Board: ESP32-C6-DevKitC-1
- FDC1004 I2C SDA: GP1
- FDC1004 I2C SCL: GP0
- I2C clock: 100 kHz
- WS2812 data pin: GP8

## Boot and Runtime Sequence

1. app_main starts
2. NVS initializes (with recovery erase if needed)
3. I2C bus initializes
4. WS2812 RMT driver initializes and LED is turned off
5. FDC1004 is detected and configured
6. Wi-Fi AP starts
7. HTTP server starts with routes:
   - /
   - /data
   - /download.csv
8. Main loop runs continuously:
   - LED heartbeat update
   - Periodic sensor sampling every 100 ms

## Sensor Configuration Details

The firmware configures FDC1004 in repeating mode at 100 SPS with:
- Measurement 1: CH1 - CH3
- Measurement 2: CH2 - CH4

Data path:
- Read status bits from FDC config register
- When both channels are ready, read raw 24-bit signed values
- Convert to pF using scale factor 2^-19 pF/LSB
- Push result into ring buffer and update latest snapshot

## Web Dashboard and Chart Behavior

The page served from / contains:
- Current values for CH1-CH3 and CH2-CH4
- Rolling line chart rendered on HTML canvas
- Download CSV button

Polling:
- Browser fetches /data every 500 ms

X axis:
- Rolling time window of 60 seconds
- Labels shown as seconds ago (for example -60s ... -0s)

Y axis:
- Computed from min/max values currently in chart history
- Additional small padding for readability
- Hard-clamped between:
  - Minimum: -15 pF
  - Maximum: +15 pF

Robustness behavior:
- Non-finite values are ignored for plotting
- Duplicate or non-increasing timestamps are discarded
- If all values collapse to one point, a minimum span is enforced

## HTTP Endpoints

1. GET /
- Returns dashboard HTML

2. GET /data
- Returns latest sample as JSON

Example response:
{
  "timestamp_ms": 12345,
  "ch1_ch3_pf": 1.234567,
  "ch2_ch4_pf": 0.987654
}

3. GET /download.csv
- Streams complete ring-buffer history as CSV

CSV header:
- timestamp_ms,ch1_minus_ch3_pf,ch2_minus_ch4_pf

## Build, Flash, and Monitor

Build visual firmware:
- pio run -e esp32-c6-devkitc-1-visual

Flash visual firmware:
- pio run -e esp32-c6-devkitc-1-visual -t upload

Open serial monitor:
- pio device monitor -b 115200 --port COM10

## Connect and Open Dashboard

1. Connect laptop/phone to:
- SSID: ESP32C6-FDC1004
- Password: FDC1004demo

2. Open the URL printed in serial logs (usually):
- http://192.168.4.1

## Memory and Timing

- Sample interval: 100 ms
- Max samples buffered: 4000
- Approximate history represented in CSV at 100 ms intervals: about 400 seconds

## Known Warnings

Build may show a legacy I2C driver deprecation warning for driver/i2c.h under ESP-IDF 6.0.1.
- Current behavior: non-blocking
- Future task: migrate to driver/i2c_master.h for ESP-IDF 7.x readiness

## Troubleshooting

1. No data updates on page
- Check FDC1004 wiring and power
- Verify device ID and manufacturer ID logs in serial output
- Confirm browser is connected to ESP AP network

2. Page opens but chart is flat or empty
- Confirm /data returns valid numbers
- Verify sensor status bits indicate both measurements ready

3. Upload succeeds but no AP visible
- Check serial logs for Wi-Fi startup errors
- Reboot board and re-check antenna/board power

## Suggested Next Enhancements

1. Add chart channel toggle (show CH1-CH3 and CH2-CH4 independently)
2. Add selectable time windows (10s/30s/60s)
3. Add optional Y-axis lock/unlock around a user-defined range
4. Expose a lightweight endpoint for recent history to backfill chart on page load
