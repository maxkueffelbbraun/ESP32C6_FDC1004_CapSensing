# ESP32-C6 FDC1004 Capacitance Monitor

This project runs on an ESP32-C6 and measures differential capacitance with an FDC1004 sensor over I2C. The firmware exposes a small web dashboard over Wi-Fi, shows live values, and stores recent samples for CSV export.

## Features

- ESP32-C6 + ESP-IDF via PlatformIO
- FDC1004 differential capacitance measurements
- Two channels: CH1-CH3 and CH2-CH4
- Live values via Wi-Fi access point
- JSON endpoint for live readings
- CSV download of buffered samples
- WS2812 status LED on GPIO8

## Hardware

- MCU: ESP32-C6
- Sensor: FDC1004, I2C address 0x50
- SDA: GPIO1
- SCL: GPIO0
- I2C clock: 100 kHz
- Status LED: WS2812 on GPIO8

## Variants

This repo contains two firmware variants:

- [README.md](README.md): basic dashboard with live values and CSV export
- [README_visual.md](README_visual.md): dashboard with a rolling live chart

Both variants use the same sensor setup and sampling logic. The visual version adds browser-side rendering and a 60 s chart window.

## Wi-Fi setup

The device creates its own access point:

- SSID: `ESP32C6-FDC1004`
- Password: `FDC1004demo`

After boot, connect to the AP and open the device IP shown in the serial monitor.

## Endpoints

- `/` — dashboard page
- `/data` — live JSON values
- `/download.csv` — CSV export of the recent history

Example JSON:

```json
{
  "timestamp_ms": 12345,
  "ch1_ch3_pf": 1.234567,
  "ch2_ch4_pf": 0.987654
}
```

## Quick start

Build:

```bash
pio run -e esp32-c6-devkitc-1
```

Upload:

```bash
pio run -e esp32-c6-devkitc-1 -t upload
```

Monitor serial output:

```bash
pio device monitor -b 115200 --port COM10
```

## Notes

- Sampling interval: 100 ms
- History depth: 4000 samples
- Raw values are converted to pF using the FDC1004 datasheet scale factor
- The code is designed for direct AP access and quick sensor debugging in the field
