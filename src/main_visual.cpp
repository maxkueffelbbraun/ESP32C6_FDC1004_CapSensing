#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "driver/i2c.h"
#include "driver/rmt_tx.h"

namespace {

static const char* TAG = "fdc1004-visual";

// Hardware and bus configuration.
constexpr i2c_port_t I2C_PORT = I2C_NUM_0;
constexpr gpio_num_t I2C_SDA_PIN = GPIO_NUM_1;
constexpr gpio_num_t I2C_SCL_PIN = GPIO_NUM_0;
constexpr uint32_t I2C_FREQUENCY_HZ = 100000;
constexpr uint8_t FDC1004_ADDR = 0x50;

// Wi-Fi AP credentials and limits for direct phone/PC access.
constexpr char AP_SSID[] = "ESP32C6-FDC1004";
constexpr char AP_PASSWORD[] = "FDC1004demo";
constexpr uint8_t AP_CHANNEL = 1;
constexpr uint8_t AP_MAX_CONNECTIONS = 4;

// Single WS2812 status LED driven over RMT.
constexpr gpio_num_t WS2812_GPIO = GPIO_NUM_8;
constexpr uint32_t LED_BLINK_INTERVAL_MS = 1000;
constexpr uint32_t WS2812_RESOLUTION_HZ = 10000000;

// Sensor sampling and ring-buffer depth for CSV export.
constexpr uint32_t SAMPLE_INTERVAL_MS = 100;
constexpr size_t MAX_SAMPLES = 4000;

// FDC1004 register map used by this firmware.
constexpr uint8_t REG_MEAS1_MSB = 0x00;
constexpr uint8_t REG_MEAS1_LSB = 0x01;
constexpr uint8_t REG_MEAS2_MSB = 0x02;
constexpr uint8_t REG_MEAS2_LSB = 0x03;
constexpr uint8_t REG_CONF_MEAS1 = 0x08;
constexpr uint8_t REG_CONF_MEAS2 = 0x09;
constexpr uint8_t REG_FDC_CONF = 0x0C;
constexpr uint8_t REG_MANUFACTURER_ID = 0xFE;
constexpr uint8_t REG_DEVICE_ID = 0xFF;

constexpr uint16_t FDC_RATE_100SPS = (1u << 10);
constexpr uint16_t FDC_REPEAT = (1u << 8);
constexpr uint16_t FDC_MEAS1_ENABLE = (1u << 7);
constexpr uint16_t FDC_MEAS2_ENABLE = (1u << 6);
constexpr uint16_t FDC_MEAS1_DONE = (1u << 3);
constexpr uint16_t FDC_MEAS2_DONE = (1u << 2);

// One persisted sample used for both live UI and CSV history.
struct Sample {
  uint32_t timestampMs;
  float ch1MinusCh3Pf;
  float ch2MinusCh4Pf;
};

// Circular sample storage.
Sample sampleBuffer[MAX_SAMPLES];
size_t sampleHead = 0;
size_t sampleCount = 0;

// Latest values served by /data.
float latestCh1MinusCh3Pf = NAN;
float latestCh2MinusCh4Pf = NAN;
uint32_t latestTimestampMs = 0;

// Runtime handles and LED transmission state.
httpd_handle_t server = nullptr;
uint32_t lastSampleMs = 0;
uint32_t lastLedToggleMs = 0;
bool ledIsOn = false;
rmt_channel_handle_t ws2812Channel = nullptr;
rmt_encoder_handle_t ws2812Encoder = nullptr;
uint8_t ws2812Pixels[3] = {0, 0, 0};
rmt_symbol_word_t ws2812Zero = {};
rmt_symbol_word_t ws2812One = {};
rmt_symbol_word_t ws2812Reset = {};

// Embedded web page: live values + rolling chart rendered in browser canvas.
const char INDEX_HTML[] = R"HTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>FDC1004 Live Chart</title>
  <style>
    :root {
      --bg-1: #f3fbff;
      --bg-2: #fff8ed;
      --card: #ffffff;
      --text: #18202c;
      --muted: #5a6471;
      --accent: #0a8f6a;
      --accent-2: #d9480f;
      --line-1: #0c7c59;
      --line-2: #d9480f;
      --grid: #d8e0ea;
      --border: #d7dde7;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      font-family: "Trebuchet MS", "Lucida Sans Unicode", "Lucida Grande", sans-serif;
      color: var(--text);
      background:
        radial-gradient(circle at 8% 5%, rgba(10, 143, 106, 0.2), transparent 40%),
        radial-gradient(circle at 100% 0%, rgba(217, 72, 15, 0.17), transparent 38%),
        linear-gradient(155deg, var(--bg-1), var(--bg-2));
      display: grid;
      place-items: center;
      padding: 16px;
    }
    .panel {
      width: min(980px, 100%);
      background: var(--card);
      border: 1px solid var(--border);
      border-radius: 16px;
      box-shadow: 0 16px 44px rgba(24, 32, 44, 0.14);
      padding: 20px;
      animation: fadeIn 500ms ease;
    }
    h1 {
      margin: 0 0 6px;
      font-size: clamp(1.2rem, 1.6vw + 1rem, 2rem);
      letter-spacing: 0.015em;
    }
    .sub {
      color: var(--muted);
      margin-bottom: 16px;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(230px, 1fr));
      gap: 12px;
      margin-bottom: 12px;
    }
    .card {
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 14px;
      background: #fcfdff;
    }
    .label {
      color: var(--muted);
      font-size: 0.9rem;
      margin-bottom: 4px;
    }
    .value {
      font-size: clamp(1.3rem, 1.8vw + 0.8rem, 2rem);
      font-weight: 700;
      color: var(--line-1);
      min-height: 2.2rem;
    }
    #ch24 {
      color: var(--line-2);
    }
    .chart-wrap {
      border: 1px solid var(--border);
      border-radius: 12px;
      background: #fbfcfe;
      overflow: hidden;
    }
    canvas {
      width: 100%;
      height: clamp(240px, 38vh, 420px);
      display: block;
    }
    .timestamp {
      font-size: 0.95rem;
      color: var(--muted);
      margin-top: 8px;
    }
    .actions {
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      align-items: center;
      margin-top: 12px;
    }
    button {
      border: 0;
      border-radius: 10px;
      padding: 10px 14px;
      color: #fff;
      background: linear-gradient(120deg, var(--accent), var(--accent-2));
      font-weight: 600;
      cursor: pointer;
    }
    button:hover {
      filter: brightness(1.08);
    }
    #status {
      color: var(--muted);
      font-size: 0.9rem;
    }
    .legend {
      display: flex;
      gap: 14px;
      align-items: center;
      margin: 10px 0 2px;
      color: var(--muted);
      font-size: 0.88rem;
      flex-wrap: wrap;
    }
    .dot {
      width: 10px;
      height: 10px;
      border-radius: 999px;
      display: inline-block;
      margin-right: 6px;
    }
    .dot.ch13 { background: var(--line-1); }
    .dot.ch24 { background: var(--line-2); }
    @keyframes fadeIn {
      from { opacity: 0; transform: translateY(8px); }
      to { opacity: 1; transform: translateY(0); }
    }
  </style>
</head>
<body>
  <main class="panel">
    <h1>FDC1004 Live Capacitive Chart</h1>
    <div class="sub">Rolling time axis with dynamic capacitance scaling (clamped to -15 pF ... +15 pF)</div>

    <section class="grid">
      <article class="card">
        <div class="label">Channel 1 - Channel 3</div>
        <div class="value" id="ch13">--</div>
      </article>
      <article class="card">
        <div class="label">Channel 2 - Channel 4</div>
        <div class="value" id="ch24">--</div>
      </article>
    </section>

    <section class="chart-wrap">
      <canvas id="chart"></canvas>
    </section>

    <div class="legend">
      <span><span class="dot ch13"></span>CH1 - CH3</span>
      <span><span class="dot ch24"></span>CH2 - CH4</span>
    </div>

    <div class="timestamp" id="ts">No sample yet</div>
    <div class="actions">
      <button id="download">Download CSV</button>
      <span id="status">Connecting...</span>
    </div>
  </main>

  <script>
    // UI bindings and rendering context.
    const ch13 = document.getElementById('ch13');
    const ch24 = document.getElementById('ch24');
    const ts = document.getElementById('ts');
    const statusEl = document.getElementById('status');
    const downloadBtn = document.getElementById('download');
    const chart = document.getElementById('chart');
    const ctx = chart.getContext('2d');

    // Chart limits and rolling window.
    const HISTORY_WINDOW_MS = 60000;
    const ABS_MIN_PF = -15;
    const ABS_MAX_PF = 15;
    const history = [];
    let lastTimestampMs = 0;

    downloadBtn.addEventListener('click', () => {
      window.location.href = '/download.csv';
    });

    function fmtPf(value) {
      if (!Number.isFinite(value)) return '--';
      return value.toFixed(4) + ' pF';
    }

    // Keep only forward-moving timestamps and trim to rolling time window.
    function pushPoint(data) {
      if (!Number.isFinite(data.timestamp_ms) || data.timestamp_ms <= lastTimestampMs) {
        return;
      }
      lastTimestampMs = data.timestamp_ms;
      history.push({
        t: data.timestamp_ms,
        ch13: Number(data.ch1_ch3_pf),
        ch24: Number(data.ch2_ch4_pf),
      });

      const oldest = data.timestamp_ms - HISTORY_WINDOW_MS;
      while (history.length > 0 && history[0].t < oldest) {
        history.shift();
      }
    }

    function getYRange() {
      let minV = Infinity;
      let maxV = -Infinity;

      for (const p of history) {
        if (Number.isFinite(p.ch13)) {
          minV = Math.min(minV, p.ch13);
          maxV = Math.max(maxV, p.ch13);
        }
        if (Number.isFinite(p.ch24)) {
          minV = Math.min(minV, p.ch24);
          maxV = Math.max(maxV, p.ch24);
        }
      }

      if (!Number.isFinite(minV) || !Number.isFinite(maxV)) {
        return { yMin: ABS_MIN_PF, yMax: ABS_MAX_PF };
      }

      // Dynamic range follows current data but never leaves absolute sensor limits.
      let yMin = Math.max(ABS_MIN_PF, minV);
      let yMax = Math.min(ABS_MAX_PF, maxV);

      if (yMax <= yMin) {
        const center = Math.max(ABS_MIN_PF, Math.min(ABS_MAX_PF, yMin));
        yMin = Math.max(ABS_MIN_PF, center - 0.5);
        yMax = Math.min(ABS_MAX_PF, center + 0.5);
      }

      const span = yMax - yMin;
      const pad = Math.max(0.15, span * 0.12);
      yMin = Math.max(ABS_MIN_PF, yMin - pad);
      yMax = Math.min(ABS_MAX_PF, yMax + pad);

      if (yMax - yMin < 0.3) {
        const c = (yMax + yMin) * 0.5;
        yMin = Math.max(ABS_MIN_PF, c - 0.15);
        yMax = Math.min(ABS_MAX_PF, c + 0.15);
      }

      return { yMin, yMax };
    }

    // Draw axes, grid, labels, and both channel traces.
    function drawChart() {
      const dpr = window.devicePixelRatio || 1;
      const width = Math.max(320, Math.floor(chart.clientWidth));
      const height = Math.max(220, Math.floor(chart.clientHeight));
      if (chart.width !== Math.floor(width * dpr) || chart.height !== Math.floor(height * dpr)) {
        chart.width = Math.floor(width * dpr);
        chart.height = Math.floor(height * dpr);
      }

      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, width, height);

      const pad = { left: 58, right: 16, top: 16, bottom: 34 };
      const plotW = width - pad.left - pad.right;
      const plotH = height - pad.top - pad.bottom;
      if (plotW <= 10 || plotH <= 10) {
        return;
      }

      const now = history.length > 0 ? history[history.length - 1].t : Date.now();
      const xMin = now - HISTORY_WINDOW_MS;
      const xMax = now;
      const { yMin, yMax } = getYRange();

      const xToPx = (t) => pad.left + ((t - xMin) / Math.max(1, (xMax - xMin))) * plotW;
      const yToPx = (v) => pad.top + (1 - (v - yMin) / Math.max(0.001, (yMax - yMin))) * plotH;

      ctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue('--grid').trim();
      ctx.lineWidth = 1;
      ctx.font = '12px Trebuchet MS, sans-serif';
      ctx.fillStyle = '#6b7280';

      const yTicks = 5;
      for (let i = 0; i <= yTicks; ++i) {
        const ratio = i / yTicks;
        const yVal = yMax - ratio * (yMax - yMin);
        const y = pad.top + ratio * plotH;

        ctx.beginPath();
        ctx.moveTo(pad.left, y);
        ctx.lineTo(width - pad.right, y);
        ctx.stroke();

        ctx.fillText(yVal.toFixed(2), 8, y + 4);
      }

      const xTicks = 6;
      for (let i = 0; i <= xTicks; ++i) {
        const ratio = i / xTicks;
        const x = pad.left + ratio * plotW;
        const msAgo = Math.round((1 - ratio) * HISTORY_WINDOW_MS / 1000);

        ctx.beginPath();
        ctx.moveTo(x, pad.top);
        ctx.lineTo(x, height - pad.bottom);
        ctx.stroke();

        ctx.fillText('-' + msAgo + 's', x - 12, height - 12);
      }

      ctx.strokeStyle = '#111827';
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.moveTo(pad.left, pad.top);
      ctx.lineTo(pad.left, height - pad.bottom);
      ctx.lineTo(width - pad.right, height - pad.bottom);
      ctx.stroke();

      function drawSeries(selector, accessor) {
        ctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue(selector).trim();
        ctx.lineWidth = 2;
        ctx.beginPath();
        let hasPoint = false;
        for (const p of history) {
          const v = accessor(p);
          if (!Number.isFinite(v)) {
            continue;
          }
          const x = xToPx(p.t);
          const y = yToPx(v);
          if (!hasPoint) {
            ctx.moveTo(x, y);
            hasPoint = true;
          } else {
            ctx.lineTo(x, y);
          }
        }
        if (hasPoint) {
          ctx.stroke();
        }
      }

      drawSeries('--line-1', (p) => p.ch13);
      drawSeries('--line-2', (p) => p.ch24);

      ctx.fillStyle = '#4b5563';
      ctx.fillText('time (rolling 60 s)', width * 0.5 - 42, height - 4);
      ctx.save();
      ctx.translate(12, height * 0.5 + 36);
      ctx.rotate(-Math.PI / 2);
      ctx.fillText('capacitance (pF)', 0, 0);
      ctx.restore();
    }

    async function refresh() {
      try {
        // Fetch latest sensor sample from firmware endpoint.
        const response = await fetch('/data', { cache: 'no-store' });
        if (!response.ok) {
          throw new Error('HTTP ' + response.status);
        }
        const data = await response.json();

        ch13.textContent = fmtPf(data.ch1_ch3_pf);
        ch24.textContent = fmtPf(data.ch2_ch4_pf);
        ts.textContent = 'Sample timestamp: ' + data.timestamp_ms + ' ms';
        pushPoint(data);
        drawChart();
        statusEl.textContent = 'Live';
      } catch (error) {
        statusEl.textContent = 'Connection issue';
      }
    }

    window.addEventListener('resize', drawChart);

    setInterval(refresh, 500);
    refresh();
  </script>
</body>
</html>
)HTML";

// Monotonic uptime helper used for sampling timestamps.
uint32_t millisNow() {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

// Convert RGB bytes into WS2812 timing symbols as RMT asks for more symbols.
size_t ws2812EncoderCallback(const void* data,
                             size_t dataSize,
                             size_t symbolsWritten,
                             size_t symbolsFree,
                             rmt_symbol_word_t* symbols,
                             bool* done,
                             void* arg) {
  (void)arg;

  if (symbolsFree < 8) {
    return 0;
  }

  const size_t dataPos = symbolsWritten / 8;
  const uint8_t* dataBytes = static_cast<const uint8_t*>(data);
  if (dataPos < dataSize) {
    size_t symbolPos = 0;
    for (int bitmask = 0x80; bitmask != 0; bitmask >>= 1) {
      symbols[symbolPos++] = (dataBytes[dataPos] & bitmask) ? ws2812One : ws2812Zero;
    }
    return symbolPos;
  }

  symbols[0] = ws2812Reset;
  *done = true;
  return 1;
}

bool i2cWriteRegister16(uint8_t reg, uint16_t value) {
  uint8_t payload[3] = {
      reg,
      static_cast<uint8_t>((value >> 8) & 0xFF),
      static_cast<uint8_t>(value & 0xFF),
  };
  const esp_err_t err = i2c_master_write_to_device(
      I2C_PORT,
      FDC1004_ADDR,
      payload,
      sizeof(payload),
      pdMS_TO_TICKS(100));
  return err == ESP_OK;
}

bool i2cReadRegister16(uint8_t reg, uint16_t& outValue) {
  uint8_t rx[2] = {0, 0};
  const esp_err_t err = i2c_master_write_read_device(
      I2C_PORT,
      FDC1004_ADDR,
      &reg,
      1,
      rx,
      sizeof(rx),
      pdMS_TO_TICKS(100));
  if (err != ESP_OK) {
    return false;
  }

  outValue = static_cast<uint16_t>(rx[0]) << 8;
  outValue |= static_cast<uint16_t>(rx[1]);
  return true;
}

// Configure one differential conversion pair (CHA - CHB).
bool configureMeasurement(uint8_t confReg, uint8_t cha, uint8_t chb) {
  // Differential measurement: CHA - CHB, CAPDAC not used.
  uint16_t config = (static_cast<uint16_t>(cha & 0x07) << 13) |
                    (static_cast<uint16_t>(chb & 0x07) << 10);
  return i2cWriteRegister16(confReg, config);
}

// Verify sensor identity, configure channels, and enable repeating conversions.
bool initFdc1004() {
  uint16_t manufacturerId = 0;
  uint16_t deviceId = 0;

  if (i2cReadRegister16(REG_MANUFACTURER_ID, manufacturerId)) {
    ESP_LOGI(TAG, "FDC1004 manufacturer ID: 0x%04X", manufacturerId);
  } else {
    ESP_LOGE(TAG, "Could not read manufacturer ID (I2C communication issue).");
    return false;
  }

  if (i2cReadRegister16(REG_DEVICE_ID, deviceId)) {
    ESP_LOGI(TAG, "FDC1004 device ID: 0x%04X", deviceId);
  } else {
    ESP_LOGE(TAG, "Could not read device ID (I2C communication issue).");
    return false;
  }

  // Measurement 1: Channel 1 - Channel 3
  if (!configureMeasurement(REG_CONF_MEAS1, 0, 2)) {
    return false;
  }

  // Measurement 2: Channel 2 - Channel 4
  if (!configureMeasurement(REG_CONF_MEAS2, 1, 3)) {
    return false;
  }

  uint16_t conf = FDC_RATE_100SPS | FDC_REPEAT | FDC_MEAS1_ENABLE | FDC_MEAS2_ENABLE;
  return i2cWriteRegister16(REG_FDC_CONF, conf);
}

// Read one signed 24-bit measurement from register pair.
bool readMeasurementRaw24(uint8_t msbReg, uint8_t lsbReg, int32_t& outRaw) {
  uint16_t msb = 0;
  uint16_t lsb = 0;

  if (!i2cReadRegister16(msbReg, msb) || !i2cReadRegister16(lsbReg, lsb)) {
    return false;
  }

  // 24-bit signed value: MSB register + upper byte of LSB register.
  int32_t raw = (static_cast<int32_t>(msb) << 8) | ((lsb >> 8) & 0xFF);
  if (raw & 0x800000) {
    raw |= 0xFF000000;
  }

  outRaw = raw;
  return true;
}

// Convert raw code to picofarads using FDC1004 scale.
float rawToPf(int32_t raw24) {
  // Datasheet scale: 2^-19 pF per LSB for differential result.
  return static_cast<float>(raw24) / 524288.0f;
}

// Insert sample into circular history and update latest snapshot.
void pushSample(uint32_t timestampMs, float ch13Pf, float ch24Pf) {
  sampleBuffer[sampleHead] = {timestampMs, ch13Pf, ch24Pf};
  sampleHead = (sampleHead + 1) % MAX_SAMPLES;
  if (sampleCount < MAX_SAMPLES) {
    ++sampleCount;
  }

  latestTimestampMs = timestampMs;
  latestCh1MinusCh3Pf = ch13Pf;
  latestCh2MinusCh4Pf = ch24Pf;
}

// Return logical start index of the current ring-buffer content.
size_t oldestSampleIndex() {
  if (sampleCount < MAX_SAMPLES) {
    return 0;
  }
  return sampleHead;
}

// Serve dashboard HTML.
esp_err_t handleIndex(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

// Serve latest sample for live web refresh.
esp_err_t handleData(httpd_req_t* req) {
  char json[160];
  snprintf(json,
           sizeof(json),
           "{\"timestamp_ms\":%lu,\"ch1_ch3_pf\":%.6f,\"ch2_ch4_pf\":%.6f}",
           static_cast<unsigned long>(latestTimestampMs),
           static_cast<double>(latestCh1MinusCh3Pf),
           static_cast<double>(latestCh2MinusCh4Pf));

  httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

// Stream all buffered samples as downloadable CSV.
esp_err_t handleCsvDownload(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/csv");
  httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=fdc1004_samples.csv");
  esp_err_t err = httpd_resp_send_chunk(req, "timestamp_ms,ch1_minus_ch3_pf,ch2_minus_ch4_pf\n", HTTPD_RESP_USE_STRLEN);
  if (err != ESP_OK) {
    return err;
  }

  const size_t count = sampleCount;
  const size_t start = oldestSampleIndex();

  char line[96];
  for (size_t i = 0; i < count; ++i) {
    size_t idx = (start + i) % MAX_SAMPLES;
    const Sample& s = sampleBuffer[idx];
    snprintf(line,
             sizeof(line),
             "%lu,%.6f,%.6f\n",
             static_cast<unsigned long>(s.timestampMs),
             static_cast<double>(s.ch1MinusCh3Pf),
             static_cast<double>(s.ch2MinusCh4Pf));
    err = httpd_resp_send_chunk(req, line, HTTPD_RESP_USE_STRLEN);
    if (err != ESP_OK) {
      return err;
    }
  }

  return httpd_resp_send_chunk(req, nullptr, 0);
}

// Start web server and register all routes.
void setupWebServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  if (httpd_start(&server, &config) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start HTTP server");
    return;
  }

  httpd_uri_t uriIndex = {
      .uri = "/",
      .method = HTTP_GET,
      .handler = handleIndex,
      .user_ctx = nullptr,
  };

  httpd_uri_t uriData = {
      .uri = "/data",
      .method = HTTP_GET,
      .handler = handleData,
      .user_ctx = nullptr,
  };

  httpd_uri_t uriCsv = {
      .uri = "/download.csv",
      .method = HTTP_GET,
      .handler = handleCsvDownload,
      .user_ctx = nullptr,
  };

  httpd_register_uri_handler(server, &uriIndex);
  httpd_register_uri_handler(server, &uriData);
  httpd_register_uri_handler(server, &uriCsv);
}

// Start ESP32-C6 in access-point mode so clients can open the dashboard directly.
void setupAccessPoint() {
  esp_netif_init();
  esp_event_loop_create_default();
  esp_netif_create_default_wifi_ap();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  wifi_config_t apConfig = {};
  memcpy(apConfig.ap.ssid, AP_SSID, strlen(AP_SSID));
  memcpy(apConfig.ap.password, AP_PASSWORD, strlen(AP_PASSWORD));
  apConfig.ap.ssid_len = strlen(AP_SSID);
  apConfig.ap.channel = AP_CHANNEL;
  apConfig.ap.max_connection = AP_MAX_CONNECTIONS;
  apConfig.ap.authmode = WIFI_AUTH_WPA2_PSK;
  if (strlen(AP_PASSWORD) == 0) {
    apConfig.ap.authmode = WIFI_AUTH_OPEN;
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &apConfig));
  ESP_ERROR_CHECK(esp_wifi_start());

  esp_netif_ip_info_t ipInfo;
  esp_netif_t* apNetif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
  if (apNetif != nullptr && esp_netif_get_ip_info(apNetif, &ipInfo) == ESP_OK) {
    ESP_LOGI(TAG,
             "Access point started. SSID=%s Password=%s URL=http://%d.%d.%d.%d",
             AP_SSID,
             AP_PASSWORD,
             IP2STR(&ipInfo.ip));
  } else {
    ESP_LOGI(TAG, "Access point started. SSID=%s Password=%s", AP_SSID, AP_PASSWORD);
  }
}

// Poll conversion-ready bits, read both channels, convert, and store sample.
void sampleSensorIfReady() {
  uint16_t conf = 0;
  if (!i2cReadRegister16(REG_FDC_CONF, conf)) {
    ESP_LOGE(TAG, "Failed to read FDC conf register.");
    return;
  }

  const bool meas1Ready = (conf & FDC_MEAS1_DONE) != 0;
  const bool meas2Ready = (conf & FDC_MEAS2_DONE) != 0;
  if (!meas1Ready || !meas2Ready) {
    return;
  }

  int32_t raw13 = 0;
  int32_t raw24 = 0;
  if (!readMeasurementRaw24(REG_MEAS1_MSB, REG_MEAS1_LSB, raw13) ||
      !readMeasurementRaw24(REG_MEAS2_MSB, REG_MEAS2_LSB, raw24)) {
    ESP_LOGE(TAG, "Failed to read measurement data.");
    return;
  }

  const float ch13Pf = rawToPf(raw13);
  const float ch24Pf = rawToPf(raw24);

  const uint32_t nowMs = millisNow();
  pushSample(nowMs, ch13Pf, ch24Pf);

  ESP_LOGI(TAG,
           "t=%lu ms, CH1-CH3=%.6f pF, CH2-CH4=%.6f pF",
           static_cast<unsigned long>(nowMs),
           static_cast<double>(ch13Pf),
           static_cast<double>(ch24Pf));
}

// Configure legacy ESP-IDF I2C master driver for the FDC1004 bus.
void initI2c() {
  i2c_config_t cfg = {};
  cfg.mode = I2C_MODE_MASTER;
  cfg.sda_io_num = I2C_SDA_PIN;
  cfg.scl_io_num = I2C_SCL_PIN;
  cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
  cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
  cfg.master.clk_speed = I2C_FREQUENCY_HZ;

  ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &cfg));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, cfg.mode, 0, 0, 0));
  ESP_LOGI(TAG, "I2C initialized at 100 kHz on SDA=GP1, SCL=GP0.");
}

// Configure RMT timings and initialize LED to off.
void initStatusLed() {
  ws2812Zero.level0 = 1;
  ws2812Zero.duration0 = static_cast<uint16_t>(0.3 * WS2812_RESOLUTION_HZ / 1000000);
  ws2812Zero.level1 = 0;
  ws2812Zero.duration1 = static_cast<uint16_t>(0.9 * WS2812_RESOLUTION_HZ / 1000000);

  ws2812One.level0 = 1;
  ws2812One.duration0 = static_cast<uint16_t>(0.9 * WS2812_RESOLUTION_HZ / 1000000);
  ws2812One.level1 = 0;
  ws2812One.duration1 = static_cast<uint16_t>(0.3 * WS2812_RESOLUTION_HZ / 1000000);

  const uint16_t resetTicks = static_cast<uint16_t>(WS2812_RESOLUTION_HZ / 1000000 * 50 / 2);
  ws2812Reset.level0 = 0;
  ws2812Reset.duration0 = resetTicks;
  ws2812Reset.level1 = 0;
  ws2812Reset.duration1 = resetTicks;

  rmt_tx_channel_config_t txConfig = {};
  txConfig.gpio_num = WS2812_GPIO;
  txConfig.clk_src = RMT_CLK_SRC_DEFAULT;
  txConfig.resolution_hz = WS2812_RESOLUTION_HZ;
  txConfig.mem_block_symbols = 64;
  txConfig.trans_queue_depth = 4;
  ESP_ERROR_CHECK(rmt_new_tx_channel(&txConfig, &ws2812Channel));

  rmt_simple_encoder_config_t simpleEncoderConfig = {};
  simpleEncoderConfig.callback = ws2812EncoderCallback;
  ESP_ERROR_CHECK(rmt_new_simple_encoder(&simpleEncoderConfig, &ws2812Encoder));
  ESP_ERROR_CHECK(rmt_enable(ws2812Channel));

  rmt_transmit_config_t txRuntimeConfig = {};
  txRuntimeConfig.loop_count = 0;
  ws2812Pixels[0] = 0;
  ws2812Pixels[1] = 0;
  ws2812Pixels[2] = 0;
  ESP_ERROR_CHECK(rmt_transmit(ws2812Channel, ws2812Encoder, ws2812Pixels, sizeof(ws2812Pixels), &txRuntimeConfig));
  ESP_ERROR_CHECK(rmt_tx_wait_all_done(ws2812Channel, pdMS_TO_TICKS(100)));

  ESP_LOGI(TAG, "WS2812 status LED initialized on GP8.");
}

// Blink status LED in main loop to indicate firmware is alive.
void updateStatusLed(uint32_t nowMs) {
  if (nowMs - lastLedToggleMs < LED_BLINK_INTERVAL_MS) {
    return;
  }

  lastLedToggleMs = nowMs;
  ledIsOn = !ledIsOn;

  // This LED uses RGB byte order on this board, so green is the second byte.
  ws2812Pixels[0] = 0;
  ws2812Pixels[1] = ledIsOn ? 10 : 0;
  ws2812Pixels[2] = 0;

  rmt_transmit_config_t txRuntimeConfig = {};
  txRuntimeConfig.loop_count = 0;
  ESP_ERROR_CHECK(rmt_transmit(ws2812Channel, ws2812Encoder, ws2812Pixels, sizeof(ws2812Pixels), &txRuntimeConfig));
  ESP_ERROR_CHECK(rmt_tx_wait_all_done(ws2812Channel, pdMS_TO_TICKS(100)));
}

}  // namespace

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "ESP32-C6 FDC1004 evaluation firmware booting...");

  // Initialize persistent storage required by Wi-Fi stack.
  esp_err_t nvsErr = nvs_flash_init();
  if (nvsErr == ESP_ERR_NVS_NO_FREE_PAGES || nvsErr == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    nvsErr = nvs_flash_init();
  }
  ESP_ERROR_CHECK(nvsErr);

  // Bring up hardware interfaces.
  initI2c();
  initStatusLed();

  // Configure FDC1004 measurement engine.
  if (!initFdc1004()) {
    ESP_LOGE(TAG, "FDC1004 init failed. Check wiring and power.");
  } else {
    ESP_LOGI(TAG, "FDC1004 init successful.");
  }

  setupAccessPoint();
  setupWebServer();
  ESP_LOGI(TAG, "Web server running.");

  // Cooperative main loop: heartbeat LED + periodic sensor polling.
  while (true) {
    const uint32_t nowMs = millisNow();
    updateStatusLed(nowMs);
    if (nowMs - lastSampleMs >= SAMPLE_INTERVAL_MS) {
      lastSampleMs = nowMs;
      sampleSensorIfReady();
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
