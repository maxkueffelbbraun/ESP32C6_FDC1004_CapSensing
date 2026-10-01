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

static const char* TAG = "fdc1004";

constexpr i2c_port_t I2C_PORT = I2C_NUM_0;
constexpr gpio_num_t I2C_SDA_PIN = GPIO_NUM_1;
constexpr gpio_num_t I2C_SCL_PIN = GPIO_NUM_0;
constexpr uint32_t I2C_FREQUENCY_HZ = 100000;
constexpr uint8_t FDC1004_ADDR = 0x50;

constexpr char AP_SSID[] = "ESP32C6-FDC1004";
constexpr char AP_PASSWORD[] = "FDC1004demo";
constexpr uint8_t AP_CHANNEL = 1;
constexpr uint8_t AP_MAX_CONNECTIONS = 4;

constexpr gpio_num_t WS2812_GPIO = GPIO_NUM_8;
constexpr uint32_t LED_BLINK_INTERVAL_MS = 1000;
constexpr uint32_t WS2812_RESOLUTION_HZ = 10000000;

constexpr uint32_t SAMPLE_INTERVAL_MS = 100;
constexpr size_t MAX_SAMPLES = 4000;

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

struct Sample {
  uint32_t timestampMs;
  float ch1MinusCh3Pf;
  float ch2MinusCh4Pf;
};

Sample sampleBuffer[MAX_SAMPLES];
size_t sampleHead = 0;
size_t sampleCount = 0;

float latestCh1MinusCh3Pf = NAN;
float latestCh2MinusCh4Pf = NAN;
uint32_t latestTimestampMs = 0;

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

const char INDEX_HTML[] = R"HTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>FDC1004 Live Data</title>
  <style>
    :root {
      --bg-1: #f8fff6;
      --bg-2: #e9f6ff;
      --card: #ffffff;
      --text: #10203a;
      --muted: #50627a;
      --accent: #0f766e;
      --accent-2: #1d4ed8;
      --border: #d8e3f0;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      font-family: "Segoe UI", Tahoma, Geneva, Verdana, sans-serif;
      color: var(--text);
      background:
        radial-gradient(circle at 15% 15%, rgba(15, 118, 110, 0.16), transparent 40%),
        radial-gradient(circle at 85% 0%, rgba(29, 78, 216, 0.18), transparent 35%),
        linear-gradient(150deg, var(--bg-1), var(--bg-2));
      display: grid;
      place-items: center;
      padding: 16px;
    }
    .panel {
      width: min(920px, 100%);
      background: var(--card);
      border: 1px solid var(--border);
      border-radius: 16px;
      box-shadow: 0 14px 40px rgba(16, 32, 58, 0.12);
      padding: 20px;
      animation: fadeIn 500ms ease;
    }
    h1 {
      margin: 0 0 6px;
      font-size: clamp(1.2rem, 1.6vw + 1rem, 2rem);
      letter-spacing: 0.02em;
    }
    .sub {
      color: var(--muted);
      margin-bottom: 20px;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(230px, 1fr));
      gap: 12px;
      margin-bottom: 16px;
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
      color: var(--accent-2);
      min-height: 2.2rem;
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
      margin-top: 10px;
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
    @keyframes fadeIn {
      from { opacity: 0; transform: translateY(8px); }
      to { opacity: 1; transform: translateY(0); }
    }
  </style>
</head>
<body>
  <main class="panel">
    <h1>FDC1004 Capacitive Sensor</h1>
    <div class="sub">Live differential capacitance from ESP32-C6 access point</div>

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

    <div class="timestamp" id="ts">No sample yet</div>
    <div class="actions">
      <button id="download">Download CSV</button>
      <span id="status">Connecting...</span>
    </div>
  </main>

  <script>
    const ch13 = document.getElementById('ch13');
    const ch24 = document.getElementById('ch24');
    const ts = document.getElementById('ts');
    const statusEl = document.getElementById('status');
    const downloadBtn = document.getElementById('download');

    downloadBtn.addEventListener('click', () => {
      window.location.href = '/download.csv';
    });

    function fmtPf(value) {
      if (!Number.isFinite(value)) return '--';
      return value.toFixed(4) + ' pF';
    }

    async function refresh() {
      try {
        const response = await fetch('/data', { cache: 'no-store' });
        if (!response.ok) {
          throw new Error('HTTP ' + response.status);
        }
        const data = await response.json();

        ch13.textContent = fmtPf(data.ch1_ch3_pf);
        ch24.textContent = fmtPf(data.ch2_ch4_pf);
        ts.textContent = 'Sample timestamp: ' + data.timestamp_ms + ' ms';
        statusEl.textContent = 'Live';
      } catch (error) {
        statusEl.textContent = 'Connection issue';
      }
    }

    setInterval(refresh, 500);
    refresh();
  </script>
</body>
</html>
)HTML";

uint32_t millisNow() {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

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

bool configureMeasurement(uint8_t confReg, uint8_t cha, uint8_t chb) {
  // Differential measurement: CHA - CHB, CAPDAC not used.
  uint16_t config = (static_cast<uint16_t>(cha & 0x07) << 13) |
                    (static_cast<uint16_t>(chb & 0x07) << 10);
  return i2cWriteRegister16(confReg, config);
}

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

float rawToPf(int32_t raw24) {
  // Datasheet scale: 2^-19 pF per LSB for differential result.
  return static_cast<float>(raw24) / 524288.0f;
}

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

size_t oldestSampleIndex() {
  if (sampleCount < MAX_SAMPLES) {
    return 0;
  }
  return sampleHead;
}

esp_err_t handleIndex(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

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

  esp_err_t nvsErr = nvs_flash_init();
  if (nvsErr == ESP_ERR_NVS_NO_FREE_PAGES || nvsErr == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    nvsErr = nvs_flash_init();
  }
  ESP_ERROR_CHECK(nvsErr);

  initI2c();
  initStatusLed();

  if (!initFdc1004()) {
    ESP_LOGE(TAG, "FDC1004 init failed. Check wiring and power.");
  } else {
    ESP_LOGI(TAG, "FDC1004 init successful.");
  }

  setupAccessPoint();
  setupWebServer();
  ESP_LOGI(TAG, "Web server running.");

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
