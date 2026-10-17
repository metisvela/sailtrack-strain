#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <Wire.h>
#include "RTClib.h"
#include "HX711.h"

// --- Hardware Pin Definitions ---
// Cell 1
const int CELL1_DOUT_PIN = 18;
const int CELL1_SCK_PIN  = 5;

// Cell 2
const int CELL2_DOUT_PIN = 26;
const int CELL2_SCK_PIN  = 27;

// Cell 3 (GPIO 34 is input-only DOUT; GPIO 23 is output SCK)
const int CELL3_DOUT_PIN = 34;
const int CELL3_SCK_PIN  = 23;

// I2C Pins for DS3231 RTC
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// --- Calibration Factors (Adjust individually per amplifier/cell pair) ---
const float CAL_FACTOR_1 = 13.95; 
const float CAL_FACTOR_2 = 13.89; 
const float CAL_FACTOR_3 = 13.75; 

// --- Wi-Fi SoftAP Configuration ---
const char* AP_SSID = "LoadCell-Logger";
const char* AP_PASS = "12345678";

// --- Packed 18-byte Record (4B + 2B + 4B + 4B + 4B) ---
struct __attribute__((packed)) LogRecord {
  uint32_t epoch_s;
  uint16_t ms;
  float weight1_g;
  float weight2_g;
  float weight3_g;
};

const char* FILE_PATH = "/log.bin";

HX711 scale1;
HX711 scale2;
HX711 scale3;
RTC_DS3231 rtc;
WebServer server(80);

// FreeRTOS Mutex to prevent file read/write collisions between cores
SemaphoreHandle_t fileMutex;

// --- Web Server Handlers ---
void handleRoot() {
  xSemaphoreTake(fileMutex, portMAX_DELAY);
  File f = LittleFS.open(FILE_PATH, FILE_READ);
  size_t bytesUsed = f ? f.size() : 0;
  if (f) f.close();
  xSemaphoreGive(fileMutex);

  size_t recordCount = bytesUsed / sizeof(LogRecord);

  String html = R"rawliteral(
  <!DOCTYPE html>
  <html>
  <head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>3-Cell Load Logger</title>
    <style>
      body { font-family: sans-serif; text-align: center; margin: 40px auto; max-width: 420px; background: #f4f4f9; color: #333; }
      .card { background: white; padding: 25px; border-radius: 12px; box-shadow: 0 4px 12px rgba(0,0,0,0.1); }
      h2 { margin-top: 0; color: #111; }
      .stat { margin: 15px 0; font-size: 1.1em; }
      .btn { display: block; width: 100%; padding: 14px; margin: 12px 0; font-size: 16px; font-weight: bold; border-radius: 8px; border: none; cursor: pointer; text-decoration: none; box-sizing: border-box; }
      .btn-download { background: #007bff; color: white; }
      .btn-clear { background: #dc3545; color: white; }
    </style>
  </head>
  <body>
    <div class="card">
      <h2>3-Cell Data Logger</h2>
      <div class="stat">Records Logged: <strong>)rawliteral" + String(recordCount) + R"rawliteral(</strong></div>
      <div class="stat">Flash Used: <strong>)rawliteral" + String(bytesUsed / 1024.0, 1) + R"rawliteral( KB</strong></div>
      <a href="/download" class="btn btn-download">Download 3-Cell CSV</a>
      <a href="/clear" class="btn btn-clear" onclick="return confirm('Erase all logged data?');">Clear Memory</a>
    </div>
  </body>
  </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

void handleDownload() {
  xSemaphoreTake(fileMutex, portMAX_DELAY);
  File readFile = LittleFS.open(FILE_PATH, FILE_READ);

  if (!readFile || readFile.size() == 0) {
    if (readFile) readFile.close();
    xSemaphoreGive(fileMutex);
    server.send(200, "text/plain", "Log is empty.");
    return;
  }

  // Set HTTP headers for file download
  server.sendHeader("Content-Type", "text/csv");
  server.sendHeader("Content-Disposition", "attachment; filename=loadcell_3cell_log.csv");
  server.sendHeader("Connection", "close");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");

  // Stream CSV Header with columns for all 3 cells
  server.sendContent("DateTime,Epoch_Seconds,SubSecond_ms,Weight1_Grams,Weight2_Grams,Weight3_Grams\n");

  LogRecord rec;
  char lineBuf[140];

  // Stream data in small chunks without blocking ESP32 RAM
  while (readFile.read((uint8_t*)&rec, sizeof(rec)) == sizeof(rec)) {
    DateTime dt(rec.epoch_s);
    snprintf(lineBuf, sizeof(lineBuf),
             "%04d-%02d-%02d %02d:%02d:%02d.%03u,%lu,%.2f,%.2f,%.2f\n",
             dt.year(), dt.month(), dt.day(),
             dt.hour(), dt.minute(), dt.second(),
             rec.ms, (unsigned long)rec.epoch_s,
             rec.weight1_g, rec.weight2_g, rec.weight3_g);
    server.sendContent(lineBuf);
  }

  readFile.close();
  xSemaphoreGive(fileMutex);

  server.sendContent(""); // Terminate HTTP chunked transfer
}

void handleClear() {
  xSemaphoreTake(fileMutex, portMAX_DELAY);
  LittleFS.remove(FILE_PATH);
  xSemaphoreGive(fileMutex);

  server.sendHeader("Location", "/");
  server.send(303);
}

// --- Core 0 Dedicated Sensor Task (2 Hz Synchronized Logging) ---
void sensorLoggingTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(500); // 500 ms (2 Hz)

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    DateTime now = rtc.now();

    LogRecord record;
    record.epoch_s   = now.unixtime();
    record.ms        = (uint16_t)(millis() % 1000);
    record.weight1_g = scale1.is_ready() ? scale1.get_units(1) : 0.0;
    record.weight2_g = scale2.is_ready() ? scale2.get_units(1) : 0.0;
    record.weight3_g = scale3.is_ready() ? scale3.get_units(1) : 0.0;

    // Lock mutex and append record to LittleFS
    if (xSemaphoreTake(fileMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      File logFile = LittleFS.open(FILE_PATH, FILE_APPEND);
      if (logFile) {
        logFile.write((uint8_t*)&record, sizeof(record));
        logFile.flush();
        logFile.close();
      }
      xSemaphoreGive(fileMutex);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // 1. Create Mutex
  fileMutex = xSemaphoreCreateMutex();

  // 2. Initialize RTC
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!rtc.begin()) {
    Serial.println("[ERROR] RTC not found on I2C!");
  }

  // 3. Initialize LittleFS
  if (!LittleFS.begin(true)) {
    Serial.println("[ERROR] LittleFS mount failed!");
  }

  // 4. Initialize All Three Load Cell Amplifiers
  Serial.println("Initializing Load Cells...");
  
  scale1.begin(CELL1_DOUT_PIN, CELL1_SCK_PIN);
  scale1.set_scale(CAL_FACTOR_1);

  scale2.begin(CELL2_DOUT_PIN, CELL2_SCK_PIN);
  scale2.set_scale(CAL_FACTOR_2);

  scale3.begin(CELL3_DOUT_PIN, CELL3_SCK_PIN);
  scale3.set_scale(CAL_FACTOR_3);

  // Zero / Tare all three channels (keep unweighted at power-on)
  scale1.tare(15);
  scale2.tare(15);
  scale3.tare(15);
  Serial.println("[OK] All 3 cells tared.");

  // 5. Start Wi-Fi SoftAP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  // 6. Set up Web Server Routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/download", HTTP_GET, handleDownload);
  server.on("/clear", HTTP_GET, handleClear);
  server.begin();

  Serial.println("[OK] Wi-Fi Server Active at 192.168.4.1");

  // 7. Pin the 2 Hz Logging Engine to CORE 0
  xTaskCreatePinnedToCore(
    sensorLoggingTask,
    "SensorTask",
    4096,
    NULL,
    1,
    NULL,
    0  // Pin to Core 0
  );
}

void loop() {
  server.handleClient();
  delay(2); // Yield to prevent Core 1 watchdog triggers
}