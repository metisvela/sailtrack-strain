#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "HX711.h"

// Hardware Pin Configuration[cite: 2]
const int LOADCELL_DOUT_PIN = 18; //[cite: 2]
const int LOADCELL_SCK_PIN  = 5; //[cite: 2]
const float CALIBRATION_FACTOR = 14.54; //[cite: 2]

// Wi-Fi Access Point Details
const char* AP_SSID = "Strength-Challenge";
const char* AP_PASS = "12345678"; // At least 8 characters

HX711 scale;
WebServer server(80);

// Global live metrics
volatile float currentWeight = 0.0;
volatile float maxWeight = 0.0;
volatile float minWeight = 0.0;
volatile bool resetRequested = true;

// Web page HTML with high-contrast scoreboard design
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Strength Challenge</title>
  <style>
    * { box-sizing: border-box; }
    body {
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      background: #0f172a;
      color: #f8fafc;
      text-align: center;
      margin: 0;
      padding: 20px;
    }
    .container {
      max-width: 480px;
      margin: 0 auto;
    }
    h1 {
      font-size: 1.8rem;
      letter-spacing: 1px;
      text-transform: uppercase;
      margin-bottom: 20px;
      color: #38bdf8;
    }
    .card {
      background: #1e293b;
      border-radius: 16px;
      padding: 20px;
      margin-bottom: 16px;
      box-shadow: 0 8px 24px rgba(0,0,0,0.3);
    }
    .card-current {
      border: 2px solid #38bdf8;
    }
    .card-max {
      border: 2px solid #22c55e;
      background: #14532d22;
    }
    .card-min {
      border: 2px solid #f59e0b;
      background: #78350f22;
    }
    .label {
      font-size: 0.9rem;
      text-transform: uppercase;
      letter-spacing: 1.5px;
      color: #94a3b8;
      margin-bottom: 6px;
    }
    .val {
      font-size: 3.8rem;
      font-weight: 800;
      font-variant-numeric: tabular-nums;
      line-height: 1.1;
    }
    .val-current { color: #f8fafc; }
    .val-max { color: #4ade80; }
    .val-min { color: #fbbf24; }
    .unit {
      font-size: 1.2rem;
      color: #64748b;
      margin-left: 4px;
    }
    .grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 12px;
    }
    .grid .val {
      font-size: 2.2rem;
    }
    .btn {
      display: block;
      width: 100%;
      padding: 18px;
      font-size: 1.25rem;
      font-weight: 700;
      letter-spacing: 1px;
      text-transform: uppercase;
      border: none;
      border-radius: 12px;
      cursor: pointer;
      margin-top: 10px;
      transition: transform 0.1s, opacity 0.1s;
    }
    .btn:active {
      transform: scale(0.97);
      opacity: 0.85;
    }
    .btn-reset {
      background: #ef4444;
      color: #ffffff;
      box-shadow: 0 4px 16px rgba(239, 68, 68, 0.4);
    }
    .btn-tare {
      background: #334155;
      color: #cbd5e1;
      font-size: 0.95rem;
      padding: 12px;
      margin-top: 8px;
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>Strength Challenge</h1>

    <div class="card card-current">
      <div class="label">Current Load</div>
      <div class="val val-current"><span id="curr">0.0</span><span class="unit">g</span></div>
    </div>

    <div class="grid">
      <div class="card card-max">
        <div class="label">Record Max</div>
        <div class="val val-max"><span id="max">0.0</span><span class="unit">g</span></div>
      </div>
      <div class="card card-min">
        <div class="label">Record Min</div>
        <div class="val val-min"><span id="min">0.0</span><span class="unit">g</span></div>
      </div>
    </div>

    <button class="btn btn-reset" onclick="resetStats()">Reset Max & Min</button>
    <button class="btn btn-tare" onclick="tareScale()">Zero / Tare Scale</button>
  </div>

  <script>
    async function fetchMetrics() {
      try {
        const res = await fetch('/data');
        if (res.ok) {
          const d = await res.json();
          document.getElementById('curr').innerText = d.curr.toFixed(1);
          document.getElementById('max').innerText  = d.max.toFixed(1);
          document.getElementById('min').innerText  = d.min.toFixed(1);
        }
      } catch (e) {}
    }

    async function resetStats() {
      await fetch('/reset', { method: 'POST' });
      fetchMetrics();
    }

    async function tareScale() {
      await fetch('/tare', { method: 'POST' });
      fetchMetrics();
    }

    // Stream updates every 100ms (10 Hz)
    setInterval(fetchMetrics, 100);
  </script>
</body>
</html>
)rawliteral";

// Serve Main UI
void handleRoot() {
  server.send(200, "text/html", INDEX_HTML);
}

// Serve Live JSON Stream
void handleData() {
  char json[96];
  snprintf(json, sizeof(json), "{\"curr\":%.1f,\"max\":%.1f,\"min\":%.1f}", 
           currentWeight, maxWeight, minWeight);
  server.send(200, "application/json", json);
}

// Reset Max and Min to Current Value
void handleReset() {
  resetRequested = true;
  server.send(200, "text/plain", "OK");
}

// Tare the scale
void handleTare() {
  scale.tare(10);
  resetRequested = true;
  server.send(200, "text/plain", "OK");
}

void setup() {
  Serial.begin(115200);

  // 1. Initialize HX711[cite: 2]
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN); //[cite: 2]
  scale.set_scale(CALIBRATION_FACTOR); //[cite: 2]
  scale.tare(15); //[cite: 2]

  // 2. Start Standalone Wi-Fi Hotspot
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  // 3. Configure Web Endpoints
  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/reset", HTTP_POST, handleReset);
  server.on("/tare", HTTP_POST, handleTare);
  server.begin();
}

void loop() {
  server.handleClient();

  // Read HX711 as fast as the chip produces data
  if (scale.is_ready()) {
    float w = scale.get_units(1);
    currentWeight = w;

    if (resetRequested) {
      maxWeight = w;
      minWeight = w;
      resetRequested = false;
    } else {
      if (w > maxWeight) maxWeight = w;
      if (w < minWeight) minWeight = w;
    }
  }
}