#include "esp32-hal-gpio.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <Adafruit_BMP085.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>

// ── Configuration ──────────────────────────────
#define WIFI_SSID       "pitest"
#define WIFI_PASS       "12345678"
#define NTP_SERVER      "pool.ntp.org"
#define GMT_OFFSET_SEC  19800          // IST (UTC+5:30)
#define DST_OFFSET_SEC  0

// Open-Meteo — no API key needed, just set your coordinates
#define WX_CITY         "Roorkee"
#define WX_LAT          "29.8667"
#define WX_LON          "77.8833"
#define WX_TZ           "Asia/Kolkata"
#define WX_UPDATE_MS    600000         // refresh every 10 min

#define DHTPIN          4
#define DHTTYPE         DHT22
#define SDA_PIN         21
#define SCL_PIN         22
#define SCREEN_W        128
#define SCREEN_H        64
#define OLED_RESET      -1
#define OLED_ADDR       0x3C

#define NUM_SCREENS     7
#define SCREEN_MS       5000
#define READ_MS         2000
#define SPARK_CAP       60
#define SPARK_MS        10000
#define MAX_FORECASTS   3

// ── Objects ────────────────────────────────────
DHT              dht(DHTPIN, DHTTYPE);
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, OLED_RESET);
Adafruit_BMP085  bmp;

// ── State ──────────────────────────────────────
bool  bmpFound  = false;
bool  timeReady = false;
int   curScr    = 0;
int   prevScr   = -1;

unsigned long lastSwitch = 0, lastRead  = 0, startTime = 0;
unsigned long lastSpark  = 0, lastWxMs  = 0;

bool  sliding    = false;
int   slideOff   = 0;
const int SLIDE_STEP = 32;

// Indoor sensors
float temp = 0, hum = 0, pressure = 0;
float heatIdx = 0, dewPt = 0, alt = 0;

// Sparkline
float sparkBuf[SPARK_CAP];
int   sparkHead = 0, sparkN = 0;

// ── Weather data (Open-Meteo) ──────────────────
struct WeatherNow {
  float temp, feelsLike, humidity, windSpeed;
  float dayMin, dayMax;
  int   wmoCode;
  char  sunrise[6];   // "HH:MM"
  char  sunset[6];    // "HH:MM"
  bool  valid;
} wx = { 0,0,0,0, 999,-999, 0, "", "", false };

struct FcSlot {
  int   hour;         // 0-23
  float temp;
  int   wmoCode;
} fc[MAX_FORECASTS];

int  fcCount    = 0;
bool wxFetching = false;

// ════════════════════════════════════════════════
//  WMO WEATHER CODE HELPERS
// ════════════════════════════════════════════════

const char* wmoDesc(int c) {
  if (c == 0)              return "clear sky";
  if (c == 1)              return "mainly clear";
  if (c == 2)              return "partly cloudy";
  if (c == 3)              return "overcast";
  if (c <= 48)             return "foggy";
  if (c <= 55)             return "drizzle";
  if (c <= 65)             return c <= 61 ? "light rain" : "heavy rain";
  if (c <= 77)             return "snowing";
  if (c <= 82)             return "rain showers";
  if (c <= 86)             return "snow showers";
  return "thunderstorm";
}

// Map WMO code to one of our icon types: 0=sun 1=partcloud 2=cloud 3=drop 4=snow 5=thunder 6=mist
int wmoIcon(int c) {
  if (c <= 1)  return 0;  // clear
  if (c == 2)  return 1;  // partly cloudy
  if (c == 3)  return 2;  // overcast
  if (c <= 48) return 6;  // fog/mist
  if (c <= 67) return 3;  // drizzle/rain
  if (c <= 77) return 4;  // snow
  if (c <= 82) return 3;  // rain showers
  if (c <= 86) return 4;  // snow showers
  return 5;               // thunder
}

// ════════════════════════════════════════════════
//  UTILITY
// ════════════════════════════════════════════════

float computeDewPoint(float t, float h) {
  const float a = 17.271, b = 237.7;
  float g = (a * t / (b + t)) + log(h / 100.0);
  return (b * g) / (a - g);
}

const char* comfortLabel(float t, float h) {
  if (t > 35 && h > 60) return "Hot & Humid";
  if (t > 35)            return "Too Hot";
  if (t < 10)            return "Too Cold";
  if (h > 70)            return "Too Humid";
  if (h < 30)            return "Too Dry";
  return "Comfortable";
}

// Parse "HH:MM" from Open-Meteo ISO string "2026-04-23T05:42"
void parseHHMM(const char* iso, char* out) {
  if (strlen(iso) >= 16) {
    out[0] = iso[11]; out[1] = iso[12];
    out[2] = ':';
    out[3] = iso[14]; out[4] = iso[15];
    out[5] = '\0';
  } else {
    strlcpy(out, "--:--", 6);
  }
}

// Format current-time unix timestamp → "HH:MM"
void fmtTime(long epoch, char* buf) {
  time_t t = (time_t)epoch;
  struct tm ti;
  localtime_r(&t, &ti);
  sprintf(buf, "%02d:%02d", ti.tm_hour, ti.tm_min);
}

// ════════════════════════════════════════════════
//  DRAWING PRIMITIVES
// ════════════════════════════════════════════════

void drawBar(int x, int y, int w, int h, float v, float lo, float hi) {
  int f = (int)((constrain(v, lo, hi) - lo) / (hi - lo) * w);
  display.drawRect(x, y, w, h, SSD1306_WHITE);
  display.fillRect(x, y, f, h, SSD1306_WHITE);
}

// ── Procedural weather icons ───────────────────

void drawSun(int cx, int cy, int r) {
  display.fillCircle(cx, cy, r, SSD1306_WHITE);
  for (int a = 0; a < 360; a += 45) {
    float rad = a * PI / 180.0;
    int x1 = cx + cos(rad) * (r + 2);
    int y1 = cy + sin(rad) * (r + 2);
    int x2 = cx + cos(rad) * (r + 5);
    int y2 = cy + sin(rad) * (r + 5);
    display.drawLine(x1, y1, x2, y2, SSD1306_WHITE);
  }
}

void drawCloud(int x, int y) {
  display.fillCircle(x + 5,  y + 8, 4, SSD1306_WHITE);
  display.fillCircle(x + 11, y + 5, 5, SSD1306_WHITE);
  display.fillCircle(x + 17, y + 8, 4, SSD1306_WHITE);
  display.fillRect(x + 3, y + 8, 16, 4, SSD1306_WHITE);
}

void drawDrop(int cx, int cy) {
  display.fillCircle(cx, cy + 2, 4, SSD1306_WHITE);
  display.fillTriangle(cx - 3, cy + 1, cx + 3, cy + 1, cx, cy - 6, SSD1306_WHITE);
}

void drawSnow(int cx, int cy, int r) {
  display.drawLine(cx, cy - r, cx, cy + r, SSD1306_WHITE);
  display.drawLine(cx - r, cy, cx + r, cy, SSD1306_WHITE);
  display.drawLine(cx - r + 1, cy - r + 1, cx + r - 1, cy + r - 1, SSD1306_WHITE);
  display.drawLine(cx - r + 1, cy + r - 1, cx + r - 1, cy - r + 1, SSD1306_WHITE);
}

void drawThunder(int cx, int cy) {
  display.drawLine(cx,     cy - 6, cx - 2, cy,     SSD1306_WHITE);
  display.drawLine(cx - 2, cy,     cx + 1, cy,     SSD1306_WHITE);
  display.drawLine(cx + 1, cy,     cx - 1, cy + 6, SSD1306_WHITE);
}

void drawMist(int x, int y) {
  for (int i = 0; i < 4; i++)
    display.drawLine(x + 2, y + 3 + i * 3, x + 16, y + 3 + i * 3, SSD1306_WHITE);
}

// Draw by icon type (0-6) at full size
void drawIconType(int type, int x, int y) {
  switch (type) {
    case 0: drawSun(x + 8, y + 7, 4); break;
    case 1: drawSun(x + 14, y + 3, 3); drawCloud(x - 2, y); break;
    case 2: drawCloud(x, y); break;
    case 3: drawDrop(x + 8, y + 8); break;
    case 4: drawSnow(x + 8, y + 7, 6); break;
    case 5: drawThunder(x + 8, y + 7); break;
    case 6: drawMist(x, y); break;
  }
}

// Draw by icon type at small size (for forecast columns)
void drawIconSmall(int type, int cx, int cy) {
  switch (type) {
    case 0: drawSun(cx, cy, 3); break;
    case 1: drawSun(cx + 4, cy - 2, 2); drawCloud(cx - 7, cy - 4); break;
    case 2:
      display.fillCircle(cx - 5, cy, 3, SSD1306_WHITE);
      display.fillCircle(cx,     cy - 2, 4, SSD1306_WHITE);
      display.fillCircle(cx + 5, cy, 3, SSD1306_WHITE);
      display.fillRect(cx - 7, cy, 14, 3, SSD1306_WHITE);
      break;
    case 3: display.fillCircle(cx, cy, 3, SSD1306_WHITE);
            display.fillTriangle(cx - 2, cy, cx + 2, cy, cx, cy - 5, SSD1306_WHITE); break;
    case 4: drawSnow(cx, cy, 4); break;
    case 5: drawThunder(cx, cy); break;
    case 6:
      for (int i = 0; i < 3; i++)
        display.drawLine(cx - 5, cy - 2 + i * 3, cx + 5, cy - 2 + i * 3, SSD1306_WHITE);
      break;
  }
}

// Indoor icon (based on sensor readings)
void drawIndoorIcon(int x, int y) {
  if      (temp < 10)  drawIconType(4, x, y);
  else if (hum  > 70)  drawIconType(3, x, y);
  else if (temp > 30)  drawIconType(0, x, y);
  else                 drawIconType(2, x, y);
}

// ── Sparkline ──────────────────────────────────

void sparkPush(float v) {
  sparkBuf[sparkHead] = v;
  sparkHead = (sparkHead + 1) % SPARK_CAP;
  if (sparkN < SPARK_CAP) sparkN++;
}

void drawSparkline(int x, int y, int w, int h) {
  if (sparkN < 2) {
    display.setCursor(x + 4, y + h / 2 - 3);
    display.print("collecting...");
    return;
  }
  display.drawRect(x, y, w, h, SSD1306_WHITE);

  float mn = 1e6, mx = -1e6;
  for (int i = 0; i < sparkN; i++) {
    int idx = (sparkHead - sparkN + i + SPARK_CAP) % SPARK_CAP;
    mn = min(mn, sparkBuf[idx]);
    mx = max(mx, sparkBuf[idx]);
  }
  if (mx - mn < 1.0f) { mn -= 0.5f; mx += 0.5f; }

  int px = -1, py = -1;
  for (int i = 0; i < sparkN; i++) {
    int idx = (sparkHead - sparkN + i + SPARK_CAP) % SPARK_CAP;
    int xi = x + 1 + (int)((float)i / (sparkN - 1) * (w - 3));
    int yi = y + h - 2 - (int)((sparkBuf[idx] - mn) / (mx - mn) * (h - 4));
    if (px >= 0) display.drawLine(px, py, xi, yi, SSD1306_WHITE);
    px = xi; py = yi;
  }

  char lb[6];
  dtostrf(mn, 3, 0, lb); display.setCursor(x + w + 2, y + h - 8); display.print(lb);
  dtostrf(mx, 3, 0, lb); display.setCursor(x + w + 2, y);         display.print(lb);
}

// ════════════════════════════════════════════════
//  OPEN-METEO  FETCH  (single call, no API key)
// ════════════════════════════════════════════════

void fetchWeather() {
  if (WiFi.status() != WL_CONNECTED) return;

  String url =
    "https://api.open-meteo.com/v1/forecast"
    "?latitude="  WX_LAT
    "&longitude=" WX_LON
    "&current=temperature_2m,relative_humidity_2m,apparent_temperature,"
               "weather_code,wind_speed_10m"
    "&daily=temperature_2m_max,temperature_2m_min,sunrise,sunset"
    "&hourly=temperature_2m,weather_code"
    "&timezone=" WX_TZ
    "&forecast_days=2";

  HTTPClient http;
  http.begin(url);
  int code = http.GET();

  if (code != 200) {
    Serial.printf("[WX] fetch failed: %d\n", code);
    http.end();
    return;
  }

  String payload = http.getString();
  http.end();

  JsonDocument doc;
  if (deserializeJson(doc, payload)) {
    Serial.println("[WX] JSON parse error");
    return;
  }

  // ── Current ──
  JsonObject cur = doc["current"];
  wx.temp      = cur["temperature_2m"]      | 0.0f;
  wx.feelsLike = cur["apparent_temperature"] | 0.0f;
  wx.humidity  = cur["relative_humidity_2m"] | 0.0f;
  wx.windSpeed = cur["wind_speed_10m"]       | 0.0f;
  wx.wmoCode   = cur["weather_code"]        | 0;

  // ── Daily (today = index 0) ──
  wx.dayMin = doc["daily"]["temperature_2m_min"][0] | 0.0f;
  wx.dayMax = doc["daily"]["temperature_2m_max"][0] | 0.0f;
  parseHHMM(doc["daily"]["sunrise"][0] | "", wx.sunrise);
  parseHHMM(doc["daily"]["sunset"][0]  | "", wx.sunset);

  wx.valid = true;
  Serial.printf("[WX] %.1fC feels %.1fC | WMO:%d | Lo:%.1f Hi:%.1f | %s-%s\n",
                wx.temp, wx.feelsLike, wx.wmoCode, wx.dayMin, wx.dayMax,
                wx.sunrise, wx.sunset);

  // ── Hourly forecast — pick next 3 slots at +3h, +6h, +9h ──
  struct tm ti;
  getLocalTime(&ti, 100);
  int baseIdx = ti.tm_hour + 3;   // first slot ~3h from now

  JsonArray hTime = doc["hourly"]["time"];
  JsonArray hTemp = doc["hourly"]["temperature_2m"];
  JsonArray hCode = doc["hourly"]["weather_code"];
  int hLen = hTime.size();

  fcCount = 0;
  for (int s = 0; s < MAX_FORECASTS; s++) {
    int idx = baseIdx + s * 3;
    if (idx >= hLen) break;
    fc[s].hour    = (idx) % 24;
    fc[s].temp    = hTemp[idx] | 0.0f;
    fc[s].wmoCode = hCode[idx] | 0;
    fcCount++;
  }
  Serial.printf("[WX] Forecast: %d slots\n", fcCount);
}

// ════════════════════════════════════════════════
//  CHROME  (inverted header + live clock + dots)
// ════════════════════════════════════════════════

void drawChrome(const char* title, int idx, int xo) {
  display.fillRect(xo, 0, 128, 11, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(xo + 2, 2);
  display.print(title);

  if (timeReady) {
    struct tm ti;
    if (getLocalTime(&ti, 10)) {
      char c[6];
      sprintf(c, "%02d:%02d", ti.tm_hour, ti.tm_min);
      display.setCursor(xo + 98, 2);
      display.print(c);
    }
  }

  display.setTextColor(SSD1306_WHITE);
  int dx = xo + 64 - NUM_SCREENS * 4;
  for (int i = 0; i < NUM_SCREENS; i++) {
    int cx = dx + i * 8 + 3;
    if (i == idx) display.fillCircle(cx, 60, 2, SSD1306_WHITE);
    else          display.drawCircle(cx, 60, 2, SSD1306_WHITE);
  }
}

// ════════════════════════════════════════════════
//  SCREENS
// ════════════════════════════════════════════════

// 0 ── Clock & Date ─────────────────────────────
void scrClock(int xo) {
  drawChrome("TIME", 0, xo);
  display.setTextColor(SSD1306_WHITE);

  struct tm ti;
  if (timeReady && getLocalTime(&ti, 10)) {
    display.setTextSize(3);
    bool colon = (millis() / 500) % 2;
    char tb[6];
    sprintf(tb, "%02d%c%02d", ti.tm_hour, colon ? ':' : ' ', ti.tm_min);
    display.setCursor(xo + 4, 15);
    display.print(tb);

    display.setTextSize(2);
    char sb[3];
    sprintf(sb, "%02d", ti.tm_sec);
    display.setCursor(xo + 98, 19);
    display.print(sb);

    display.setTextSize(1);
    static const char* dn[] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
    static const char* mn[] = {"Jan","Feb","Mar","Apr","May","Jun",
                               "Jul","Aug","Sep","Oct","Nov","Dec"};
    char db[24];
    sprintf(db, "%s, %02d %s %04d",
            dn[ti.tm_wday], ti.tm_mday, mn[ti.tm_mon], ti.tm_year + 1900);
    int tw = strlen(db) * 6;
    display.setCursor(xo + (128 - tw) / 2, 46);
    display.print(db);
  } else {
    display.setTextSize(1);
    display.setCursor(xo + 18, 30);
    display.print("Syncing time...");
  }
}

// 1 ── Indoor Weather ───────────────────────────
void scrWeather(int xo) {
  drawChrome("INDOOR", 1, xo);
  display.setTextColor(SSD1306_WHITE);

  drawIndoorIcon(xo + 104, 13);

  display.setTextSize(2);
  display.setCursor(xo, 14);
  display.print(temp, 1);
  display.setTextSize(1);
  display.print(" C");

  display.setCursor(xo, 33);
  display.print("Humidity: ");
  display.print(hum, 1);
  display.print("%");
  drawBar(xo, 43, 96, 5, hum, 0, 100);

  display.setCursor(xo, 51);
  display.print("Pressure: ");
  if (bmpFound) { display.print(pressure, 0); display.print(" hPa"); }
  else          display.print("N/A");
}

// 2 ── Thermal Comfort ──────────────────────────
void scrComfort(int xo) {
  drawChrome("COMFORT", 2, xo);
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(xo, 14);
  display.print("Feels Like:");

  display.setTextSize(2);
  display.setCursor(xo, 24);
  display.print(heatIdx, 1);
  display.setTextSize(1);
  display.print(" C");

  display.setCursor(xo, 42);
  display.print("Dew Point: ");
  display.print(dewPt, 1);
  display.print(" C");

  display.setCursor(xo, 52);
  display.print("> ");
  display.print(comfortLabel(temp, hum));
}

// 3 ── Outdoor Weather (Open-Meteo) ────────────
void scrOutdoor(int xo) {
  drawChrome("OUTDOOR", 3, xo);
  display.setTextColor(SSD1306_WHITE);

  if (!wx.valid) {
    display.setTextSize(1);
    display.setCursor(xo + 18, 30);
    display.print("Fetching data...");
    return;
  }

  // ── Row 1: Icon + City ──
  drawIconType(wmoIcon(wx.wmoCode), xo, 12);
  display.setTextSize(1);
  display.setCursor(xo + 26, 14);
  display.print(WX_CITY);

  // ── Row 2: Big temperature ──
  display.setTextSize(2);
  display.setCursor(xo + 26, 24);
  display.print(wx.temp, 1);
  display.setTextSize(1);
  display.print("C");

  // ── Row 3: Description ──
  display.setCursor(xo, 41);
  display.print(wmoDesc(wx.wmoCode));

  // ── Row 4: Lo/Hi  +  Sunrise/Sunset ──
  display.setCursor(xo, 51);
  display.print(wx.dayMin, 0);
  display.print("/");
  display.print(wx.dayMax, 0);
  display.print("C  ^");
  display.print(wx.sunrise);
  display.print(" v");
  display.print(wx.sunset);
}

// 4 ── 3-Hour Forecast (Open-Meteo hourly) ─────
void scrForecast(int xo) {
  drawChrome("FORECAST", 4, xo);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  if (fcCount == 0) {
    display.setCursor(xo + 18, 30);
    display.print("Fetching data...");
    return;
  }

  const int colW = 42;
  for (int i = 0; i < fcCount; i++) {
    int cx = xo + i * colW;

    // Separator
    if (i > 0) display.drawLine(xo + i * colW - 1, 12, xo + i * colW - 1, 56, SSD1306_WHITE);

    // Time label  "14:00"
    char tb[6];
    sprintf(tb, "%02d:00", fc[i].hour);
    int tw = strlen(tb) * 6;
    display.setCursor(cx + (colW - tw) / 2, 14);
    display.print(tb);

    // Icon centred in column
    drawIconSmall(wmoIcon(fc[i].wmoCode), cx + colW / 2, 30);

    // Temperature
    char tempBuf[7];
    dtostrf(fc[i].temp, 3, 1, tempBuf);
    int tempW = (strlen(tempBuf) + 1) * 6;
    display.setCursor(cx + (colW - tempW) / 2, 41);
    display.print(tempBuf);
    display.print("C");

    // Short condition label (7 chars max to fit column)
    const char* d = wmoDesc(fc[i].wmoCode);
    char descBuf[8];
    strlcpy(descBuf, d, sizeof(descBuf));
    int descW = strlen(descBuf) * 6;
    display.setCursor(cx + (colW - min(descW, colW)) / 2, 51);
    display.print(descBuf);
  }
}

// 5 ── Pressure & Altitude ──────────────────────
void scrPressure(int xo) {
  drawChrome("PRESSURE", 5, xo);
  display.setTextColor(SSD1306_WHITE);

  if (bmpFound) {
    display.setTextSize(1);
    display.setCursor(xo, 14);
    display.print("Pressure:");
    display.setTextSize(2);
    display.setCursor(xo, 24);
    display.print(pressure, 1);
    display.setTextSize(1);
    display.print(" hPa");

    drawBar(xo, 39, 100, 5, pressure, 900, 1050);
    display.setCursor(xo + 104, 38);
    if      (pressure < 1009) display.print("Low");
    else if (pressure < 1022) display.print("Norm");
    else                      display.print("High");

    display.setCursor(xo, 48);
    display.print("Altitude: ");
    display.print(alt, 0);
    display.print(" m");
  } else {
    display.setTextSize(1);
    display.setCursor(xo + 10, 30);
    display.print("BMP180 not found");
  }
}

// 6 ── System + Sparkline ───────────────────────
void scrSystem(int xo) {
  drawChrome("SYSTEM", 6, xo);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  unsigned long s = (millis() - startTime) / 1000;
  unsigned long m = s / 60, h = m / 60;
  s %= 60; m %= 60;

  char ub[12]; sprintf(ub, "%02lu:%02lu:%02lu", h, m, s);
  display.setCursor(xo, 14);
  display.print("Up ");
  display.print(ub);

  display.setCursor(xo + 78, 14);
  display.print("DHT:");
  display.print(isnan(temp) ? "!" : "OK");

  display.setCursor(xo, 24);
  display.print("WiFi:");
  display.print(WiFi.status() == WL_CONNECTED ? "OK" : "--");
  display.setCursor(xo + 48, 24);
  display.print("BMP:");
  display.print(bmpFound ? "OK" : "--");
  display.setCursor(xo + 86, 24);
  display.print("API:");
  display.print(wx.valid ? "OK" : "--");

  display.setCursor(xo, 35);
  display.print("Temp trend");
  drawSparkline(xo, 44, 104, 14);
}

// ════════════════════════════════════════════════
//  SCREEN DISPATCH + TRANSITION ENGINE
// ════════════════════════════════════════════════

typedef void (*ScrFn)(int);
ScrFn scrTable[] = {
  scrClock, scrWeather, scrComfort, scrOutdoor, scrForecast, scrPressure, scrSystem
};

void renderFrame() {
  display.clearDisplay();

  if (sliding && prevScr >= 0) {
    scrTable[prevScr](-slideOff);
    scrTable[curScr](128 - slideOff);
    slideOff += SLIDE_STEP;
    if (slideOff >= 128) { sliding = false; prevScr = curScr; }
  } else {
    scrTable[curScr](0);
  }

  display.display();
}

// ════════════════════════════════════════════════
//  ANIMATED BOOT SPLASH
// ════════════════════════════════════════════════

void bootSplash() {
  for (int i = 0; i <= 128; i += 8) {
    display.clearDisplay();
    int w = min(i, 128), h = min(i / 2, 64);
    if (w > 0 && h > 0) display.drawRect(0, 0, w, h, SSD1306_WHITE);
    display.display();
    delay(15);
  }
  display.clearDisplay();
  display.drawRect(0, 0, 128, 64, SSD1306_WHITE);

  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  const char* line1 = "ESP32";
  display.setCursor(46, 6);
  for (int i = 0; line1[i]; i++) { display.print(line1[i]); display.display(); delay(60); }

  const char* line2 = "WEATHER STATION";
  display.setCursor(17, 18);
  for (int i = 0; line2[i]; i++) { display.print(line2[i]); display.display(); delay(35); }
  delay(200);

  display.setCursor(12, 32);
  display.print("DHT22  : OK");
  display.display(); delay(250);

  display.setCursor(12, 42);
  display.print("BMP180 : ");
  display.print(bmpFound ? "OK" : "N/A");
  display.display(); delay(250);

  display.setCursor(12, 52);
  display.print("WiFi   : ");
  display.display();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  const int barX = 12, barW = 104, barY = 58, barH = 4;
  const int maxAttempts = 20;
  for (int a = 0; a < maxAttempts; a++) {
    display.fillRect(barX, barY, (a + 1) * barW / maxAttempts, barH, SSD1306_WHITE);
    display.display();
    if (WiFi.status() == WL_CONNECTED) {
      display.fillRect(barX, barY, barW, barH, SSD1306_WHITE);
      display.setCursor(66, 52); display.print("OK");
      display.display(); delay(400);
      break;
    }
    delay(500);
  }
  if (WiFi.status() != WL_CONNECTED) {
    display.setCursor(66, 52); display.print("FAIL");
    display.display(); delay(600);
  }

  display.invertDisplay(true);  delay(80);
  display.invertDisplay(false); delay(80);
  display.invertDisplay(true);  delay(60);
  display.invertDisplay(false); delay(300);
}

// ════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  dht.begin();
  startTime = millis();

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 not found!");
    while (1);
  }

  bmpFound = bmp.begin();
  Serial.println(bmpFound ? "BMP180 OK" : "BMP180 not found");

  bootSplash();

  if (WiFi.status() == WL_CONNECTED) {
    configTime(GMT_OFFSET_SEC, DST_OFFSET_SEC, NTP_SERVER);
    Serial.println("NTP configured");
    fetchWeather();
  }
}

// ════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════

void loop() {
  unsigned long now = millis();

  if (!timeReady) {
    struct tm ti;
    if (getLocalTime(&ti, 10)) { timeReady = true; Serial.println("Time synced"); }
  }

  if (WiFi.status() != WL_CONNECTED) {
    static unsigned long lastRecon = 0;
    if (now - lastRecon > 30000) { lastRecon = now; WiFi.reconnect(); }
  }

  if (WiFi.status() == WL_CONNECTED && !wxFetching) {
    if (now - lastWxMs >= WX_UPDATE_MS || (lastWxMs == 0 && !wx.valid)) {
      lastWxMs = now;
      wxFetching = true;
      fetchWeather();
      wxFetching = false;
    }
  }

  if (now - lastRead >= READ_MS) {
    lastRead = now;
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t) && !isnan(h)) {
      temp    = t;
      hum     = h;
      heatIdx = dht.computeHeatIndex(temp, hum, false);
      dewPt   = computeDewPoint(temp, hum);
    }
    if (bmpFound) {
      pressure = bmp.readPressure() / 100.0f;
      alt      = bmp.readAltitude();
    }
    Serial.printf("[%lu] T:%.1f H:%.1f P:%.1f HI:%.1f DP:%.1f A:%.0f\n",
                  now / 1000, temp, hum, pressure, heatIdx, dewPt, alt);
  }

  if (now - lastSpark >= SPARK_MS) {
    lastSpark = now;
    if (!isnan(temp)) sparkPush(temp);
  }

  if (!sliding && now - lastSwitch >= SCREEN_MS) {
    lastSwitch = now;
    prevScr = curScr;
    curScr  = (curScr + 1) % NUM_SCREENS;
    sliding  = true;
    slideOff = 0;
  }

  renderFrame();
  delay(sliding ? 25 : 80);
}
