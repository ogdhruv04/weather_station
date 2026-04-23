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
#define GMT_OFFSET_SEC  19800          // IST  (UTC+5:30)
#define DST_OFFSET_SEC  0

// OpenWeatherMap — get a free key at https://openweathermap.org/api
#define OWM_API_KEY     "07bd2426a16fd1aecbf4fced394e6802"
#define OWM_CITY        "Roorkee"        // change to your city
#define OWM_UNITS       "metric"       // "metric" = Celsius, "imperial" = Fahrenheit
#define OWM_UPDATE_MS   600000         // refresh every 10 min

#define DHTPIN          4
#define DHTTYPE         DHT22
#define SDA_PIN         21
#define SCL_PIN         22
#define SCREEN_W        128
#define SCREEN_H        64
#define OLED_RESET      -1
#define OLED_ADDR       0x3C

#define NUM_SCREENS     7
#define SCREEN_MS       5000           // rotate every 5 s
#define READ_MS         2000           // sensor poll
#define SPARK_CAP       60             // sparkline samples
#define SPARK_MS        10000          // sample every 10 s
#define MAX_FORECASTS   3              // columns shown on forecast screen
#define FC_FETCH_CNT    8              // slots fetched (covers ~24 h)

// ── Objects ────────────────────────────────────
DHT              dht(DHTPIN, DHTTYPE);
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, OLED_RESET);
Adafruit_BMP085  bmp;

// ── State ──────────────────────────────────────
bool  bmpFound   = false;
bool  timeReady  = false;
int   curScr     = 0;
int   prevScr    = -1;

unsigned long lastSwitch = 0, lastRead = 0, startTime = 0;
unsigned long lastSpark  = 0, lastOwm  = 0;

// Transition
bool  sliding     = false;
int   slideOff    = 0;
const int SLIDE_STEP = 32;

// Indoor sensors
float temp = 0, hum = 0, pressure = 0;
float heatIdx = 0, dewPt = 0, alt = 0;

// Sparkline ring-buffer
float sparkBuf[SPARK_CAP];
int   sparkHead = 0, sparkN = 0;

// ── OpenWeatherMap data ────────────────────────
struct OwmCurrent {
  char  city[20];
  char  desc[24];
  char  icon[4];
  float temp, feelsLike;
  float humidity, windSpeed;
  int   windDeg;
  long  sunrise, sunset;
  bool  valid;
} owmNow = { "", "", "", 0,0,0,0,0,0,0, false };

// Derived from forecast slots — the real daily range
float fcDayMin =  999;
float fcDayMax = -999;

struct OwmForecast {
  long  dt;
  float temp;
  char  icon[4];
  char  desc[20];
} owmFc[MAX_FORECASTS];

int  owmFcCount = 0;
bool owmFetching = false;

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

// Format a unix timestamp → "HH:MM" (IST)
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
  display.fillCircle(x + 5, y + 8, 4, SSD1306_WHITE);
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
  // Lightning bolt
  display.drawLine(cx, cy - 6, cx - 2, cy, SSD1306_WHITE);
  display.drawLine(cx - 2, cy, cx + 1, cy, SSD1306_WHITE);
  display.drawLine(cx + 1, cy, cx - 1, cy + 6, SSD1306_WHITE);
}

void drawMist(int x, int y) {
  for (int i = 0; i < 4; i++) {
    int yy = y + 3 + i * 3;
    display.drawLine(x + 2, yy, x + 16, yy, SSD1306_WHITE);
  }
}

// Draw icon based on sensor readings (indoor)
void drawWeatherIcon(int x, int y) {
  if (temp < 10)      drawSnow(x + 8, y + 7, 6);
  else if (hum > 70)  drawDrop(x + 8, y + 8);
  else if (temp > 30) drawSun(x + 8, y + 7, 4);
  else                drawCloud(x, y);
}

// Draw icon based on OWM icon code (outdoor)
void drawOwmIcon(int x, int y, const char* icon) {
  // icon codes: 01=clear, 02=few clouds, 03/04=clouds,
  //             09/10=rain, 11=thunder, 13=snow, 50=mist
  if      (strncmp(icon, "01", 2) == 0) drawSun(x + 8, y + 7, 4);
  else if (strncmp(icon, "02", 2) == 0) { drawSun(x + 14, y + 3, 3); drawCloud(x - 2, y); }
  else if (strncmp(icon, "03", 2) == 0) drawCloud(x, y);
  else if (strncmp(icon, "04", 2) == 0) drawCloud(x, y);
  else if (strncmp(icon, "09", 2) == 0) drawDrop(x + 8, y + 8);
  else if (strncmp(icon, "10", 2) == 0) drawDrop(x + 8, y + 8);
  else if (strncmp(icon, "11", 2) == 0) drawThunder(x + 8, y + 7);
  else if (strncmp(icon, "13", 2) == 0) drawSnow(x + 8, y + 7, 6);
  else if (strncmp(icon, "50", 2) == 0) drawMist(x, y);
  else                                   drawCloud(x, y);
}

// Small icon (for forecast columns, fits ~12px)
void drawOwmIconSmall(int x, int y, const char* icon) {
  if      (strncmp(icon, "01", 2) == 0) drawSun(x + 5, y + 5, 3);
  else if (strncmp(icon, "02", 2) == 0) { drawSun(x + 10, y + 2, 2); drawCloud(x - 2, y); }
  else if (strncmp(icon, "03", 2) == 0 || strncmp(icon, "04", 2) == 0) {
    display.fillCircle(x + 3, y + 6, 3, SSD1306_WHITE);
    display.fillCircle(x + 8, y + 4, 4, SSD1306_WHITE);
    display.fillCircle(x + 13, y + 6, 3, SSD1306_WHITE);
    display.fillRect(x + 2, y + 6, 12, 3, SSD1306_WHITE);
  }
  else if (strncmp(icon, "09", 2) == 0 || strncmp(icon, "10", 2) == 0) {
    display.fillCircle(x + 6, y + 4, 3, SSD1306_WHITE);
    display.fillTriangle(x + 4, y + 3, x + 8, y + 3, x + 6, y - 2, SSD1306_WHITE);
  }
  else if (strncmp(icon, "11", 2) == 0) drawThunder(x + 6, y + 5);
  else if (strncmp(icon, "13", 2) == 0) drawSnow(x + 6, y + 5, 4);
  else if (strncmp(icon, "50", 2) == 0) {
    for (int i = 0; i < 3; i++) display.drawLine(x + 1, y + 3 + i * 3, x + 12, y + 3 + i * 3, SSD1306_WHITE);
  }
  else drawCloud(x - 2, y);
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
  dtostrf(mn, 3, 0, lb);
  display.setCursor(x + w + 2, y + h - 8);
  display.print(lb);
  dtostrf(mx, 3, 0, lb);
  display.setCursor(x + w + 2, y);
  display.print(lb);
}

// ════════════════════════════════════════════════
//  OPENWEATHERMAP  FETCH
// ════════════════════════════════════════════════

void fetchCurrentWeather() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  String url = "http://api.openweathermap.org/data/2.5/weather?q=";
  url += OWM_CITY;
  url += "&appid=";
  url += OWM_API_KEY;
  url += "&units=";
  url += OWM_UNITS;

  http.begin(url);
  int code = http.GET();
  if (code == 200) {
    String payload = http.getString();
    JsonDocument doc;
    if (!deserializeJson(doc, payload)) {
      strlcpy(owmNow.city, doc["name"] | "???", sizeof(owmNow.city));
      strlcpy(owmNow.desc, doc["weather"][0]["description"] | "", sizeof(owmNow.desc));
      strlcpy(owmNow.icon, doc["weather"][0]["icon"] | "03d", sizeof(owmNow.icon));
      owmNow.temp      = doc["main"]["temp"]       | 0.0f;
      owmNow.feelsLike = doc["main"]["feels_like"] | 0.0f;
      owmNow.humidity  = doc["main"]["humidity"]   | 0.0f;
      owmNow.windSpeed = doc["wind"]["speed"]      | 0.0f;
      owmNow.windDeg   = doc["wind"]["deg"]        | 0;
      owmNow.sunrise   = doc["sys"]["sunrise"]     | 0L;
      owmNow.sunset    = doc["sys"]["sunset"]      | 0L;
      owmNow.valid     = true;
      Serial.printf("[OWM] Current: %s %.1fC (%s)\n", owmNow.city, owmNow.temp, owmNow.desc);
    }
  } else {
    Serial.printf("[OWM] Current failed: %d\n", code);
  }
  http.end();
}

void fetchForecast() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  String url = "http://api.openweathermap.org/data/2.5/forecast?q=";
  url += OWM_CITY;
  url += "&appid=";
  url += OWM_API_KEY;
  url += "&units=";
  url += OWM_UNITS;
  url += "&cnt=";
  url += FC_FETCH_CNT;

  http.begin(url);
  int code = http.GET();
  if (code == 200) {
    String payload = http.getString();
    JsonDocument doc;
    if (!deserializeJson(doc, payload)) {
      JsonArray list = doc["list"];
      owmFcCount = 0;
      fcDayMin =  999;
      fcDayMax = -999;

      for (int i = 0; i < (int)list.size(); i++) {
        float t = list[i]["main"]["temp"] | 0.0f;
        if (t < fcDayMin) fcDayMin = t;
        if (t > fcDayMax) fcDayMax = t;

        // Store first MAX_FORECASTS slots for the forecast screen
        if (i < MAX_FORECASTS) {
          owmFc[i].dt   = list[i]["dt"] | 0L;
          owmFc[i].temp = t;
          strlcpy(owmFc[i].icon, list[i]["weather"][0]["icon"] | "03d", sizeof(owmFc[i].icon));
          strlcpy(owmFc[i].desc, list[i]["weather"][0]["main"] | "", sizeof(owmFc[i].desc));
          owmFcCount++;
        }
      }
      Serial.printf("[OWM] Forecast: %d slots, Lo:%.1f Hi:%.1f\n",
                    (int)list.size(), fcDayMin, fcDayMax);
    }
  } else {
    Serial.printf("[OWM] Forecast failed: %d\n", code);
  }
  http.end();
}

void fetchOwm() {
  owmFetching = true;
  fetchCurrentWeather();
  fetchForecast();
  owmFetching = false;
}

// ════════════════════════════════════════════════
//  CHROME  (header with live clock + page dots)
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
//  SCREENS   (each takes an x-offset for sliding)
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
    sprintf(db, "%s, %02d %s %04d", dn[ti.tm_wday], ti.tm_mday,
            mn[ti.tm_mon], ti.tm_year + 1900);
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

  drawWeatherIcon(xo + 104, 13);

  display.setTextSize(2);
  display.setCursor(xo + 0, 14);
  display.print(temp, 1);
  display.setTextSize(1);
  display.print(" C");

  display.setCursor(xo + 0, 33);
  display.print("Humidity: ");
  display.print(hum, 1);
  display.print("%");
  drawBar(xo, 43, 96, 5, hum, 0, 100);

  display.setCursor(xo + 0, 51);
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

// 3 ── Outdoor Weather (OWM) ────────────────────
void scrOutdoor(int xo) {
  drawChrome("OUTDOOR", 3, xo);
  display.setTextColor(SSD1306_WHITE);

  if (!owmNow.valid) {
    display.setTextSize(1);
    display.setCursor(xo + 10, 25);
    if (strcmp(OWM_API_KEY, "YOUR_API_KEY_HERE") == 0)
      display.print("Set OWM_API_KEY!");
    else
      display.print("Fetching data...");
    return;
  }

  // ── Row 1 (y12): Icon + City name ──
  drawOwmIcon(xo + 0, 12, owmNow.icon);
  display.setTextSize(1);
  display.setCursor(xo + 26, 14);
  display.print(owmNow.city);

  // ── Row 2 (y24): Big temperature ──
  display.setTextSize(2);
  display.setCursor(xo + 26, 24);
  display.print(owmNow.temp, 1);
  display.setTextSize(1);
  display.print("C");

  // ── Row 3 (y41): Description ──
  display.setCursor(xo, 41);
  display.print(owmNow.desc);

  // ── Row 4 (y51): Lo/Hi + Sunrise/Sunset ──
  display.setCursor(xo, 51);
  display.print(fcDayMin, 0);
  display.print("/");
  display.print(fcDayMax, 0);
  display.print("C");
  if (owmNow.sunrise > 0) {
    char buf[6];
    display.print("  ^");
    fmtTime(owmNow.sunrise, buf);
    display.print(buf);
    display.print(" v");
    fmtTime(owmNow.sunset, buf);
    display.print(buf);
  }
}

// 4 ── 3-Hour Forecast (OWM) ───────────────────
void scrForecast(int xo) {
  drawChrome("FORECAST", 4, xo);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  if (owmFcCount == 0) {
    display.setCursor(xo + 10, 25);
    if (strcmp(OWM_API_KEY, "YOUR_API_KEY_HERE") == 0)
      display.print("Set OWM_API_KEY!");
    else
      display.print("Fetching data...");
    return;
  }

  // 3 columns, each ~40px wide
  int colW = 42;
  for (int i = 0; i < owmFcCount && i < 3; i++) {
    int cx = xo + i * colW + 1;

    // Time
    char tb[6];
    fmtTime(owmFc[i].dt, tb);
    int tw = strlen(tb) * 6;
    display.setCursor(cx + (colW - tw) / 2, 14);
    display.print(tb);

    // Separator lines between columns
    if (i > 0) display.drawLine(xo + i * colW - 1, 12, xo + i * colW - 1, 56, SSD1306_WHITE);

    // Icon (centred in column)
    drawOwmIconSmall(cx + (colW - 16) / 2, 24, owmFc[i].icon);

    // Temp
    char tempBuf[8];
    dtostrf(owmFc[i].temp, 3, 1, tempBuf);
    int tempW = strlen(tempBuf) * 6 + 6; // +6 for "C"
    display.setCursor(cx + (colW - tempW) / 2, 40);
    display.print(tempBuf);
    display.print("C");

    // Short description
    int descW = strlen(owmFc[i].desc) * 6;
    display.setCursor(cx + (colW - min(descW, colW)) / 2, 50);
    // Truncate if too wide
    char descBuf[8];
    strlcpy(descBuf, owmFc[i].desc, min((int)sizeof(descBuf), colW / 6 + 1));
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
  display.print(owmNow.valid ? "OK" : "--");

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
    if (slideOff >= 128) {
      sliding = false;
      prevScr = curScr;
    }
  } else {
    scrTable[curScr](0);
  }

  display.display();
}

// ════════════════════════════════════════════════
//  ANIMATED BOOT SPLASH
// ════════════════════════════════════════════════

void bootSplash() {
  // 1. Border draws itself
  for (int i = 0; i <= 128; i += 8) {
    display.clearDisplay();
    int w = min(i, 128), h = min(i / 2, 64);
    if (w > 0 && h > 0) display.drawRect(0, 0, w, h, SSD1306_WHITE);
    display.display();
    delay(15);
  }

  display.clearDisplay();
  display.drawRect(0, 0, 128, 64, SSD1306_WHITE);

  // 2. Title — typing effect
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  const char* line1 = "ESP32";
  display.setCursor(46, 6);
  for (int i = 0; line1[i]; i++) {
    display.print(line1[i]); display.display(); delay(60);
  }

  const char* line2 = "WEATHER STATION";
  display.setCursor(17, 18);
  for (int i = 0; line2[i]; i++) {
    display.print(line2[i]); display.display(); delay(35);
  }
  delay(200);

  // 3. Sensor status
  display.setCursor(12, 32);
  display.print("DHT22  : OK");
  display.display(); delay(250);

  display.setCursor(12, 42);
  display.print("BMP180 : ");
  display.print(bmpFound ? "OK" : "N/A");
  display.display(); delay(250);

  // 4. WiFi connect with progress bar
  display.setCursor(12, 52);
  display.print("WiFi   : ");
  display.display();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int barX = 12, barW = 104, barY = 58, barH = 4;
  int maxAttempts = 20;

  for (int a = 0; a < maxAttempts; a++) {
    int fill = (int)((float)(a + 1) / maxAttempts * barW);
    display.fillRect(barX, barY, fill, barH, SSD1306_WHITE);
    display.display();

    if (WiFi.status() == WL_CONNECTED) {
      display.fillRect(barX, barY, barW, barH, SSD1306_WHITE);
      display.setCursor(66, 52);
      display.print("OK");
      display.display();
      delay(400);
      break;
    }
    delay(500);
  }

  if (WiFi.status() != WL_CONNECTED) {
    display.setCursor(66, 52);
    display.print("FAIL");
    display.display();
    delay(600);
  }

  // Flash effect
  display.invertDisplay(true);  delay(80);
  display.invertDisplay(false); delay(80);
  display.invertDisplay(true);  delay(60);
  display.invertDisplay(false);
  delay(300);
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
    // First weather fetch
    fetchOwm();
  }
}

// ════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════

void loop() {
  unsigned long now = millis();

  // ── Check time sync ──
  if (!timeReady) {
    struct tm ti;
    if (getLocalTime(&ti, 10)) {
      timeReady = true;
      Serial.println("Time synced");
    }
  }

  // ── WiFi auto-reconnect ──
  if (WiFi.status() != WL_CONNECTED) {
    static unsigned long lastRecon = 0;
    if (now - lastRecon > 30000) {
      lastRecon = now;
      WiFi.reconnect();
    }
  }

  // ── Fetch OWM weather data ──
  if (WiFi.status() == WL_CONNECTED && !owmFetching) {
    if (now - lastOwm >= OWM_UPDATE_MS || (lastOwm == 0 && !owmNow.valid)) {
      lastOwm = now;
      fetchOwm();
    }
  }

  // ── Read sensors ──
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

  // ── Sparkline sampling ──
  if (now - lastSpark >= SPARK_MS) {
    lastSpark = now;
    if (!isnan(temp)) sparkPush(temp);
  }

  // ── Screen rotation ──
  if (!sliding && now - lastSwitch >= SCREEN_MS) {
    lastSwitch = now;
    prevScr = curScr;
    curScr  = (curScr + 1) % NUM_SCREENS;
    sliding = true;
    slideOff = 0;
  }

  // ── Render ──
  renderFrame();

  delay(sliding ? 25 : 80);
}
