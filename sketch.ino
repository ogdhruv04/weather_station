#include "esp32-hal-gpio.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <Adafruit_BMP085.h>
#include <WiFi.h>
#include <time.h>

// ── Configuration ──────────────────────────────
#define WIFI_SSID       "pitest"
#define WIFI_PASS       "12345678"
#define NTP_SERVER      "pool.ntp.org"
#define GMT_OFFSET_SEC  19800          // IST  (UTC+5:30)
#define DST_OFFSET_SEC  0

#define DHTPIN          4
#define DHTTYPE         DHT22
#define SDA_PIN         21
#define SCL_PIN         22
#define SCREEN_W        128
#define SCREEN_H        64
#define OLED_RESET      -1
#define OLED_ADDR       0x3C

#define NUM_SCREENS     5
#define SCREEN_MS       5000           // rotate every 5 s
#define READ_MS         2000           // sensor poll
#define SPARK_CAP       60             // sparkline samples
#define SPARK_MS        10000          // sample every 10 s

// ── Objects ────────────────────────────────────
DHT              dht(DHTPIN, DHTTYPE);
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, OLED_RESET);
Adafruit_BMP085  bmp;

// ── State ──────────────────────────────────────
bool  bmpFound   = false;
bool  timeReady  = false;
int   curScr     = 0;
int   prevScr    = -1;

unsigned long lastSwitch = 0, lastRead = 0, startTime = 0, lastSpark = 0;

// Transition
bool  sliding     = false;
int   slideOff    = 0;
const int SLIDE_STEP  = 32;           // px per frame

// Sensors
float temp = 0, hum = 0, pressure = 0;
float heatIdx = 0, dewPt = 0, alt = 0;

// Sparkline ring-buffer
float sparkBuf[SPARK_CAP];
int   sparkHead = 0, sparkN = 0;

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

void drawWeatherIcon(int x, int y) {
  if (temp < 10)      drawSnow(x + 8, y + 7, 6);
  else if (hum > 70)  drawDrop(x + 8, y + 8);
  else if (temp > 30) drawSun(x + 8, y + 7, 4);
  else                drawCloud(x, y);
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

  // Min/max labels
  char lb[6];
  dtostrf(mn, 3, 0, lb);
  display.setCursor(x + w + 2, y + h - 8);
  display.print(lb);
  dtostrf(mx, 3, 0, lb);
  display.setCursor(x + w + 2, y);
  display.print(lb);
}

// ════════════════════════════════════════════════
//  CHROME  (header with live clock + page dots)
// ════════════════════════════════════════════════

void drawChrome(const char* title, int idx, int xo) {
  // ── inverted header bar ──
  display.fillRect(xo, 0, 128, 11, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(xo + 2, 2);
  display.print(title);

  // live clock (right side of header)
  if (timeReady) {
    struct tm ti;
    if (getLocalTime(&ti, 10)) {
      char c[6];
      sprintf(c, "%02d:%02d", ti.tm_hour, ti.tm_min);
      display.setCursor(xo + 98, 2);
      display.print(c);
    }
  }

  // ── centred page dots ──
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
    // Large HH:MM with blinking colon
    display.setTextSize(3);
    bool colon = (millis() / 500) % 2;
    char tb[6];
    sprintf(tb, "%02d%c%02d", ti.tm_hour, colon ? ':' : ' ', ti.tm_min);
    display.setCursor(xo + 4, 15);
    display.print(tb);

    // Seconds
    display.setTextSize(2);
    char sb[3];
    sprintf(sb, "%02d", ti.tm_sec);
    display.setCursor(xo + 98, 19);
    display.print(sb);

    // Date
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

// 1 ── Main Weather ─────────────────────────────
void scrWeather(int xo) {
  drawChrome("WEATHER", 1, xo);
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

// 3 ── Pressure & Altitude ──────────────────────
void scrPressure(int xo) {
  drawChrome("PRESSURE", 3, xo);
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

// 4 ── System + Sparkline ───────────────────────
void scrSystem(int xo) {
  drawChrome("SYSTEM", 4, xo);
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
  display.setCursor(xo + 78, 24);
  display.print("BMP:");
  display.print(bmpFound ? "OK" : "--");

  display.setCursor(xo, 35);
  display.print("Temp trend");
  drawSparkline(xo, 44, 104, 14);
}

// ════════════════════════════════════════════════
//  SCREEN DISPATCH + TRANSITION ENGINE
// ════════════════════════════════════════════════

typedef void (*ScrFn)(int);
ScrFn scrTable[] = { scrClock, scrWeather, scrComfort, scrPressure, scrSystem };

void renderFrame() {
  display.clearDisplay();

  if (sliding && prevScr >= 0) {
    scrTable[prevScr](-slideOff);           // old slides left
    scrTable[curScr](128 - slideOff);       // new slides in from right
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

  // 3. Sensor status — appears line by line
  display.setCursor(12, 32);
  display.print("DHT22  : OK");
  display.display(); delay(250);

  display.setCursor(12, 42);
  display.print("BMP180 : ");
  display.print(bmpFound ? "OK" : "N/A");
  display.display(); delay(250);

  // 4. WiFi — connect with animated progress bar
  display.setCursor(12, 52);
  display.print("WiFi   : ");
  display.display();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int barX = 12, barW = 104, barY = 58, barH = 4;
  int maxAttempts = 20;                    // ~10 s timeout

  for (int a = 0; a < maxAttempts; a++) {
    // Grow progress bar
    int fill = (int)((float)(a + 1) / maxAttempts * barW);
    display.fillRect(barX, barY, fill, barH, SSD1306_WHITE);
    display.display();

    if (WiFi.status() == WL_CONNECTED) {
      // Fill bar fully & show success
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

  // Brief flash effect
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

  // Animated boot (also connects WiFi)
  bootSplash();

  // Start NTP
  if (WiFi.status() == WL_CONNECTED) {
    configTime(GMT_OFFSET_SEC, DST_OFFSET_SEC, NTP_SERVER);
    Serial.println("NTP configured");
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

  delay(sliding ? 25 : 80);               // faster during transition
}
