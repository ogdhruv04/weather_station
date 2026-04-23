#line 1 "/home/dhruv/weather_station/src_build/src_build.ino"
#include "esp32-hal-gpio.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <Adafruit_BMP085.h>

#define DHTPIN      4
#define DHTTYPE     DHT22
#define SDA_PIN     21
#define SCL_PIN     22
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_RESET    -1
#define OLED_ADDRESS  0x3C

DHT              dht(DHTPIN, DHTTYPE);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_BMP085  bmp;

bool bmpFound   = false;
int  screen     = 0;
unsigned long lastSwitch   = 0;
unsigned long lastRead     = 0;
unsigned long startTime    = 0;
const long SCREEN_INTERVAL = 4000;  // rotate every 4s
const long READ_INTERVAL   = 2000;  // read sensors every 2s

// Cached sensor values
float temp = 0, hum = 0, pressure = 0;
float heatIdx = 0, dewPoint = 0, altitude = 0;

// ── Helpers ──────────────────────────────────

// Dew point via Magnus formula
#line 37 "/home/dhruv/weather_station/src_build/src_build.ino"
float computeDewPoint(float t, float h);
#line 44 "/home/dhruv/weather_station/src_build/src_build.ino"
String comfortLabel(float t, float h);
#line 53 "/home/dhruv/weather_station/src_build/src_build.ino"
void drawBar(int x, int y, int w, int h, float value, float minV, float maxV);
#line 60 "/home/dhruv/weather_station/src_build/src_build.ino"
void drawChrome(const char* title, int screenNum, int total);
#line 79 "/home/dhruv/weather_station/src_build/src_build.ino"
void screenMain();
#line 105 "/home/dhruv/weather_station/src_build/src_build.ino"
void screenFeels();
#line 129 "/home/dhruv/weather_station/src_build/src_build.ino"
void screenPressure();
#line 163 "/home/dhruv/weather_station/src_build/src_build.ino"
void screenSystem();
#line 192 "/home/dhruv/weather_station/src_build/src_build.ino"
void setup();
#line 227 "/home/dhruv/weather_station/src_build/src_build.ino"
void loop();
#line 37 "/home/dhruv/weather_station/src_build/src_build.ino"
float computeDewPoint(float t, float h) {
  float a = 17.271, b = 237.7;
  float gamma = (a * t / (b + t)) + log(h / 100.0);
  return (b * gamma) / (a - gamma);
}

// Comfort label
String comfortLabel(float t, float h) {
  if (t > 35 && h > 60) return "Hot & Humid";
  if (t > 35)            return "Too Hot";
  if (t < 10)            return "Too Cold";
  if (h > 70)            return "Too Humid";
  if (h < 30)            return "Too Dry";
  return "Comfortable";
}
// Draw a horizontal bar (0–100 scale)
void drawBar(int x, int y, int w, int h, float value, float minV, float maxV) {
  int filled = map(constrain(value, minV, maxV), minV, maxV, 0, w);
  display.drawRect(x, y, w, h, SSD1306_WHITE);
  display.fillRect(x, y, filled, h, SSD1306_WHITE);
}

// Header + footer shared across screens
void drawChrome(const char* title, int screenNum, int total) {
  // Title bar
  display.fillRect(0, 0, 128, 10, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(2, 1);
  display.print(title);

  // Dot indicators (bottom right)
  display.setTextColor(SSD1306_WHITE);
  int dotX = 128 - (total * 8);
  for (int i = 0; i < total; i++) {
    if (i == screenNum)
      display.fillCircle(dotX + i * 8 + 3, 60, 2, SSD1306_WHITE);
    else
      display.drawCircle(dotX + i * 8 + 3, 60, 2, SSD1306_WHITE);
  }
}
// ── Screen 1: Main Readings ───────────────────
void screenMain() {
  drawChrome("~~ WEATHER STATION ~~", 0, 4);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 13);
  display.print("Temp:");
  display.setTextSize(2);
  display.setCursor(40, 11);
  display.print(temp, 1);
  display.print("C");

  display.setTextSize(1);
  display.setCursor(0, 30);
  display.print("Humidity:  ");
  display.print(hum, 1);
  display.print("%");
  drawBar(0, 39, 90, 5, hum, 0, 100);

  display.setCursor(0, 47);
  display.print("Pressure: ");
  if (bmpFound) { display.print(pressure, 1); display.print(" hPa"); }
  else display.print("N/A");
}

// ── Screen 2: Feels Like + Dew Point ─────────
void screenFeels() {
  drawChrome("  THERMAL COMFORT   ", 1, 4);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 13);
  display.print("Feels Like:");
  display.setTextSize(2);
  display.setCursor(0, 23);
  display.print(heatIdx, 1);
  display.print("C");

  display.setTextSize(1);
  display.setCursor(0, 42);
  display.print("Dew Point: ");
  display.print(dewPoint, 1);
  display.print(" C");

  display.setCursor(0, 52);
  String c = comfortLabel(temp, hum);
  display.print("> "); display.print(c);
}

// ── Screen 3: BMP180 Detail ───────────────────
void screenPressure() {
  drawChrome("  PRESSURE & ALT    ", 2, 4);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  if (bmpFound) {
    display.setCursor(0, 13);
    display.print("Pressure:");
    display.setTextSize(2);
    display.setCursor(0, 22);
    display.print(pressure, 1);
    display.setTextSize(1);
    display.print(" hPa");

    // Pressure bar (900–1050 hPa typical range)
    drawBar(0, 36, 100, 5, pressure, 900, 1050);

    // Pressure level label
    display.setCursor(104, 34);
    if      (pressure < 1009) display.print("Low");
    else if (pressure < 1022) display.print("Norm");
    else                      display.print("High");

    display.setCursor(0, 45);
    display.print("Altitude: ");
    display.print(altitude, 0);
    display.print(" m");
  } else {
    display.setCursor(10, 25);
    display.print("BMP180 not found");
  }
}

// ── Screen 4: System Uptime ───────────────────
void screenSystem() {
  drawChrome("   SYSTEM STATUS    ", 3, 4);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  unsigned long secs  = (millis() - startTime) / 1000;
  unsigned long mins  = secs / 60;
  unsigned long hrs   = mins / 60;
  secs %= 60; mins %= 60;

  display.setCursor(0, 13);
  display.print("Uptime:");
  display.setTextSize(2);
  display.setCursor(0, 23);
  char buf[12];
  sprintf(buf, "%02lu:%02lu:%02lu", hrs, mins, secs);
  display.print(buf);

  display.setTextSize(1);
  display.setCursor(0, 42);
  display.print("DHT22  : ");
  display.print(isnan(temp) ? "ERROR" : "OK");

  display.setCursor(0, 51);
  display.print("BMP180 : ");
  display.print(bmpFound   ? "OK"    : "N/A");
}

// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  dht.begin();
  startTime = millis();

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("SSD1306 not found!");
    while (1);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  if (bmp.begin()) {
    bmpFound = true;
    Serial.println("BMP180 OK");
  }

  // Splash
  display.fillRect(0, 0, 128, 10, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setCursor(5, 1);
  display.print("ESP32 WEATHER STATION");
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(20, 20);  display.print("DHT22  Temperature");
  display.setCursor(20, 30);  display.print("DHT22  Humidity");
  display.setCursor(20, 40);  display.print("BMP180 Pressure");
  display.setCursor(20, 50);  display.print("SSD1306 Display");
  display.display();
  delay(3000);
}

// ─────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // Read sensors every 2s
  if (now - lastRead >= READ_INTERVAL) {
    lastRead = now;
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t) && !isnan(h)) {
      temp     = t;
      hum      = h;
      heatIdx  = dht.computeHeatIndex(temp, hum, false);
      dewPoint = computeDewPoint(temp, hum);
    }
    if (bmpFound) {
      pressure = bmp.readPressure() / 100.0F;
      altitude = bmp.readAltitude();
    }
    Serial.printf("[%lu] T:%.1fC H:%.1f%% P:%.1fhPa HI:%.1f DP:%.1f Alt:%.0fm\n",
                  now/1000, temp, hum, pressure, heatIdx, dewPoint, altitude);
  }

  // Rotate screen every 4s
  if (now - lastSwitch >= SCREEN_INTERVAL) {
    lastSwitch = now;
    screen = (screen + 1) % 4;
  }

  display.clearDisplay();
  switch (screen) {
    case 0: screenMain();     break;
    case 1: screenFeels();    break;
    case 2: screenPressure(); break;
    case 3: screenSystem();   break;
  }
  display.display();
  delay(100);
}
