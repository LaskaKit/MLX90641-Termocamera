/* MLX90641 Thermal Camera (LaskaKit ESP32-S3 Devkit + Python Visualization)
 *
 * Board:   LaskaKit ESP32-S3-DEVKit                  https://www.laskakit.cz/laskakit-esp32-s3-devkit
 * Sensor:  LaskaKit MLX90641 Thermocamera 16x12      tbd
 *
 * This project uses the **LaskaKit MLX90641 (16×12 thermal sensor)** with LaskaKit ESP32-S3 Devkit**.  
 * The ESP32 reads thermal frames over I²C and streams them as CSV over USB serial.  
 * A Python script on your PC/Mac listens to the serial data and plots a **live heatmap**.
 *
 * Make sure you include (All are prepared in this folder):
 *  - `mlx90641-test.ino` (this sketch)
 *  - `MLX90641_API.cpp/.h` (from [Melexis MLX90641 library: https://github.com/melexis/mlx90641-library/tree/master/)
 *  - `MLX90641_I2C_Driver.cpp/.h` (Arduino Wire-based driver, rewrited by us, ! not the mbed version!)
 *
 * Email:podpora@laskakit.cz
 * Web:laskakit.cz
 */


#define POWER_PIN 47
#define I2C_SDA   42
#define I2C_SCL    2

#include <Arduino.h>
#include <Wire.h>
#include "MLX90641_API.h"
#include "MLX90641_I2C_Driver.h"

// ----------------------- User configuration -----------------------
// Emissivity (paper ~0.95, skin ~0.98, matte plastics ~0.95, shiny metals << 0.95)
static const float EMISSIVITY = 0.95f;

// MLX90641 refresh (0x03=8Hz, 0x04=16Hz, 0x05=32Hz, 0x06=64Hz)
// Start at 8 or 16 Hz while testing. Higher rates need clean wiring.
static const uint8_t MLX_REFRESH = 0x04; // 16 Hz

// I2C speeds: start slower for reliable init; speed up after success.
static const uint32_t I2C_SPEED_INIT_HZ = 100000;  // 100 kHz during init
static const uint32_t I2C_SPEED_RUN_HZ  = 400000;  // 400 kHz after init

// Output modes (set exactly one of them to 1 if you prefer a single format)
// You can enable multiple; they will print in this order each frame.
#define USE_ASCII_HEATMAP 1   // Human-readable ASCII heatmap 16x12
#define USE_CSV_OUTPUT    0   // One CSV line with 192 values (°C with 2 decimals)
#define USE_JSON_OUTPUT   0   // JSON-like array [v0, v1, ..., v191]

// Optional frame averaging to reduce noise. 0 disables averaging.
#define AVERAGE_N         4   // Average N consecutive frames (e.g., 4). Set 0 to disable.

// Optional image orientation fixes
#define FLIP_VERTICAL     0   // 1 to flip top/bottom
#define FLIP_HORIZONTAL   0   // 1 to flip left/right

// Optional I2C scan at boot (handy for debugging)
#define RUN_I2C_SCAN      0

// ------------------------------------------------------------------

static const uint8_t MLX_ADDR = 0x33;    // Default 7-bit I2C address
paramsMLX90641 gParams;                  // Parameter struct for MLX90641
uint16_t gEE[832];                       // EEPROM dump (832 words)
uint16_t gFrame[242];                    // Raw frame (242 words)
float    gTo[16 * 12];                   // Temperatures (°C), 192 values

#if AVERAGE_N > 0
static float gAcc[16 * 12];              // Accumulator for averaging
static int   gAccCount = 0;
#endif

// ----------------------- Helpers -----------------------
static void i2cScan() {
  Serial.println("I2C scan:");
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; ++a) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  found 0x%02X\n", a);
      ++found;
    }
  }
  Serial.printf("Found %u device(s)\n", found);
}

static void flipIfNeeded(float* img, int w, int h) {
#if FLIP_VERTICAL
  for (int r = 0; r < h / 2; ++r) {
    for (int c = 0; c < w; ++c) {
      float tmp = img[r * w + c];
      img[r * w + c] = img[(h - 1 - r) * w + c];
      img[(h - 1 - r) * w + c] = tmp;
    }
  }
#endif
#if FLIP_HORIZONTAL
  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < w / 2; ++c) {
      float tmp = img[r * w + c];
      img[r * w + c] = img[r * w + (w - 1 - c)];
      img[r * w + (w - 1 - c)] = tmp;
    }
  }
#endif
}

static void printStats(const float* img, int count) {
  float tMin = 1e9f, tMax = -1e9f, sum = 0.f;
  for (int i = 0; i < count; ++i) {
    float v = img[i];
    if (v < tMin) tMin = v;
    if (v > tMax) tMax = v;
    sum += v;
  }
  float avg = sum / count;
  Serial.printf("MLX90641 16x12  min=%.2f°C  max=%.2f°C  avg=%.2f°C\n", tMin, tMax, avg);
}

#if USE_ASCII_HEATMAP
static void printAsciiHeatmap(const float* img, int w, int h) {
  // Compute min/max for mapping to ASCII ramp
  float tMin =  1e9f, tMax = -1e9f;
  for (int i = 0; i < w*h; ++i) { 
    if (img[i] < tMin) tMin = img[i];
    if (img[i] > tMax) tMax = img[i];
  }
  const char* ramp = " .:-=+*#%@"; // 10 levels
  float range = max(0.1f, tMax - tMin);

  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < w; ++c) {
      float t = img[r * w + c];
      int level = (int)((t - tMin) / range * 9.0f + 0.5f);
      level = constrain(level, 0, 9);
      Serial.print(ramp[level]);
    }
    Serial.println();
  }
}
#endif

#if USE_CSV_OUTPUT
static void printCSV(const float* img, int count) {
  // Single CSV line with count values (two decimals)
  for (int i = 0; i < count; ++i) {
    Serial.print(img[i], 2);
    if (i < count - 1) Serial.print(',');
  }
  Serial.println();
}
#endif

#if USE_JSON_OUTPUT
static void printJSON(const float* img, int count) {
  Serial.print('[');
  for (int i = 0; i < count; ++i) {
    if (i) Serial.print(',');
    Serial.print(img[i], 2);
  }
  Serial.println(']');
}
#endif

// ----------------------- Setup/Loop -----------------------
void setup() {
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, HIGH);       // For production, consider powering MLX from 3V3 rail.
  delay(400);                          // Give sensor time after power-up

  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 2000)) { delay(10); } // Don't block forever

  // Start I2C slow and with a reasonable timeout for DumpEE
  Wire.begin(I2C_SDA, I2C_SCL, I2C_SPEED_INIT_HZ);
  Wire.setTimeOut(500);

#if RUN_I2C_SCAN
  i2cScan(); // Expect to see 0x33
#endif

  // --- Robust initialization sequence ---
  const int MAX_RETRY = 5;
  int st = -1;

  // Dump EEPROM (832 words). Our I2CRead() reads in chunks to avoid Wire RX buffer overflow.
  for (int i = 0; i < MAX_RETRY; ++i) {
    delay(60);
    st = MLX90641_DumpEE(MLX_ADDR, gEE);
    if (st == 0) break;
    Serial.printf("DumpEE try %d failed: %d\n", i + 1, st);
  }
  if (st != 0) {
    Serial.printf("DumpEE final fail: %d\n", st);
    while (1) delay(10);
  }

  // Extract calibration parameters for MLX90641
  st = MLX90641_ExtractParameters(gEE, &gParams);
  if (st != 0) {
    Serial.printf("ExtractParameters fail: %d\n", st);
    while (1) delay(10);
  }

  // Set refresh (MLX90641 has no Chess/Interleaved mode switch)
  MLX90641_SetRefreshRate(MLX_ADDR, MLX_REFRESH);

  // Optional: read Control Register 1 (0x800D) for info/debug
  uint16_t ctrl1 = 0;
  if (MLX90641_I2CRead(MLX_ADDR, 0x800D, 1, &ctrl1) == 0) {
    Serial.printf("CTRL1=0x%04X\n", ctrl1);
  }

  // After successful init: speed up I2C for frame reads
  Wire.setClock(I2C_SPEED_RUN_HZ);
  Serial.println("Init OK.");
}

void loop() {
  // Grab one raw frame (242 words)
  int st = MLX90641_GetFrameData(MLX_ADDR, gFrame);
  if (st < 0) {
    Serial.printf("GetFrameData error=%d\n", st);
    delay(60);
    return;
  }

  // Compute ambient temperature (Ta) and then per-pixel object temps (To, °C)
  float Ta = MLX90641_GetTa(gFrame, &gParams);
  MLX90641_CalculateTo(gFrame, &gParams, EMISSIVITY, Ta, gTo);

  // Optional image orientation fixes
  flipIfNeeded(gTo, 16, 12);

#if AVERAGE_N > 0
  // Simple running average over AVERAGE_N frames to reduce noise
  for (int i = 0; i < 16 * 12; ++i) {
    // Running average: acc = (acc*n + new)/(n+1)
    gAcc[i] = (gAcc[i] * gAccCount + gTo[i]) / (gAccCount + 1);
  }
  gAccCount++;

  if (gAccCount >= AVERAGE_N) {
    printStats(gAcc, 16 * 12);

  #if USE_ASCII_HEATMAP
    printAsciiHeatmap(gAcc, 16, 12);
  #endif
  #if USE_CSV_OUTPUT
    printCSV(gAcc, 16 * 12);
  #endif
  #if USE_JSON_OUTPUT
    printJSON(gAcc, 16 * 12);
  #endif

    // Reset accumulator
    memset(gAcc, 0, sizeof(gAcc));
    gAccCount = 0;
  }
#else
  // No averaging: print current frame directly
  printStats(gTo, 16 * 12);

#if USE_ASCII_HEATMAP
  printAsciiHeatmap(gTo, 16, 12);
#endif
#if USE_CSV_OUTPUT
  printCSV(gTo, 16 * 12);
#endif
#if USE_JSON_OUTPUT
  printJSON(gTo, 16 * 12);
#endif

#endif // AVERAGE_N

  // Delay according to refresh. For 16 Hz, ~62.5 ms; we keep ~150 ms to be gentle.
  delay(150);
}