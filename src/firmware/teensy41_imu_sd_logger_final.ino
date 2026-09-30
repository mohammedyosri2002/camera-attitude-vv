// ============================================================
// teensy41_imu_sd_logger_final.ino
// ------------------------------------------------------------
// Rotating-platform IMU logger. Reads ONLY the ISM330DHCX
// (gyro XYZ, accel XYZ, die temperature) and logs to the
// built-in SDIO microSD. Fully autonomous:
//
//   POWER ON -> init IMU + SD -> wait exactly 20 s (LED blinks)
//   -> LED solid ON -> log forever -> user removes power.
//
// Next power-up creates the next RUNnnnn.CSV (never overwrites).
//
// Power-cut tolerant: data is buffered (RingBuf) and the file is
// synchronized to the card about once per second, so pulling
// power loses at most ~the last second of samples.
//
// LED codes (built-in pin 13):
//   blink 2 Hz  = 20 s startup wait
//   solid ON    = recording
//   fast blink  = FATAL init error (IMU or SD) -- NOT recording
//
// Rate: rows are written when the IMU reports new data
// (ODR 208 Hz), polled continuously -> ~208 rows/s with true
// per-sample 64-bit timestamps. No fake/duplicated samples.
//
// LIBRARIES (Arduino Library Manager):
//   - SdFat (Bill Greiman, >= 2.1; a version ships with
//     Teensyduino and supports SDXC/exFAT -- 256 GB card OK,
//     format it exFAT with the SD Association formatter)
//   - SparkFun 6DoF ISM330DHCX Arduino Library
//
// WIRING (Qwiic / I2C, all 3.3 V):
//   Teensy 3V3 -> breakout 3V3     Teensy GND -> breakout GND
//   Teensy 18 (SDA) -> SDA         Teensy 19 (SCL) -> SCL
//   Power: 5 V battery pack -> Teensy VIN + GND. If USB and VIN
//   are ever connected at the same time, follow the PJRC
//   VUSB/VIN separation guidance (cut pad or diode).
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <SdFat.h>
#include <RingBuf.h>
#include <SparkFun_ISM330DHCX.h>

// ============================================================
// CONFIG
// ============================================================
const uint32_t STARTUP_WAIT_MS = 20000UL;   // exactly 20 s
const uint8_t  PIN_LED = 13;                // built-in LED
const uint8_t  ISM_I2C_ADDR = 0x6B;         // SparkFun default
const uint32_t SD_SYNC_INTERVAL_US = 1000000UL;   // ~1 s
const uint16_t TEMP_EVERY_N_SAMPLES = 256;  // ~0.8 Hz temperature read

// ============================================================
// GLOBALS
// ============================================================
SdFs sd;
FsFile dataFile;
// ~208 rows/s * ~90 B/row = ~19 KB/s. 64 KiB ring buffers ~3 s
// of worst-case SD write latency.
RingBuf<FsFile, 65536> rb;

SparkFun_ISM330DHCX ism;
uint32_t sampleIndex = 0;
char fileName[16];

// ============================================================
// 64-BIT MONOTONIC MICROSECONDS (handles micros() wrap; called
// only from loop context)
// ============================================================
uint64_t micros64() {
  static uint32_t hi = 0;
  static uint32_t last = 0;
  uint32_t now = micros();
  if (now < last) hi++;
  last = now;
  return ((uint64_t)hi << 32) | now;
}

// ============================================================
// ISM330DHCX die temperature, direct register read
// (OUT_TEMP_L/H = 0x20/0x21, 256 LSB/degC, offset 25 degC;
// read directly because the SparkFun library's temperature
// accessor is not present in all versions)
// ============================================================
float ismReadTempC() {
  Wire.beginTransmission(ISM_I2C_ADDR);
  Wire.write(0x20);
  if (Wire.endTransmission(false) != 0) return NAN;
  if (Wire.requestFrom((int)ISM_I2C_ADDR, 2) != 2) return NAN;
  int16_t raw = Wire.read();
  raw |= ((int16_t)Wire.read()) << 8;
  return 25.0f + (float)raw / 256.0f;
}

// ============================================================
// FATAL ERROR: fast blink forever, never pretend to record
// ============================================================
void fatalBlink() {
  while (1) {
    digitalWriteFast(PIN_LED, HIGH); delay(80);
    digitalWriteFast(PIN_LED, LOW);  delay(80);
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);          // optional; not required for a run
  pinMode(PIN_LED, OUTPUT);
  digitalWriteFast(PIN_LED, LOW);

  Wire.begin();
  Wire.setClock(400000);

  // ---- IMU ----
  if (!ism.begin()) {
    if (Serial) Serial.println("FATAL: ISM330DHCX not found");
    fatalBlink();
  }
  ism.deviceReset();
  while (!ism.getDeviceReset()) delay(1);
  ism.setDeviceConfig();
  ism.setBlockDataUpdate();
  ism.setAccelDataRate(ISM_XL_ODR_208Hz);
  ism.setAccelFullScale(ISM_4g);
  ism.setGyroDataRate(ISM_GY_ODR_208Hz);
  ism.setGyroFullScale(ISM_125dps);     // finest resolution; motion <= 30 dps
  ism.setAccelFilterLP2();
  ism.setGyroFilterLP1();

  // ---- SD ----
  if (!sd.begin(SdioConfig(FIFO_SDIO))) {
    if (Serial) Serial.println("FATAL: SD init failed");
    fatalBlink();
  }

  // ---- next RUNnnnn.CSV, never overwrite ----
  bool opened = false;
  for (int n = 1; n <= 9999; n++) {
    snprintf(fileName, sizeof(fileName), "RUN%04d.CSV", n);
    if (!sd.exists(fileName)) {
      opened = dataFile.open(fileName, O_RDWR | O_CREAT | O_TRUNC);
      break;
    }
  }
  if (!opened) {
    if (Serial) Serial.println("FATAL: could not create run file");
    fatalBlink();
  }
  rb.begin(&dataFile);

  // header (config recorded in the first comment line)
  rb.print("# teensy41_imu_sd_logger_final ISM330DHCX ");
  rb.print("odr=208Hz gyro_fs=125dps accel_fs=4g startup_wait_ms=");
  rb.print(STARTUP_WAIT_MS);
  rb.println();
  rb.print("sample_index,teensy_time_us,"
           "gyro_x_dps,gyro_y_dps,gyro_z_dps,"
           "accel_x_g,accel_y_g,accel_z_g,imu_temperature_c");
  rb.println();

  if (Serial) Serial.printf("Logging to %s after 20 s wait\n", fileName);

  // ---- exactly 20 s wait, LED blinking ----
  uint32_t t0 = millis();
  while (millis() - t0 < STARTUP_WAIT_MS) {
    digitalWriteFast(PIN_LED, (millis() / 250) & 1);   // 2 Hz blink
    // drain IMU FIFO/status during the wait so sample 0 is fresh
    if (ism.checkStatus()) {
      sfe_ism_data_t g, a;
      ism.getGyro(&g);
      ism.getAccel(&a);
    }
  }

  digitalWriteFast(PIN_LED, HIGH);   // recording
}

// ============================================================
// LOOP: poll IMU status; log every new sample; sync ~1 s
// ============================================================
uint64_t lastSyncUs = 0;
float tempC = NAN;

void loop() {
  uint64_t now = micros64();

  // ---- new IMU sample? ----
  if (ism.checkStatus()) {
    sfe_ism_data_t g, a;
    ism.getGyro(&g);
    ism.getAccel(&a);

    if ((sampleIndex % TEMP_EVERY_N_SAMPLES) == 0) {
      tempC = ismReadTempC();
    }

    rb.print(sampleIndex++);            rb.print(',');
    rb.print(now);                      rb.print(',');
    rb.print(g.xData / 1000.0f, 5);     rb.print(',');   // mdps -> dps
    rb.print(g.yData / 1000.0f, 5);     rb.print(',');
    rb.print(g.zData / 1000.0f, 5);     rb.print(',');
    rb.print(a.xData / 1000.0f, 5);     rb.print(',');   // mg -> g
    rb.print(a.yData / 1000.0f, 5);     rb.print(',');
    rb.print(a.zData / 1000.0f, 5);     rb.print(',');
    rb.print(tempC, 2);
    rb.println();

    if (rb.getWriteError()) {
      // ring overrun: note it in-stream and continue
      rb.clearWriteError();
      rb.println("# RB_OVERRUN");
    }
  }

  // ---- move buffered bytes to the card ----
  size_t n = rb.bytesUsed();
  if (n >= 512 && !dataFile.isBusy()) {
    rb.writeOut(512);
  }

  // ---- ~1 s: drain buffer fully and sync so power-cut loses
  //      at most the last ~second ----
  if (now - lastSyncUs >= SD_SYNC_INTERVAL_US) {
    while (rb.bytesUsed() > 0) rb.writeOut(512);
    dataFile.sync();
    lastSyncUs = now;
  }
}
