# SMART-SPIN
Intelligent Wearable IoT Posture Correction System for IC-SIT'2026 Competition
// ============================================================
//   SMART SPINE — IC-SIT'2026
//   Intelligent Wearable IoT Posture Correction System
//   Hardware: ESP32 NodeMCU + 3x MPU6050 IMU Sensors
//   Platform: Blynk IoT Cloud
//   Author:   MOAAZ AHMED ABDLHAMED MOHAMED SALEM — [STEM FAYOUM SCHOOL]
// ============================================================

// ─── LIBRARIES ───────────────────────────────────────────────
#include <Wire.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <ArduinoOTA.h>
#include <math.h>

// ─── BLYNK CREDENTIALS ───────────────────────────────────────
#define BLYNK_TEMPLATE_ID   "YOUR_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "SmartSpine"
#define BLYNK_AUTH_TOKEN    "YOUR_AUTH_TOKEN"

// ─── WIFI CREDENTIALS ────────────────────────────────────────
const char* ssid     = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// ─── VIRTUAL PINS (BLYNK DASHBOARD) ──────────────────────────
#define VPIN_CERVICAL   V1   // Cervical  (neck) angle stream
#define VPIN_THORACIC   V2   // Thoracic  (upper back) angle stream
#define VPIN_LUMBAR     V3   // Lumbar    (lower back) angle stream
#define VPIN_ALERT      V4   // Alert status flag

// ─── I2C PIN ASSIGNMENTS ──────────────────────────────────────
// Primary I2C bus (Cervical + Thoracic sensors)
#define I2C1_SDA  21
#define I2C1_SCL  22

// Secondary I2C bus (Lumbar sensor — separate software bus)
#define I2C2_SDA  16
#define I2C2_SCL  17

// ─── MPU6050 I2C ADDRESSES ────────────────────────────────────
#define MPU_CERVICAL  0x68   // AD0 → GND
#define MPU_THORACIC  0x69   // AD0 → 3.3V
#define MPU_LUMBAR    0x68   // AD0 → open  (on secondary bus)

// ─── HAPTIC FEEDBACK PIN ─────────────────────────────────────
#define HAPTIC_PIN    25     // GPIO25 → vibration motor driver

// ─── POSTURE THRESHOLDS ───────────────────────────────────────
#define HEALTHY_ZONE_MAX      15.0f   // degrees — healthy range upper limit
#define UNHEALTHY_ZONE_MAX    20.0f   // degrees — severe deviation threshold
#define ALERT_TIME_THRESHOLD  5000    // ms — sustained bad posture before alert

// ─── SENSOR SAMPLING ─────────────────────────────────────────
#define SAMPLE_INTERVAL_MS    100     // 10 Hz sensor sampling
#define CLOUD_INTERVAL_MS     1000    // 1 Hz cloud transmission

// ─── MPU6050 REGISTER ADDRESSES ──────────────────────────────
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_ACCEL_XOUT_H  0x3B
#define MPU6050_CONFIG        0x1A
#define MPU6050_SMPLRT_DIV    0x19

// ─── GLOBAL STATE ────────────────────────────────────────────
TwoWire I2C_Bus1 = TwoWire(0);   // Primary I2C bus object
TwoWire I2C_Bus2 = TwoWire(1);   // Secondary I2C bus object

struct SensorData {
  float pitch;
  float roll;
  bool  valid;
};

SensorData cervical, thoracic, lumbar;

// Alert state machine
bool  alertActive       = false;
bool  inBadPosture      = false;
unsigned long badPostureStartTime = 0;

// Timer trackers
unsigned long lastSampleTime = 0;
unsigned long lastCloudTime  = 0;

BlynkTimer blynkTimer;

// ─── FUNCTION PROTOTYPES ─────────────────────────────────────
void    initMPU6050(TwoWire &bus, uint8_t addr);
bool    readAccelerometer(TwoWire &bus, uint8_t addr, float &ax, float &ay, float &az);
SensorData computeAngles(TwoWire &bus, uint8_t addr);
void    checkPostureAndAlert();
void    sendToBlynk();
void    setupOTA();
void    printDebug();

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n╔══════════════════════════════════╗");
  Serial.println("║        SMART SPINE BOOTING       ║");
  Serial.println("╚══════════════════════════════════╝");

  // ── Haptic pin ──
  pinMode(HAPTIC_PIN, OUTPUT);
  digitalWrite(HAPTIC_PIN, LOW);

  // ── I2C Buses ──
  I2C_Bus1.begin(I2C1_SDA, I2C1_SCL, 400000);   // 400 kHz fast mode
  I2C_Bus2.begin(I2C2_SDA, I2C2_SCL, 400000);

  // ── MPU6050 Initialization ──
  initMPU6050(I2C_Bus1, MPU_CERVICAL);
  delay(50);
  initMPU6050(I2C_Bus1, MPU_THORACIC);
  delay(50);
  initMPU6050(I2C_Bus2, MPU_LUMBAR);
  delay(50);
  Serial.println("[SENSORS] All MPU6050 sensors initialized.");

  // ── WiFi & Blynk ──
  Serial.print("[WIFI] Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 30) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WIFI] Connected! IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\n[WIFI] Connection failed — running offline.");
  }

  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect(5000);

  // ── OTA ──
  setupOTA();

  // ── Blynk Timer ──
  blynkTimer.setInterval(CLOUD_INTERVAL_MS, sendToBlynk);

  // ── Startup haptic pulse ──
  digitalWrite(HAPTIC_PIN, HIGH);
  delay(200);
  digitalWrite(HAPTIC_PIN, LOW);

  Serial.println("[SYSTEM] Smart Spine is active.\n");
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  Blynk.run();
  blynkTimer.run();
  ArduinoOTA.handle();

  unsigned long now = millis();

  // ── High-frequency sensor sampling (100ms) ──
  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = now;

    cervical = computeAngles(I2C_Bus1, MPU_CERVICAL);
    thoracic = computeAngles(I2C_Bus1, MPU_THORACIC);
    lumbar   = computeAngles(I2C_Bus2, MPU_LUMBAR);

    checkPostureAndAlert();

    #ifdef DEBUG_SERIAL
    printDebug();
    #endif
  }
}

// ============================================================
//  MPU6050 INITIALIZATION
// ============================================================
void initMPU6050(TwoWire &bus, uint8_t addr) {
  // Wake up device (clear sleep bit)
  bus.beginTransmission(addr);
  bus.write(MPU6050_PWR_MGMT_1);
  bus.write(0x00);  // Wake up, use internal 8MHz oscillator
  bus.endTransmission(true);
  delay(10);

  // Set sample rate divider: 1kHz / (1+9) = 100Hz
  bus.beginTransmission(addr);
  bus.write(MPU6050_SMPLRT_DIV);
  bus.write(0x09);
  bus.endTransmission(true);

  // Set DLPF (Digital Low-Pass Filter) to ~44Hz to reduce noise
  bus.beginTransmission(addr);
  bus.write(MPU6050_CONFIG);
  bus.write(0x03);
  bus.endTransmission(true);

  Serial.printf("[INIT] MPU6050 @ 0x%02X ready.\n", addr);
}

// ============================================================
//  READ RAW ACCELEROMETER DATA
// ============================================================
bool readAccelerometer(TwoWire &bus, uint8_t addr, float &ax, float &ay, float &az) {
  bus.beginTransmission(addr);
  bus.write(MPU6050_ACCEL_XOUT_H);
  if (bus.endTransmission(false) != 0) return false;

  bus.requestFrom(addr, (uint8_t)6, (uint8_t)true);
  if (bus.available() < 6) return false;

  int16_t rawX = (bus.read() << 8) | bus.read();
  int16_t rawY = (bus.read() << 8) | bus.read();
  int16_t rawZ = (bus.read() << 8) | bus.read();

  // Convert to g-force (±2g range → scale factor 16384 LSB/g)
  const float SCALE = 16384.0f;
  ax = rawX / SCALE;
  ay = rawY / SCALE;
  az = rawZ / SCALE;

  return true;
}

// ============================================================
//  COMPUTE PITCH & ROLL ANGLES FROM ACCELEROMETER
// ============================================================
SensorData computeAngles(TwoWire &bus, uint8_t addr) {
  SensorData result = {0.0f, 0.0f, false};
  float ax, ay, az;

  if (!readAccelerometer(bus, addr, ax, ay, az)) {
    Serial.printf("[ERROR] Failed to read sensor @ 0x%02X\n", addr);
    return result;
  }

  // ── Pitch: forward/backward lean (rotation around Y-axis) ──
  // Formula: Pitch = atan2(-Ax, sqrt(Ay² + Az²)) × (180/π)
  float denomPitch = sqrt(ay * ay + az * az);
  if (denomPitch < 1e-6f) denomPitch = 1e-6f;   // guard against division by zero
  result.pitch = atan2(-ax, denomPitch) * (180.0f / M_PI);

  // ── Roll: sideways tilt (rotation around X-axis) ──
  // Formula: Roll = atan2(Ay, Az) × (180/π)
  result.roll = atan2(ay, az) * (180.0f / M_PI);

  // ── Take absolute values for deviation magnitude ──
  result.pitch = fabs(result.pitch);
  result.roll  = fabs(result.roll);

  result.valid = true;
  return result;
}

// ============================================================
//  POSTURE ALERT STATE MACHINE
// ============================================================
void checkPostureAndAlert() {
  if (!cervical.valid || !thoracic.valid || !lumbar.valid) return;

  // Check if ANY sensor exceeds the unhealthy posture threshold
  bool currentlyBad = (cervical.pitch > HEALTHY_ZONE_MAX ||
                       thoracic.pitch > HEALTHY_ZONE_MAX ||
                       lumbar.pitch   > HEALTHY_ZONE_MAX);

  unsigned long now = millis();

  if (currentlyBad) {
    if (!inBadPosture) {
      // Start the bad posture timer
      inBadPosture = true;
      badPostureStartTime = now;
    } else {
      // Check if sustained beyond threshold (5 seconds)
      unsigned long duration = now - badPostureStartTime;
      if (duration >= ALERT_TIME_THRESHOLD && !alertActive) {
        alertActive = true;
        // Activate haptic feedback
        digitalWrite(HAPTIC_PIN, HIGH);
        Serial.println("[ALERT] Poor posture detected! Haptic feedback activated.");
        // Blynk notification
        Blynk.virtualWrite(VPIN_ALERT, 1);
        Blynk.logEvent("poor_posture", "Sustained poor posture detected for 5+ seconds!");
      }
    }
  } else {
    // User returned to healthy posture — reset everything
    if (alertActive || inBadPosture) {
      inBadPosture = false;
      alertActive  = false;
      badPostureStartTime = 0;
      digitalWrite(HAPTIC_PIN, LOW);
      Blynk.virtualWrite(VPIN_ALERT, 0);
      Serial.println("[OK] Good posture restored. Alert cleared.");
    }
  }
}

// ============================================================
//  SEND DATA TO BLYNK CLOUD (1 Hz)
// ============================================================
void sendToBlynk() {
  if (!Blynk.connected()) return;

  if (cervical.valid) {
    Blynk.virtualWrite(VPIN_CERVICAL, cervical.pitch);
  }
  if (thoracic.valid) {
    Blynk.virtualWrite(VPIN_THORACIC, thoracic.pitch);
  }
  if (lumbar.valid) {
    Blynk.virtualWrite(VPIN_LUMBAR, lumbar.pitch);
  }
}

// ============================================================
//  OTA WIRELESS FIRMWARE UPDATES
// ============================================================
void setupOTA() {
  ArduinoOTA.setHostname("SmartSpine-ESP32");
  ArduinoOTA.setPassword("smartspine2026");

  ArduinoOTA.onStart([]() {
    Serial.println("[OTA] Update starting...");
    digitalWrite(HAPTIC_PIN, LOW);   // Safety: disable haptic during update
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\n[OTA] Update complete!");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("[OTA] Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA] Error[%u]: ", error);
    if      (error == OTA_AUTH_ERROR)    Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)   Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)     Serial.println("End Failed");
  });

  ArduinoOTA.begin();
  Serial.println("[OTA] OTA listener active.");
}

// ============================================================
//  DEBUG SERIAL OUTPUT (enable with #define DEBUG_SERIAL)
// ============================================================
void printDebug() {
  Serial.printf(
    "[DATA] Cervical: P=%.1f° R=%.1f° | Thoracic: P=%.1f° R=%.1f° | Lumbar: P=%.1f° R=%.1f° | Alert: %s\n",
    cervical.pitch, cervical.roll,
    thoracic.pitch, thoracic.roll,
    lumbar.pitch,   lumbar.roll,
    alertActive ? "ACTIVE" : "OFF"
  );
}

// ============================================================
//  BLYNK CONNECTED CALLBACK
// ============================================================
BLYNK_CONNECTED() {
  Serial.println("[BLYNK] Connected to cloud server.");
  Blynk.syncAll();
}

// ============================================================
//  END OF SMART SPINE FIRMWARE
// ============================================================
s
