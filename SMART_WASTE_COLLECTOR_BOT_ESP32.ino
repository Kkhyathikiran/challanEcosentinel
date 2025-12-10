
#define BLYNK_TEMPLATE_ID "TMPL3AEMFw0IT"
#define BLYNK_TEMPLATE_NAME "SMART WASTE COLLECTOR"

#include <Wire.h>
#include <Adafruit_PN532.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>



// ---------------- PIN DEFINITIONS ----------------
#define SDA_PIN 21
#define SCL_PIN 22

#define GPS_RX 16   // GPS TX → ESP32 RX2
#define GPS_TX 17   // GPS RX → ESP32 TX2

#define SERVO_PIN 18
#define LASER_PIN 19
#define BUZZER 2
#define RED_LED 25
#define GREEN_LED 26

// Motor driver MX1508
#define IN1 32
#define IN2 33
#define IN3 27
#define IN4 14

// ---------------- OBJECTS ----------------
Adafruit_PN532 nfc(SDA_PIN, SCL_PIN);
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

// ---------------- BLYNK SETUP ----------------
char auth[] = "23apIo5Mjijir1KQu_iFTnySqh59ljkT";    // Replace with actual token
char ssid[] = "Meghana";     // Replace with your WiFi SSID
char pass[] = "meghana123"; // Replace with your WiFi Password

BlynkTimer timer;

// ---------------- SERVO VARIABLES ----------------
const int servoChannel = 5;
const int servoFreq = 50;
const int servoResolution = 16;

// ---------------- SYSTEM FLAGS ----------------
bool gpsConnected = false;
bool gpsAttempted = false;
unsigned long lastGPSTryTime = 0;

// ---------------- FUNCTION DECLARATIONS ----------------
void checkAllSensors();
bool checkGPS(int attempts);
bool checkRFID();
void runMotors(bool run);
void handleError();
void sendToBlynk(String tagID, float lat, float lon);
void beep(int count, int delayMs);
void servoRotate();
void tryReconnectGPS();
void blinkGreenIfNoGPS();

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  pinMode(LASER_PIN, INPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // --- Servo Setup using LEDC PWM ---
  ledcSetup(servoChannel, servoFreq, servoResolution);
  ledcAttachPin(SERVO_PIN, servoChannel);




  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BUZZER, LOW);

  Serial.println("Smart Waste Collector Initializing...");

  Blynk.begin(auth, ssid, pass);

  nfc.begin();
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.println("PN532 not detected!");
    handleError();
  }
  nfc.SAMConfig();

  checkAllSensors();

  // Blink LED every 500ms if GPS not connected
  timer.setInterval(500L, blinkGreenIfNoGPS);

  // Try reconnecting GPS every 15 seconds if not connected
  timer.setInterval(15000L, tryReconnectGPS);

  timer.setInterval(2000L, []() {
    Blynk.run();
  });
}

// ---------------- MAIN LOOP ----------------
void loop() {
  Blynk.run();
  timer.run();

  // Laser beam broken detection
  if (digitalRead(LASER_PIN) == LOW) { // Beam broken
    Serial.println("Laser Beam Broken - Scanning RFID...");

    uint8_t success;
    uint8_t uid[7];
    uint8_t uidLength;

    success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);

    if (success) {
      String tagID = "";
      for (uint8_t i = 0; i < uidLength; i++) {
        tagID += String(uid[i], HEX);
      }
      tagID.toUpperCase();
      Serial.println("Tag ID: " + tagID);

      // Get GPS data
      while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
      }
      float lat = gps.location.lat();
      float lon = gps.location.lng();

      Serial.printf("GPS: %.6f, %.6f\n", lat, lon);

      // Send to Blynk
      sendToBlynk(tagID, lat, lon);

      // Servo rotate
      servoRotate();
      delay(1000);
    }
  }
}

// ---------------- FUNCTION DEFINITIONS ----------------

void checkAllSensors() {
  Serial.println("Checking all sensors...");
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);

  beep(1, 300);
  bool gpsOk = checkGPS(3); // Try GPS 3 times
  bool rfidOk = checkRFID();

  if (rfidOk) {
    if (gpsOk) {
      Serial.println("All Sensors OK (GPS Connected)");
      digitalWrite(RED_LED, LOW);
      digitalWrite(GREEN_LED, HIGH);
      beep(2, 150);
      runMotors(true);
    } else {
      Serial.println("RFID OK but GPS not locked! Continuing...");
      gpsConnected = false;
      digitalWrite(RED_LED, LOW);
      runMotors(true);
    }
  } else {
    handleError();
  }
}

bool checkGPS(int attempts) {
  Serial.println("Checking GPS lock...");
  gpsAttempted = true;
  for (int i = 0; i < attempts; i++) {
    Serial.printf("GPS Attempt %d/%d\n", i + 1, attempts);
    unsigned long start = millis();
    while (millis() - start < 10000) { // Wait 10s each attempt
      while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
        if (gps.location.isValid()) {
          Serial.println("GPS Connected");
          gpsConnected = true;
          return true;
        }
      }
    }
    Serial.println("GPS not connected, retrying...");
  }
  gpsConnected = false;
  return false;
}

bool checkRFID() {
  Serial.println("Checking PN532...");
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (versiondata) {
    Serial.println("RFID OK");
    return true;
  } else {
    Serial.println("RFID Failed");
    return false;
  }
}

void runMotors(bool run) {
  if (run) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("Motors Running...");
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    Serial.println("Motors Stopped.");
  }
}

void handleError() {
  runMotors(false);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, HIGH);
  Serial.println("System Error - Check Sensors!");
  while (true) {
    digitalWrite(BUZZER, HIGH);
    delay(500);
    digitalWrite(BUZZER, LOW);
    delay(500);
  }
}

void sendToBlynk(String tagID, float lat, float lon) {
  Blynk.virtualWrite(V0, tagID);
  Blynk.virtualWrite(V1, lat);
  Blynk.virtualWrite(V2, lon);
  Serial.println("Data sent to Blynk!");
}

void beep(int count, int delayMs) {
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER, HIGH);
    delay(delayMs);
    digitalWrite(BUZZER, LOW);
    delay(delayMs);
  }
}

void servoRotate() {
  int minDuty = 1638;  // ~0°
  int maxDuty = 7864;  // ~160°
  for (int duty = minDuty; duty <= maxDuty; duty += 40) {
    ledcWrite(servoChannel, duty);
    delay(10);
  }
  delay(500);
  for (int duty = maxDuty; duty >= minDuty; duty -= 40) {
    ledcWrite(servoChannel, duty);
    delay(10);
  }
}

// Blink green LED if GPS not connected
void blinkGreenIfNoGPS() {
  static bool ledState = false;
  if (!gpsConnected) {
    ledState = !ledState;
    digitalWrite(GREEN_LED, ledState);
  } else {
    digitalWrite(GREEN_LED, HIGH);
  }
}

// Try to reconnect GPS every 15s
void tryReconnectGPS() {
  if (!gpsConnected) {
    Serial.println("Trying to reconnect GPS...");
    checkGPS(1);
  }
}