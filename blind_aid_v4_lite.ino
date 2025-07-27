#include <NewPing.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <SoftwareSerial.h>

// HC-SR04 Pins
#define TRIG_PIN 6
#define ECHO_PIN 7
#define MAX_DISTANCE 400 // cm
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_MPU6050 mpu;

// Vibration Motor Pin
#define VIB_PIN 9

// SIM800L Pins
#define SIM_RX 11
#define SIM_TX 10
SoftwareSerial sim800l(SIM_RX, SIM_TX);

// Constants
const float DISTANCE_THRESHOLD = 200.0;
const float MIN_DISTANCE = 2.0;
const float MAX_VIB_STRENGTH = 255.0;
const float MIN_VIB_STRENGTH = 50.0;
const unsigned long SAMPLE_INTERVAL = 100;
const float FREE_FALL_THRESHOLD = 2.0;
const float IMPACT_THRESHOLD = 29.4;
const float TILT_THRESHOLD = 45.0;
const unsigned long FALL_WINDOW = 2000;
const unsigned long SMS_COOLDOWN = 60000;
const char* CAREGIVER_NUMBER = "+256751167441";

// Variables
float ax_offset = 0.0, gx_offset = 0.0;
float lastDistance = 0.0;
unsigned long lastTime = 0;
float wearerSpeed = 0.0;
bool fallDetected = false;
unsigned long lastSMSTime = 0;

void setup() {
  Serial.begin(9600);
  pinMode(VIB_PIN, OUTPUT);

  // Initialize VL53L0X
  if (!lox.begin()) {
    Serial.println(F("VL53L0X init failed"));
  }

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println(F("MPU6050 init failed"));
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  ax_offset = ax_calibrate();
  gx_offset = gx_calibrate();

  // Initialize SIM800L
  sim800l.begin(9600);
  delay(1000);
  if (!initializeSIM800L()) {
    Serial.println(F("SIM800L init failed"));
  }
}

float ax_calibrate() {
  float ax_sum = 0.0;
  for (int i = 0; i < 100; i++) { 
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    ax_sum += a.acceleration.x;
    delay(5);
  }
  return ax_sum / 100.0;
}

float gx_calibrate() {
  float gx_sum = 0.0;
  for (int i = 0; i < 100; i++) { 
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gx_sum += g.gyro.x;
    delay(5);
  }
  return gx_sum / 100.0;
}

bool initializeSIM800L() {
  sim800l.println(F("AT"));
  delay(100);
  if (sim800l.find("OK")) {
    sim800l.println(F("AT+CMGF=1"));
    delay(100);
    return sim800l.find("OK");
  }
  return false;
}

void sendSMS(float distance) {
  if (millis() - lastSMSTime < SMS_COOLDOWN) return;
  char message[50]; 
  snprintf(message, sizeof(message), "Fall! Dist: %.1f cm", distance);
  sim800l.print(F("AT+CMGS=\""));
  sim800l.print(CAREGIVER_NUMBER);
  sim800l.println(F("\""));
  delay(100);
  sim800l.print(message);
  delay(100);
  sim800l.write(26);
  delay(1000);
  if (sim800l.find("OK")) {
    Serial.println(F("SMS sent"));
  }
  lastSMSTime = millis();
}

float getTemperature() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  return temp.temperature;
}

float getWearerSpeed() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float ax = a.acceleration.x - ax_offset;
  static float filtered_ax = 0.0;
  const float alpha = 0.1;
  filtered_ax = alpha * ax + (1 - alpha) * filtered_ax;
  static float velocity = 0.0;
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();

  if (abs(filtered_ax) < 0.1) {
    velocity = 0.0;
  } else if (lastUpdate != 0) {
    float dt = (now - lastUpdate) / 1000.0;
    velocity += filtered_ax * dt;
    velocity = constrain(velocity, -2.0, 2.0);
  }
  lastUpdate = now;
  return velocity;
}

bool detectFall() {
  static unsigned long fallStartTime = 0;
  static bool freeFallDetected = false, impactDetected = false;
  static float total_tilt = 0.0;
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float acc_magnitude = sqrt(sq(a.acceleration.x - ax_offset) + sq(a.acceleration.y) + sq(a.acceleration.z));
  float tilt = abs(g.gyro.x - gx_offset) * (SAMPLE_INTERVAL / 1000.0) * 180.0 / PI;
  total_tilt += tilt;

  if (!freeFallDetected && acc_magnitude < FREE_FALL_THRESHOLD) {
    freeFallDetected = true;
    fallStartTime = millis();
  } else if (freeFallDetected && !impactDetected && acc_magnitude > IMPACT_THRESHOLD) {
    impactDetected = true;
  } else if (freeFallDetected && impactDetected && total_tilt > TILT_THRESHOLD) {
    if (millis() - fallStartTime <= FALL_WINDOW) {
      freeFallDetected = false;
      impactDetected = false;
      total_tilt = 0.0;
      return true;
    }
  }
  if (freeFallDetected && (millis() - fallStartTime > FALL_WINDOW)) {
    freeFallDetected = false;
    impactDetected = false;
    total_tilt = 0.0;
  }
  return false;
}

float getDistance() {
  if (lastDistance <= DISTANCE_THRESHOLD) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) {
      float distance_cm = measure.RangeMilliMeter / 10.0;
      if (distance_cm >= MIN_DISTANCE && distance_cm <= DISTANCE_THRESHOLD) {
        return distance_cm;
      }
    }
  }
  
  unsigned int duration = sonar.ping();
  float temperature = getTemperature();
  float speedOfSound = 331.4 + (0.606 * temperature);
  float distance_cm = (duration * (speedOfSound / 10000)) / 2;
  return (distance_cm >= MIN_DISTANCE && distance_cm <= MAX_DISTANCE) ? distance_cm : -1.0;
}

float calculateObjectSpeed(float currentDistance, float deltaTime) {
  if (lastDistance == 0.0 || deltaTime == 0) return 0.0;
  float distanceChange = lastDistance - currentDistance;
  return (distanceChange / (deltaTime / 1000.0)) / 100.0;
}

void vibrateFeedback(float distance, float relativeSpeed) {
  if (distance < 0) {
    analogWrite(VIB_PIN, 0);
    return;
  }
  float strength = map(distance, MIN_DISTANCE, DISTANCE_THRESHOLD, MAX_VIB_STRENGTH, MIN_VIB_STRENGTH);
  strength = constrain(strength, MIN_VIB_STRENGTH, MAX_VIB_STRENGTH);
  if (abs(relativeSpeed) < 0.1) {
    analogWrite(VIB_PIN, (int)strength);
  } else {
    float absSpeed = abs(relativeSpeed);
    long pulseDelay = map(absSpeed * 100, 0, 200, 500, 50);
    pulseDelay = constrain(pulseDelay, 50, 500);
    analogWrite(VIB_PIN, (int)strength);
    delay(pulseDelay / 2);
    analogWrite(VIB_PIN, 0);
    delay(pulseDelay / 2);
  }
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= SAMPLE_INTERVAL) {
    float distance = getDistance();
    wearerSpeed = getWearerSpeed();
    float deltaTime = currentTime - lastTime;
    float objectSpeed = calculateObjectSpeed(distance, deltaTime);
    float relativeSpeed = objectSpeed - wearerSpeed;
    if (detectFall()) {
      fallDetected = true;
      sendSMS(distance);
    }
    if (!fallDetected) {
      vibrateFeedback(distance, relativeSpeed);
    }
    lastDistance = distance;
    lastTime = currentTime;
  }
}