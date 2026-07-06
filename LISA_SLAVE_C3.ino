/*
 * LISA (Learning Intelligent Servo-balanced Assistant)
 * Layer 3: High-Frequency Kinematics & Balancing (ESP32-C3)
 * 
 * Hardware Layout:
 * - BMI160 IMU: SDA -> GPIO 4, SCL -> GPIO 5
 * - Servos:
 *   - Left Hip: GPIO 0 (Neutral: 90°)
 *   - Right Hip: GPIO 1 (Neutral: 90°)
 *   - Left Knee: GPIO 6 (Neutral: 180° / Straight)
 *   - Right Knee: GPIO 7 (Neutral: 180° / Straight)
 *   - Left Ankle: GPIO 2 (Range: 45° to 135°, Neutral: 90°)
 *   - Right Ankle: GPIO 3 (Range: 45° to 135°, Neutral: 90°)
 *   - Left Arm: GPIO 8 (Neutral: 90° / Home)
 *   - Right Arm: GPIO 10 (Neutral: 90° / Home)
 * - Master Link: ESP-NOW (Dynamic Channel Scan 1-13)
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <math.h>

// --- Pin configurations ---
#define IMU_SDA_PIN      4
#define IMU_SCL_PIN      5
#define IMU_I2C_ADDR     0x68

#define PIN_LEFT_HIP     0
#define PIN_RIGHT_HIP    1
#define PIN_LEFT_ANKLE   2
#define PIN_RIGHT_ANKLE  3
#define PIN_LEFT_KNEE    6
#define PIN_RIGHT_KNEE   7
#define PIN_LEFT_ARM     8
#define PIN_RIGHT_ARM    10

// PWM Servo constants (50Hz, 14-bit resolution)
#define LEDC_TIMER_14_BIT 14
#define LEDC_BASE_FREQ    50
#define SERVO_MIN_DUTY    410
#define SERVO_MAX_DUTY    2048

// --- Control Loop Timings ---
const double dt = 0.005; // 200Hz = 5ms interval

// --- Sensor Fusion Variables ---
double rollAngle = 0.0;
double fusedRoll = 0.0;
const double alpha = 0.98; // Complementary filter weight

// --- PID Controller Parameters ---
double Kp = 4.0;
double Ki = 0.15;
double Kd = 0.6;
double integralError = 0.0;
double lastError = 0.0;
const double maxIntegral = 15.0; // Anti-windup clamping threshold
const double maxAnkleOffset = 22.0; // Max balance angle adjustment (degrees)

// --- Inverse Kinematics Leg parameters ---
const double LegNominalLength = 100.0; // Hip to ankle distance in mm

struct __attribute__((packed)) MovementPayload {
  char move[16];
  int speed;
  int track_x; // Horizontal tracking offset percentage (-100 to 100)
};

MovementPayload command = {"idle", 90, 0};
volatile unsigned long lastPacketTime = 0;
uint8_t currentChannel = 1;
bool connectionLocked = false;

TaskHandle_t balanceTaskHandle = NULL;

void initBMI160();
void readIMUData();
void initServos();
void writeServoAngle(int pin, int channel, double angle);
void solveLegIK(double x, double y, double &hipAngle, double &kneeAngle, double &ankleAngle);
void onDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len);
void balanceLoopTask(void *parameter);

void setup() {
  Serial.begin(115200);
  delay(1000);

  initServos();

  // Initialize high-speed I2C
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN, 400000);
  initBMI160();

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);
  lastPacketTime = millis();

  // High-priority 200Hz Balancing Loop Thread
  xTaskCreatePinnedToCore(
    balanceLoopTask,
    "BalanceLoop",
    4096,
    NULL,
    configMAX_PRIORITIES - 1,
    &balanceTaskHandle,
    0
  );

  Serial.println("LISA Slave Core Initialized!");
}

void loop() {
  // Channel Sweep Recovery Watchdog
  if (millis() - lastPacketTime > 1500) {
    connectionLocked = false;
    currentChannel++;
    if (currentChannel > 13) currentChannel = 1;

    esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
    Serial.printf("Telemetry lost. Scanning channel: %d...\n", currentChannel);
    delay(100);
  } else {
    if (!connectionLocked) {
      connectionLocked = true;
      Serial.printf("Telemetry locked on channel: %d\n", currentChannel);
    }
  }
}

// 200Hz High-Frequency Control Task
void balanceLoopTask(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(5); // 5ms

  while (true) {
    readIMUData();

    // 1. PID Feedback Loop Calculation
    double rollError = 0.0 - rollAngle;
    
    // Proportional term
    double pTerm = rollError * Kp;
    
    // Integral term (with anti-windup accumulator clamp)
    integralError += rollError * dt;
    integralError = constrain(integralError, -maxIntegral / Ki, maxIntegral / Ki);
    double iTerm = integralError * Ki;
    
    // Derivative term (rate of change)
    double dTerm = ((rollError - lastError) / dt) * Kd;
    lastError = rollError;

    double pidCorrection = pTerm + iTerm + dTerm;
    pidCorrection = constrain(pidCorrection, -maxAnkleOffset, maxAnkleOffset);

    // 2. Trajectory stride & pose calculations
    double leftFootX = 0.0;
    double leftFootY = 0.0;
    double rightFootX = 0.0;
    double rightFootY = 0.0;
    
    double leftArmAngle = 90.0;
    double rightArmAngle = 90.0;

    double leftHipAngle, leftKneeAngle, leftAnkleAngle;
    double rightHipAngle, rightKneeAngle, rightAnkleAngle;

    bool useIK = true;

    if (strcmp(command.move, "walk") == 0) {
      // Walking stride kinematics
      double phase = millis() * 0.005 * command.speed;
      double stride = sin(phase) * 15.0; // Stride sweep in mm
      double stepLift = cos(phase) * 8.0; // Height lift in mm

      leftFootX = stride;
      rightFootX = -stride;

      if (leftFootX > 0) leftFootY = -abs(stepLift);
      else rightFootY = -abs(stepLift);

    } else if (strcmp(command.move, "dance") == 0) {
      // Dynamic dancing sways and squats
      double dancePhase = millis() * 0.006 * command.speed;
      
      double squat = -abs(sin(dancePhase) * 15.0); 
      leftFootY = squat;
      rightFootY = squat;

      double sway = cos(dancePhase * 0.5) * 12.0;
      leftFootX = sway;
      rightFootX = -sway;

      leftArmAngle = 90.0 + sin(dancePhase) * 45.0;
      rightArmAngle = 90.0 - sin(dancePhase) * 45.0;

    } else if (strcmp(command.move, "wave") == 0) {
      // Lean left slightly and wave the left arm
      leftFootX = -5.0; 
      rightFootX = -5.0; 
      
      double wavePhase = millis() * 0.01 * command.speed;
      leftArmAngle = 90.0 + sin(wavePhase) * 45.0;
      rightArmAngle = 90.0;

    } else if (strcmp(command.move, "bow") == 0) {
      // The Respect Bow (static override, bypassing standard standing IK)
      useIK = false;
      leftHipAngle = 60.0;
      rightHipAngle = 120.0;
      leftKneeAngle = 130.0;
      rightKneeAngle = 130.0;
      leftAnkleAngle = 60.0;
      rightAnkleAngle = 120.0;
      
      leftArmAngle = 50.0;
      rightArmAngle = 130.0;

    } else if (strcmp(command.move, "hero") == 0) {
      // Superhero landing pose (static pose override)
      useIK = false;
      // Left leg crouched low
      leftHipAngle = 120.0;
      leftKneeAngle = 90.0;
      leftAnkleAngle = 70.0;
      
      // Right leg extended backward
      rightHipAngle = 50.0;
      rightKneeAngle = 160.0;
      rightAnkleAngle = 130.0;

      // Punch left arm to the floor
      leftArmAngle = 30.0;
      rightArmAngle = 160.0;
    }

    // 3. Solve Leg IK (if not using pose override)
    if (useIK) {
      solveLegIK(leftFootX, leftFootY, leftHipAngle, leftKneeAngle, leftAnkleAngle);
      solveLegIK(rightFootX, rightFootY, rightHipAngle, rightKneeAngle, rightAnkleAngle);

      // Apply horizontal face tracking offset (YAW pivot) to hip servos
      double trackingYaw = (command.track_x / 100.0) * 30.0; // max +/- 30 degrees pivot
      leftHipAngle += trackingYaw;
      rightHipAngle += trackingYaw;
    }

    // 4. Actuate Servos
    writeServoAngle(PIN_LEFT_HIP, 0, leftHipAngle);
    writeServoAngle(PIN_RIGHT_HIP, 1, rightHipAngle);
    writeServoAngle(PIN_LEFT_KNEE, 4, leftKneeAngle);
    writeServoAngle(PIN_RIGHT_KNEE, 5, rightKneeAngle);
    
    writeServoAngle(PIN_LEFT_ARM, 6, leftArmAngle);
    writeServoAngle(PIN_RIGHT_ARM, 7, rightArmAngle);

    // Apply PID balance correction overlay to ankles
    writeServoAngle(PIN_LEFT_ANKLE, 2, leftAnkleAngle - pidCorrection);
    writeServoAngle(PIN_RIGHT_ANKLE, 3, rightAnkleAngle - pidCorrection);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 3-Joint Geometric Inverse Kinematics Solver
void solveLegIK(double x, double y, double &hipAngle, double &kneeAngle, double &ankleAngle) {
  double targetY = LegNominalLength + y;
  double L1 = 50.0; // Thigh
  double L2 = 50.0; // Calf
  
  double D = sqrt(x*x + targetY*targetY);
  D = constrain(D, 20.0, L1 + L2 - 1.0);

  // Law of cosines for knee internal angle
  double cosKnee = (L1*L1 + L2*L2 - D*D) / (2.0 * L1 * L2);
  cosKnee = constrain(cosKnee, -1.0, 1.0);
  double kneeRad = acos(cosKnee);
  double kneeDeg = kneeRad * (180.0 / M_PI);

  // Angle of hip-to-ankle vector relative to vertical
  double beta = atan2(x, targetY);

  // Angle between thigh and hip-to-ankle vector
  double cosHip = (L1*L1 + D*D - L2*L2) / (2.0 * L1 * D);
  cosHip = constrain(cosHip, -1.0, 1.0);
  double hipInternal = acos(cosHip);

  // Hip pitch angle
  double hipPitchDeg = (beta - hipInternal) * (180.0 / M_PI);
  hipAngle = 90.0 + hipPitchDeg;

  // Knee angle (where 180 is straight)
  kneeAngle = kneeDeg;

  // Ankle angle to keep foot parallel to ground
  double kneePitch = 180.0 - kneeDeg;
  ankleAngle = 90.0 - hipPitchDeg + kneePitch;
}

void initServos() {
  ledcSetup(0, LEDC_BASE_FREQ, LEDC_TIMER_14_BIT);
  ledcSetup(1, LEDC_BASE_FREQ, LEDC_TIMER_14_BIT);
  ledcSetup(2, LEDC_BASE_FREQ, LEDC_TIMER_14_BIT);
  ledcSetup(3, LEDC_BASE_FREQ, LEDC_TIMER_14_BIT);
  ledcSetup(4, LEDC_BASE_FREQ, LEDC_TIMER_14_BIT);
  ledcSetup(5, LEDC_BASE_FREQ, LEDC_TIMER_14_BIT);
  ledcSetup(6, LEDC_BASE_FREQ, LEDC_TIMER_14_BIT);
  ledcSetup(7, LEDC_BASE_FREQ, LEDC_TIMER_14_BIT);

  ledcAttachPin(PIN_LEFT_HIP, 0);
  ledcAttachPin(PIN_RIGHT_HIP, 1);
  ledcAttachPin(PIN_LEFT_ANKLE, 2);
  ledcAttachPin(PIN_RIGHT_ANKLE, 3);
  ledcAttachPin(PIN_LEFT_KNEE, 4);
  ledcAttachPin(PIN_RIGHT_KNEE, 5);
  ledcAttachPin(PIN_LEFT_ARM, 6);
  ledcAttachPin(PIN_RIGHT_ARM, 7);

  writeServoAngle(PIN_LEFT_HIP, 0, 90);
  writeServoAngle(PIN_RIGHT_HIP, 1, 90);
  writeServoAngle(PIN_LEFT_ANKLE, 2, 90);
  writeServoAngle(PIN_RIGHT_ANKLE, 3, 90);
  writeServoAngle(PIN_LEFT_KNEE, 4, 180);
  writeServoAngle(PIN_RIGHT_KNEE, 5, 180);
  writeServoAngle(PIN_LEFT_ARM, 6, 90);
  writeServoAngle(PIN_RIGHT_ARM, 7, 90);
}

void writeServoAngle(int pin, int channel, double angle) {
  angle = constrain(angle, 0.0, 180.0);
  uint32_t duty = SERVO_MIN_DUTY + (uint32_t)((angle / 180.0) * (SERVO_MAX_DUTY - SERVO_MIN_DUTY));
  ledcWrite(channel, duty);
}

void initBMI160() {
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(0x7E);
  Wire.write(0x11);
  Wire.endTransmission();
  delay(50);

  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(0x7E);
  Wire.write(0x15);
  Wire.endTransmission();
  delay(50);

  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(0x43);
  Wire.write(0x02);
  Wire.endTransmission();

  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(0x41);
  Wire.write(0x03);
  Wire.endTransmission();
}

void readIMUData() {
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(0x0C);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_I2C_ADDR, 2);
  
  int16_t rawGyroX = 0;
  if (Wire.available() == 2) {
    rawGyroX = (Wire.read() | (Wire.read() << 8));
  }

  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(0x14);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_I2C_ADDR, 2);

  int16_t rawAccelY = 0;
  if (Wire.available() == 2) {
    rawAccelY = (Wire.read() | (Wire.read() << 8));
  }

  double gyroRateX = (double)rawGyroX / 65.5;
  double accY = (double)rawAccelY / 16384.0;
  double accelRoll = accY * 90.0;

  fusedRoll = alpha * (fusedRoll + gyroRateX * dt) + (1.0 - alpha) * accelRoll;
  rollAngle = fusedRoll;
}

void onDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
  if (len == sizeof(MovementPayload)) {
    memcpy(&command, data, sizeof(MovementPayload));
    lastPacketTime = millis();
  }
}
