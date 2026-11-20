#include <SoftwareSerial.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define SERIAL_BAUD 115200
#define HOVER_BAUD 115200
#define HOVER_RX_PIN 2
#define HOVER_TX_PIN 3
#define START_FRAME 0xABCD
#define CONTROL_INTERVAL_MS 100
#define TELEMETRY_INTERVAL_MS 500
#define FEEDBACK_TIMEOUT_MS 500
#define COMMAND_TIMEOUT_MS 500
#define WHEEL_DIAMETER 0.165f
#define WHEEL_SEPARATION 0.521f
#define MAX_LINEAR_VELOCITY 2.1f
#define MAX_ANGULAR_VELOCITY 2.49f
#define MAX_WHEEL_VELOCITY 2.10f
#define MAX_LINEAR_ACCEL 0.40f
#define MAX_ANGULAR_ACCEL 1.50f
#define STEER_GAIN 0.0043f
#define SPEED_GAIN 0.0086f
#define MAX_MOTOR_COMMAND 500.0f
#define SPEED_TO_RPM 1.0f
#define LEFT_KP 0.30f
#define LEFT_KI 0.15f
#define LEFT_KD 0.00f
#define RIGHT_KP 0.30f
#define RIGHT_KI 0.15f
#define RIGHT_KD 0.00f
#define PID_INTEGRAL_LIMIT 0.50f
#define PID_OUTPUT_LIMIT 0.50f
#define IMU_Z_SIGN 1.0f

SoftwareSerial HoverSerial(HOVER_RX_PIN, HOVER_TX_PIN);
MPU6050 mpu6050(Wire);

struct __attribute__((packed)) SerialCommand {
  uint16_t start;
  int16_t steer;
  int16_t speed;
  uint16_t checksum;
};

struct __attribute__((packed)) SerialFeedback {
  uint16_t start;
  int16_t cmd1;
  int16_t cmd2;
  int16_t speedR_meas;
  int16_t speedL_meas;
  int16_t batVoltage;
  int16_t boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
};

SerialCommand Command;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

struct WheelPID {
  float kp;
  float ki;
  float kd;
  float integral;
  float previousError;
  float output;
};

WheelPID leftPID = {LEFT_KP, LEFT_KI, LEFT_KD, 0, 0, 0};
WheelPID rightPID = {RIGHT_KP, RIGHT_KI, RIGHT_KD, 0, 0, 0};

uint8_t rxBuffer[sizeof(SerialFeedback)];
uint8_t rxIndex = 0;
char inputBuffer[40];
uint8_t inputIndex = 0;
float targetV = 0.0f;
float targetW = 0.0f;
float currentV = 0.0f;
float currentW = 0.0f;
float targetLeft = 0.0f;
float targetRight = 0.0f;
float measuredLeft = 0.0f;
float measuredRight = 0.0f;
float commandLeft = 0.0f;
float commandRight = 0.0f;
float imuAngleZ = 0.0f;
float imuGyroZ = 0.0f;
unsigned long lastControl = 0;
unsigned long lastTelemetry = 0;
unsigned long lastFeedback = 0;
unsigned long lastCommand = 0;
bool feedbackReceived = false;
bool commandActive = false;

float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

float approach(float current, float target, float maxStep) {
  if (current < target) return fminf(current + maxStep, target);
  if (current > target) return fmaxf(current - maxStep, target);
  return current;
}

void resetPID(WheelPID &pid) {
  pid.integral = 0.0f;
  pid.previousError = 0.0f;
  pid.output = 0.0f;
}

float updatePID(WheelPID &pid, float target, float measured, float dt) {
  float error = target - measured;
  float derivative = 0.0f;
  if (dt > 0.0f) derivative = (error - pid.previousError) / dt;
  pid.integral += error * dt;
  pid.integral = clampFloat(pid.integral, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
  float output = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
  output = clampFloat(output, -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
  pid.previousError = error;
  pid.output = output;
  return output;
}

void sendHoverCommand(int16_t steer, int16_t speed) {
  Command.start = START_FRAME;
  Command.steer = steer;
  Command.speed = speed;
  Command.checksum = Command.start ^ (uint16_t)Command.steer ^ (uint16_t)Command.speed;
  HoverSerial.write((uint8_t*)&Command, sizeof(Command));
}

void driveMotors(float leftMps, float rightMps) {
  float steer = -(leftMps + rightMps) / (2.0f * STEER_GAIN);
  float speed = (leftMps - rightMps) / (2.0f * SPEED_GAIN);
  steer = clampFloat(steer, -MAX_MOTOR_COMMAND, MAX_MOTOR_COMMAND);
  speed = clampFloat(speed, -MAX_MOTOR_COMMAND, MAX_MOTOR_COMMAND);
  sendHoverCommand((int16_t)roundf(steer), (int16_t)roundf(speed));
}

float rawToMps(int16_t raw) {
  return raw * SPEED_TO_RPM * PI * WHEEL_DIAMETER / 60.0f;
}

void receiveFeedback() {
  while (HoverSerial.available()) {
    uint8_t incoming = HoverSerial.read();
    if (rxIndex == 0) {
      if (incoming == 0xCD) rxBuffer[rxIndex++] = incoming;
      continue;
    }
    if (rxIndex == 1) {
      if (incoming == 0xAB) rxBuffer[rxIndex++] = incoming;
      else rxIndex = (incoming == 0xCD) ? 1 : 0;
      continue;
    }
    rxBuffer[rxIndex++] = incoming;
    if (rxIndex == sizeof(SerialFeedback)) {
      memcpy(&NewFeedback, rxBuffer, sizeof(SerialFeedback));
      uint16_t checksum = NewFeedback.start ^ (uint16_t)NewFeedback.cmd1 ^ (uint16_t)NewFeedback.cmd2 ^ (uint16_t)NewFeedback.speedR_meas ^ (uint16_t)NewFeedback.speedL_meas ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed;
      if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
        Feedback = NewFeedback;
        feedbackReceived = true;
        lastFeedback = millis();
        measuredLeft = rawToMps(Feedback.speedR_meas);
        measuredRight = rawToMps(-Feedback.speedL_meas);
      }
      rxIndex = 0;
    }
  }
}

void updateIMU() {
  mpu6050.update();
  imuAngleZ = IMU_Z_SIGN * mpu6050.getAngleZ();
  imuGyroZ = IMU_Z_SIGN * mpu6050.getGyroZ();
}

void computeWheelTargets() {
  targetLeft = currentV - (currentW * WHEEL_SEPARATION / 2.0f);
  targetRight = currentV + (currentW * WHEEL_SEPARATION / 2.0f);
  float maxWheel = fmaxf(fabsf(targetLeft), fabsf(targetRight));
  if (maxWheel > MAX_WHEEL_VELOCITY) {
    float scale = MAX_WHEEL_VELOCITY / maxWheel;
    targetLeft *= scale;
    targetRight *= scale;
  }
}

void stopMotors() {
  commandActive = false;
  targetV = 0.0f;
  targetW = 0.0f;
  currentV = 0.0f;
  currentW = 0.0f;
  targetLeft = 0.0f;
  targetRight = 0.0f;
  commandLeft = 0.0f;
  commandRight = 0.0f;
  resetPID(leftPID);
  resetPID(rightPID);
  sendHoverCommand(0, 0);
}

void checkCommandTimeout() {
  if (commandActive && millis() - lastCommand >= COMMAND_TIMEOUT_MS) stopMotors();
}

void processCommand(char* input) {
  if (strcmp(input, "STOP") == 0 || strcmp(input, "stop") == 0) {
    stopMotors();
    return;
  }
  char* comma = strchr(input, ',');
  if (!comma) return;
  *comma = '\0';
  char* endV;
  char* endW;
  float v = strtod(input, &endV);
  float w = strtod(comma + 1, &endW);
  if (endV == input || *endV != '\0' || endW == comma + 1 || *endW != '\0' || isnan(v) || isnan(w) || isinf(v) || isinf(w)) return;
  if (fabsf(v) > MAX_LINEAR_VELOCITY || fabsf(w) > MAX_ANGULAR_VELOCITY) return;
  targetV = v;
  targetW = w;
  lastCommand = millis();
  commandActive = true;
}

void readSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputIndex > 0) {
        inputBuffer[inputIndex] = '\0';
        processCommand(inputBuffer);
        inputIndex = 0;
      }
    } else if (inputIndex < sizeof(inputBuffer) - 1) {
      inputBuffer[inputIndex++] = c;
    } else {
      inputIndex = 0;
    }
  }
}

void printTelemetry() {
  bool fresh = feedbackReceived && (millis() - lastFeedback <= FEEDBACK_TIMEOUT_MS);
  if (!fresh) return;
  Serial.print(measuredLeft, 4);
  Serial.print(',');
  Serial.print(measuredRight, 4);
  Serial.print(',');
  Serial.print(Feedback.batVoltage / 100.0f, 2);
  Serial.print(',');
  Serial.print(Feedback.boardTemp / 10.0f, 1);
  Serial.print(',');
  Serial.print(imuAngleZ, 3);
  Serial.print(',');
  Serial.println(imuGyroZ, 3);
}


void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);
  HoverSerial.begin(HOVER_BAUD);
  stopMotors();
  Wire.begin();
  Wire.setClock(100000);

  mpu6050.begin();
  Serial.flush();

  mpu6050.calcGyroOffsets(true);
  mpu6050.update();
  lastControl = millis();
  lastTelemetry = millis();
  lastCommand = millis();
}


void loop() {
  unsigned long now = millis();
  checkCommandTimeout();
  readSerialInput();
  receiveFeedback();
  updateIMU();
  if (now - lastControl >= CONTROL_INTERVAL_MS) {
    float dt = (now - lastControl) / 1000.0f;
    lastControl = now;
    currentV = approach(currentV, targetV, MAX_LINEAR_ACCEL * dt);
    currentW = approach(currentW, targetW, MAX_ANGULAR_ACCEL * dt);
    computeWheelTargets();
    bool fresh = feedbackReceived && (now - lastFeedback <= FEEDBACK_TIMEOUT_MS);
    if (fresh && commandActive) {
      float correctionLeft = updatePID(leftPID, targetLeft, measuredLeft, dt);
      float correctionRight = updatePID(rightPID, targetRight, measuredRight, dt);
      commandLeft = clampFloat(targetLeft + correctionLeft, -MAX_WHEEL_VELOCITY, MAX_WHEEL_VELOCITY);
      commandRight = clampFloat(targetRight + correctionRight, -MAX_WHEEL_VELOCITY, MAX_WHEEL_VELOCITY);
    } else {
      resetPID(leftPID);
      resetPID(rightPID);
      commandLeft = 0.0f;
      commandRight = 0.0f;
    }
    if (fabsf(targetLeft) < 0.001f) {
      commandLeft = 0.0f;
      resetPID(leftPID);
    }
    if (fabsf(targetRight) < 0.001f) {
      commandRight = 0.0f;
      resetPID(rightPID);
    }
    driveMotors(commandLeft, commandRight);
  }
  if (now - lastTelemetry >= TELEMETRY_INTERVAL_MS) {
    lastTelemetry = now;
    printTelemetry();
  }
  digitalWrite(LED_BUILTIN, (now % 1000) < 500);
}