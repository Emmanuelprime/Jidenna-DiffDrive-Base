
#include <SoftwareSerial.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// =====================================
// CONFIGURATION
// =====================================
#define HOVER_BAUD       115200
#define SERIAL_BAUD      115200
#define START_FRAME      0xABCD

#define SEND_INTERVAL    100
#define PRINT_INTERVAL   500

// Velocity limits
#define MAX_WHEEL_SPEED  2.1f

// Acceleration limit (m/s^2)
#define MAX_ACCEL        0.40f

// Maximum raw firmware command
#define MAX_COMMAND      500.0f

// Experimental mixing gains
#define STEER_GAIN       0.0043f
#define SPEED_GAIN       0.0086f

// Feedback conversion
#define WHEEL_DIAMETER   0.165f
#define SPEED_TO_RPM     1.0f

SoftwareSerial HoverSerial(2, 3);

// =====================================
// SERIAL STRUCTURES
// =====================================
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

// =====================================
// GLOBAL VARIABLES
// =====================================
uint8_t rxBuffer[sizeof(SerialFeedback)];
uint8_t rxIndex = 0;

char inputBuffer[40];
uint8_t inputIndex = 0;

float targetLeft = 0.0f;
float targetRight = 0.0f;

float currentLeft = 0.0f;
float currentRight = 0.0f;

unsigned long lastSend = 0;
unsigned long lastPrint = 0;
unsigned long lastFeedback = 0;

bool feedbackReceived = false;

// =====================================
// UTILITIES
// =====================================
float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

float approach(float current, float target, float step) {
  if (current < target)
    return fminf(current + step, target);

  if (current > target)
    return fmaxf(current - step, target);

  return current;
}

// =====================================
// SEND HOVERBOARD COMMAND
// =====================================
void sendHoverCommand(int16_t steer, int16_t speed) {

  Command.start = START_FRAME;
  Command.steer = steer;
  Command.speed = speed;

  Command.checksum =
      Command.start ^
      (uint16_t)Command.steer ^
      (uint16_t)Command.speed;

  HoverSerial.write(
    (uint8_t*)&Command,
    sizeof(Command)
  );
}

// =====================================
// CORRECTED DIFFERENTIAL DRIVE
// =====================================
void driveMotors(float leftMps, float rightMps) {

  // Empirical motor model:
  //
  // vL = SPEED_GAIN * speed
  //      - STEER_GAIN * steer
  //
  // vR = -SPEED_GAIN * speed
  //      - STEER_GAIN * steer

  // Inverse mixing equations
  float steer =
    -(leftMps + rightMps) /
    (2.0f * STEER_GAIN);

  float speed =
    (leftMps - rightMps) /
    (2.0f * SPEED_GAIN);

  // Protect firmware command limits
  steer = clampFloat(
    steer, -MAX_COMMAND, MAX_COMMAND
  );

  speed = clampFloat(
    speed, -MAX_COMMAND, MAX_COMMAND
  );

  sendHoverCommand(
    (int16_t)roundf(steer),
    (int16_t)roundf(speed)
  );
}

// =====================================
// RECEIVE FEEDBACK
// =====================================
void receiveFeedback() {

  while (HoverSerial.available()) {

    uint8_t incoming = HoverSerial.read();

    if (rxIndex == 0) {

      if (incoming == 0xCD)
        rxBuffer[rxIndex++] = incoming;

      continue;
    }

    if (rxIndex == 1) {

      if (incoming == 0xAB) {
        rxBuffer[rxIndex++] = incoming;
      } else {
        rxIndex = (incoming == 0xCD) ? 1 : 0;
      }

      continue;
    }

    rxBuffer[rxIndex++] = incoming;

    if (rxIndex == sizeof(SerialFeedback)) {

      memcpy(
        &NewFeedback,
        rxBuffer,
        sizeof(SerialFeedback)
      );

      uint16_t checksum =
          NewFeedback.start ^
          (uint16_t)NewFeedback.cmd1 ^
          (uint16_t)NewFeedback.cmd2 ^
          (uint16_t)NewFeedback.speedR_meas ^
          (uint16_t)NewFeedback.speedL_meas ^
          NewFeedback.batVoltage ^
          NewFeedback.boardTemp ^
          NewFeedback.cmdLed;

      if (NewFeedback.start == START_FRAME &&
          checksum == NewFeedback.checksum) {

        Feedback = NewFeedback;
        feedbackReceived = true;
        lastFeedback = millis();
      }

      rxIndex = 0;
    }
  }
}

// =====================================
// RAW FEEDBACK TO M/S
// =====================================
float rawToMps(int16_t raw) {

  float rpm = raw * SPEED_TO_RPM;

  return rpm * PI * WHEEL_DIAMETER / 60.0f;
}

// =====================================
// PRINT CSV TELEMETRY
// =====================================
void printCSV() {

  bool fresh =
    feedbackReceived &&
    (millis() - lastFeedback <= 1000);

  Serial.print(targetLeft, 3);
  Serial.print(',');

  Serial.print(targetRight, 3);
  Serial.print(',');

  if (fresh) {

    // Physical LEFT = Firmware RIGHT
    float leftMeasured =
      rawToMps(Feedback.speedR_meas);

    // Physical RIGHT = Firmware LEFT
    // Reverse sign for physical forward
    float rightMeasured =
      rawToMps(-Feedback.speedL_meas);

    Serial.print(leftMeasured, 4);
    Serial.print(',');

    Serial.print(rightMeasured, 4);
    Serial.print(',');

    Serial.print(
      Feedback.batVoltage / 100.0f, 2
    );
    Serial.print(',');

    Serial.print(
      Feedback.boardTemp / 10.0f, 1
    );

  } else {

    Serial.print(F("nan,nan,nan,nan"));
  }

  Serial.print(',');
  Serial.println(fresh ? 1 : 0);
}

// =====================================
// PROCESS SERIAL COMMAND
// =====================================
void processCommand(char* input) {

  if (strcmp(input, "STOP") == 0 ||
      strcmp(input, "stop") == 0) {

    targetLeft = 0;
    targetRight = 0;

    currentLeft = 0;
    currentRight = 0;

    sendHoverCommand(0, 0);

    Serial.println(F("# STOP"));
    return;
  }

  char* comma = strchr(input, ',');

  if (!comma) {
    Serial.println(F("# Use left,right"));
    return;
  }

  *comma = '\0';

  char* endLeft;
  char* endRight;

  float left = strtod(input, &endLeft);
  float right = strtod(comma + 1, &endRight);

  if (endLeft == input ||
      *endLeft != '\0' ||
      endRight == comma + 1 ||
      *endRight != '\0' ||
      isnan(left) ||
      isnan(right) ||
      isinf(left) ||
      isinf(right)) {

    Serial.println(F("# Invalid input"));
    return;
  }

  if (fabsf(left) > MAX_WHEEL_SPEED ||
      fabsf(right) > MAX_WHEEL_SPEED) {

    Serial.println(F("# Speed limit +/-0.65 m/s"));
    return;
  }

  targetLeft = left;
  targetRight = right;

  Serial.println(F("# OK"));
}

// =====================================
// READ SERIAL INPUT
// =====================================
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
      Serial.println(F("# Input too long"));
    }
  }
}

// =====================================
// SETUP
// =====================================
void setup() {

  Serial.begin(SERIAL_BAUD);
  HoverSerial.begin(HOVER_BAUD);

  pinMode(LED_BUILTIN, OUTPUT);

  // Start with motors stopped
  sendHoverCommand(0, 0);

  Serial.println(F("# JIDENNA MOTOR CONTROLLER"));
  Serial.println(F("# Independent wheel velocity control"));
  Serial.println(F("# Enter: left,right in m/s"));
  Serial.println(F("# Example: 0.2,0.2"));
  Serial.println(F("# STOP to stop motors"));
  Serial.println(F("# No command timeout"));
  Serial.println(F("target_left,target_right,left,right,bat,temp,feedback_ok"));

  lastSend = millis();
  lastPrint = millis();
}

// =====================================
// MAIN LOOP
// =====================================
void loop() {

  unsigned long now = millis();

  readSerialInput();
  receiveFeedback();

  // Motor control
  if (now - lastSend >= SEND_INTERVAL) {

    float dt =
      (now - lastSend) / 1000.0f;

    lastSend = now;

    // Acceleration limiting
    float maxStep = MAX_ACCEL * dt;

    currentLeft = approach(
      currentLeft,
      targetLeft,
      maxStep
    );

    currentRight = approach(
      currentRight,
      targetRight,
      maxStep
    );

    driveMotors(
      currentLeft,
      currentRight
    );
  }

  // CSV telemetry
  if (now - lastPrint >= PRINT_INTERVAL) {

    lastPrint = now;
    printCSV();
  }

  digitalWrite(
    LED_BUILTIN,
    (now % 1000) < 500
  );
}
