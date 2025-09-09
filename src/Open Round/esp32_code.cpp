#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>

// ==== PIN Definitions ====
#define BUTTON_PIN 15 // GPIO pin for the push button

#define IN1 13
#define IN2 12
#define ENA 25

#define SERVO_PIN 4
#define SERVO_LEFT 25
#define SERVO_RIGHT 145
#define SERVO_STRAIGHT 94

// Front Ultrasonic
#define TRIG_FRONT 5
#define ECHO_FRONT 18

// Right Ultrasonic
#define TRIG_RIGHT 17
#define ECHO_RIGHT 16

// Left Ultrasonic
#define TRIG_LEFT 26
#define ECHO_LEFT 27

// ==== Tuning Parameters ====
const int REVERSE_DURATION_MS = 700;  // Milliseconds to reverse. Calibrate this for 10cm!

// PI Controller for Yaw
float Kp = 6.0;   // Proportional gain (how strongly it corrects)
float Ki = 0.2;   // Integral gain (corrects long-term steady error)
const float MAX_CORRECTION = 22.0; // Max steering correction from PI controller
const float INTEGRAL_LIMIT = 150.0; // Prevents integral from getting too large
const float DEADBAND = 0.6;    // How much error is acceptable before correcting

// Speeds
const int DRIVE_SPEED = 150;
const int TURN_SPEED = 160;
const int REVERSE_SPEED = 120;

// ==== Global Variables ====
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Servo steeringservo;

// Lap counting
int turn_count = 0;
int lap_count = 0;
int turns_per_lap = 4;
bool first_lap = true;

// State management
bool returning_to_start = false;
bool inTurn = false; // Flag to disable PI controller during turns

// IMU and Position
float yawOffset = 0;
float targetYaw = 0;
float yawErrorIntegral = 0;
float starting_front_distance = 0;
bool starting_position_recorded = false;

// WiFi & Telnet for Debugging
const char* ssid = "Airtel_yolabs_apr";
const char* password = "yolabs4321";
WiFiServer telnetServer(23);
WiFiClient telnetClient;

// ==== WiFi and Logging Functions ====
void setupWiFiTelnet() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  telnetServer.begin();
  telnetServer.setNoDelay(true);
  Serial.println("Telnet server started");
}

void handleTelnetClient() {
  if (telnetServer.hasClient()) {
    if (!telnetClient || !telnetClient.connected()) {
      if (telnetClient) telnetClient.stop();
      telnetClient = telnetServer.available();
      Serial.println("New Telnet client connected");
    } else {
      telnetServer.available().stop();
    }
  }
}

void logPrint(String msg) {
  Serial.println(msg);
  if (telnetClient && telnetClient.connected()) {
    telnetClient.println(msg);
  }
}

// ==== Sensor and Motor Functions ====
float readUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 999.0;
  float distance = duration * 0.034 / 2.0;
  return (distance > 0 && distance < 400) ? distance : 999.0;
}

float getCompensatedYaw() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  float currentYaw = orientationData.orientation.x;
  // Convert 0-360 range to -180 to 180 for easier calculations
  if (currentYaw > 180) currentYaw -= 360;
  return currentYaw - yawOffset;
}

void driveForward(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);
}

void driveBackward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
}

void stopMotors() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 0);
}

// ==== Navigation Logic ====
void reverseBeforeTurn() {
  logPrint("Reversing before turn...");

  // Drive backward for a fixed duration
  driveBackward(REVERSE_SPEED);
  delay(REVERSE_DURATION_MS);

  // Stop the vehicle and pause briefly to settle before turning
  stopMotors();
  delay(200);

  logPrint("Reversing complete. Initiating turn.");
}

void maintainHeading(float currentYaw) {
  if (inTurn) return; // Disable PI controller during a turn

  float error = targetYaw - currentYaw;
  // Handle angle wrap-around
  if (error > 180.0) error -= 360.0;
  if (error < -180.0) error += 360.0;

  // If we're close enough, just go straight
  if (fabs(error) < DEADBAND) {
    yawErrorIntegral *= 0.9; // Decay the integral slowly
    steeringservo.write(SERVO_STRAIGHT);
    return;
  }

  yawErrorIntegral += error;
  // Prevent integral wind-up
  if (yawErrorIntegral > INTEGRAL_LIMIT) yawErrorIntegral = INTEGRAL_LIMIT;
  if (yawErrorIntegral < -INTEGRAL_LIMIT) yawErrorIntegral = -INTEGRAL_LIMIT;

  float correction = (Kp * error) + (Ki * yawErrorIntegral);

  // Clamp the correction value
  if (correction > MAX_CORRECTION) correction = MAX_CORRECTION;
  if (correction < -MAX_CORRECTION) correction = -MAX_CORRECTION;

  int steeringAngle = SERVO_STRAIGHT + (int)round(correction);
  if (steeringAngle < SERVO_LEFT) steeringAngle = SERVO_LEFT;
  if (steeringAngle > SERVO_RIGHT) steeringAngle = SERVO_RIGHT;

  steeringservo.write(steeringAngle);
}

void turnRight90Degrees() {
  inTurn = true;
  logPrint("=== STARTING RIGHT TURN ===");

  steeringservo.write(SERVO_RIGHT);
  delay(300);
  driveForward(TURN_SPEED);
  
  // Wait until the turn is mostly complete
  float turnTarget = targetYaw + 89.5;
  if (turnTarget > 180.0) turnTarget -= 360.0;
  
  unsigned long turnStart = millis();
  while(true) {
    float error = turnTarget - getCompensatedYaw();
    if (error > 180.0) error -= 360.0;
    if (error < -180.0) error += 360.0;

    if (fabs(error) < 8.0) break; // Exit loop when close to target
    if (millis() - turnStart > 4000) { // Safety timeout
      logPrint("!! Turn timeout !!");
      break;
    }
    delay(20);
  }

  // Stop and stabilize
  stopMotors();
  delay(200);
  steeringservo.write(SERVO_STRAIGHT);
  delay(400);

  // ### CRUCIAL FIX ###
  // Set the new target heading precisely based on the previous one.
  targetYaw += 90.0;
  if (targetYaw > 180.0) targetYaw -= 360.0;
  
  yawErrorIntegral = 0; // Reset PI integral to prevent jerky correction
  turn_count++;
  inTurn = false;
  
  logPrint("=== RIGHT TURN COMPLETED! New Target: " + String(targetYaw, 1) + " ===");
}

void turnLeft90Degrees() {
  inTurn = true;
  logPrint("=== STARTING LEFT TURN ===");

  steeringservo.write(SERVO_LEFT);
  delay(300);
  driveForward(TURN_SPEED);

  float turnTarget = targetYaw - 89.5;
  if (turnTarget < -180.0) turnTarget += 360.0;

  unsigned long turnStart = millis();
  while(true) {
    float error = turnTarget - getCompensatedYaw();
    if (error > 180.0) error -= 360.0;
    if (error < -180.0) error += 360.0;

    if (fabs(error) < 8.0) break;
    if (millis() - turnStart > 4000) {
      logPrint("!! Turn timeout !!");
      break;
    }
    delay(20);
  }
  
  stopMotors();
  delay(200);
  steeringservo.write(SERVO_STRAIGHT);
  delay(400);

  // ### CRUCIAL FIX ###
  // Set the new target heading precisely.
  targetYaw -= 90.0;
  if (targetYaw < -180.0) targetYaw += 360.0;
  
  yawErrorIntegral = 0; // Reset PI integral
  turn_count++;
  inTurn = false;
  
  logPrint("=== LEFT TURN COMPLETED! New Target: " + String(targetYaw, 1) + " ===");
}

void moveBackwardToSafeDistance() {
  logPrint("Too close! Moving backward...");
  driveBackward(REVERSE_SPEED);
  
  unsigned long startTime = millis();
  while (readUltrasonicDistance(TRIG_FRONT, ECHO_FRONT) < 40) {
    if(millis() - startTime > 2000) break; // Safety timeout
    delay(50);
  }

  stopMotors();
  delay(200);
  logPrint("Safe distance reached.");
}

// ==== Lap and Mission Logic ====
void completeFirstLap() {
  if (first_lap) {
    turns_per_lap = turn_count;
    first_lap = false;
    lap_count = 1;
    logPrint("========================");
    logPrint("FIRST LAP COMPLETED! Detected " + String(turns_per_lap) + " turns per lap.");
    logPrint("========================");
  }
}

void checkLapCompletion() {
  if (first_lap && turn_count >= 4) {
    completeFirstLap();
    return;
  }
  if (first_lap) return;

  if (turn_count >= turns_per_lap * (lap_count + 1)) {
    lap_count++;
    logPrint("========================");
    logPrint("LAP " + String(lap_count) + " COMPLETED!");
    logPrint("========================");
    
    if (lap_count >= 3) {
      logPrint("TARGET LAPS COMPLETED! Now returning to start...");
      returning_to_start = true;
    }
  }
}

void recordStartingPosition() {
  starting_front_distance = readUltrasonicDistance(TRIG_FRONT, ECHO_FRONT);
  starting_position_recorded = true;
  logPrint("========================");
  logPrint("STARTING POSITION RECORDED: " + String(starting_front_distance, 1) + "cm");
  logPrint("========================");
}

void returnToStartNavigation(float distFront, float distRight) {
  if (fabs(distFront - starting_front_distance) <= 5.0) {
    logPrint("========================");
    logPrint("STARTING POSITION REACHED! MISSION COMPLETE!");
    logPrint("========================");
    stopMotors();
    while (true) { delay(1000); } // End of mission
  }
  
  // Use normal navigation to get back
  if (distFront < 75 && distRight > 90) {
    turnRight90Degrees();
  } else if (distFront < 70 && distRight < 75) {
    turnLeft90Degrees();
  } else {
    driveForward(DRIVE_SPEED);
    maintainHeading(getCompensatedYaw());
  }
}

// ===================================================================
// ============================= SETUP ===============================
// ===================================================================
void setup() {
  Serial.begin(115200);
  setupWiFiTelnet();

  //pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  logPrint("Waiting for START command from Raspberry Pi...");

  // === Wait for RPi to send START ===
  String input = "";
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n') break;
      input += c;
    }
    if (input == "START") break;
  }

  logPrint("START command received from RPi!");
  /*logPrint("Press and hold the button for 1 second to start...");
  unsigned long buttonPressTime = millis();
  while (true) {
    if (digitalRead(BUTTON_PIN) == LOW) {
      if (millis() - buttonPressTime > 100) break;
    } else {
      buttonPressTime = millis();
    }
    delay(10);
  }
  logPrint("Button pressed! Initializing...");*/

  steeringservo.attach(SERVO_PIN);
  steeringservo.write(SERVO_STRAIGHT);

  Wire.begin(21, 22);
  if (!bno.begin()) {
    logPrint("FATAL: No BNO055 detected!");
    while (1);
  }
  bno.setMode(OPERATION_MODE_NDOF);
  delay(1000);

  logPrint("Calibrating... Please wait.");
  // Wait for the sensor to be at least partially calibrated
  while(bno.getMode() != OPERATION_MODE_NDOF) {
      delay(100);
  }

  yawOffset = getCompensatedYaw();
  targetYaw = getCompensatedYaw();
  yawErrorIntegral = 0;
  
  delay(1000);
  recordStartingPosition();
  
  logPrint("System initialized and ready to run!");
}

// ===================================================================
// ============================== LOOP ===============================
// ===================================================================
void loop() {
  handleTelnetClient();

  float yawNow = getCompensatedYaw();
  float distFront = readUltrasonicDistance(TRIG_FRONT, ECHO_FRONT);
  float distRight = readUltrasonicDistance(TRIG_RIGHT, ECHO_RIGHT);
  float distLeft = readUltrasonicDistance(TRIG_LEFT, ECHO_LEFT);

  if (returning_to_start) {
    returnToStartNavigation(distFront, distRight);
    return;
  }

  checkLapCompletion();

  if (distFront < 25) {
    moveBackwardToSafeDistance();
    //turnRight90Degrees(); // Make a turn after reversing to avoid getting stuck
    return;
  }
  
  if (distFront < 65 && distRight > 90 && distLeft < 90) {
    reverseBeforeTurn();
    turnRight90Degrees();

  } else if (distFront < 65 && distLeft > 90 && distRight < 90) {
     reverseBeforeTurn();
    turnLeft90Degrees();
  } else {
    driveForward(DRIVE_SPEED);
    maintainHeading(yawNow);
  }

  delay(50);
}
