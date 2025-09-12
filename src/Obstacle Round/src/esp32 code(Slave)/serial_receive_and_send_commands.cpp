#include <Arduino.h>
#include <ESP32Servo.h>

// ==============================
// Pin definitions
// ==============================
#define IN1 13
#define IN2 12
#define ENA 25

#define SERVO_PIN 4
#define SERVO_LEFT 25
#define SERVO_RIGHT 145
#define SERVO_STRAIGHT 94

// Ultrasonic sensors
#define TRIG_FRONT 5
#define ECHO_FRONT 18

#define TRIG_RIGHT 17
#define ECHO_RIGHT 16

#define TRIG_LEFT 26
#define ECHO_LEFT 27

// ==============================
// Globals
// ==============================
Servo steeringServo;
bool systemStarted = false;

// Current motor and servo states
int currentSpeed = 0;
int currentSteering = SERVO_STRAIGHT;

// Timing for non-blocking operations
unsigned long lastSensorRead = 0;
unsigned long lastStatusSend = 0;
const unsigned long SENSOR_INTERVAL = 50;  // Read sensors every 50ms
const unsigned long STATUS_INTERVAL = 50;  // Send status every 50ms

// Latest sensor readings
long frontDistance = 999;
long leftDistance = 999;
long rightDistance = 999;

// Command buffer to handle latest command only
String latestCommand = "";
bool hasNewCommand = false;

// ==============================
// Helper Functions
// ==============================
long readUltrasonicNonBlocking(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // Use shorter timeout to prevent blocking
  long duration = pulseIn(echo, HIGH, 15000); // 15ms timeout instead of 25ms
  if (duration == 0) return 999;              // no echo
  return duration * 0.034 / 2;                // distance in cm
}

void setMotor(int speed) {
  if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, constrain(speed, 0, 255));
  } else if (speed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, constrain(-speed, 0, 255));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
  currentSpeed = speed;
}

void setSteering(int angle) {
  angle = constrain(angle, SERVO_LEFT, SERVO_RIGHT);
  steeringServo.write(angle);
  currentSteering = angle;
}

void readAllSensors() {
  // Read all three sensors quickly
  frontDistance = readUltrasonicNonBlocking(TRIG_FRONT, ECHO_FRONT);
  leftDistance = readUltrasonicNonBlocking(TRIG_LEFT, ECHO_LEFT);
  rightDistance = readUltrasonicNonBlocking(TRIG_RIGHT, ECHO_RIGHT);
}

void processLatestCommand() {
  if (!hasNewCommand) return;
  
  String cmd = latestCommand;
  latestCommand = "";
  hasNewCommand = false;
  
  if (cmd.length() > 0) {
    int commaIndex = cmd.indexOf(',');
    if (commaIndex > 0) {
      int speed = cmd.substring(0, commaIndex).toInt();
      int steering = cmd.substring(commaIndex + 1).toInt();

      // Apply commands immediately
      setMotor(speed);
      setSteering(steering);
      
      // Optional: Send acknowledgment for critical commands
      if (abs(speed) > 200 || abs(steering - SERVO_STRAIGHT) > 40) {
        Serial.print("ACK:");
        Serial.print(speed);
        Serial.print(",");
        Serial.println(steering);
      }
    }
  }
}

void handleSerialInput() {
  // Process all available serial data to get the latest command
  while (Serial.available()) {
    String incomingCmd = Serial.readStringUntil('\n');
    incomingCmd.trim();
    
    if (incomingCmd.length() > 0) {
      // Always keep only the latest command (overwrite previous)
      latestCommand = incomingCmd;
      hasNewCommand = true;
    }
  }
}

// ==============================
// Setup
// ==============================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1); // Very short timeout for non-blocking reads

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  steeringServo.attach(SERVO_PIN);
  setSteering(SERVO_STRAIGHT);
  setMotor(0);

  Serial.println("ESP32_V2_READY");
  Serial.flush();
}

// ==============================
// Main Loop - Non-blocking approach
// ==============================
void loop() {
  unsigned long currentTime = millis();
  
  // Handle startup handshake
  if (!systemStarted) {
    if (Serial.available()) {
      String msg = Serial.readStringUntil('\n');
      msg.trim();
      if (msg == "START") {
        systemStarted = true;
        Serial.println("ACK_START");
        Serial.flush();
        
        // Initialize sensor readings
        readAllSensors();
        lastSensorRead = currentTime;
        lastStatusSend = currentTime;
      }
    }
    delay(10);
    return;
  }

  // 1. HIGHEST PRIORITY: Handle incoming commands immediately
  handleSerialInput();
  processLatestCommand();

  // 2. MEDIUM PRIORITY: Read sensors at regular intervals
  if (currentTime - lastSensorRead >= SENSOR_INTERVAL) {
    readAllSensors();
    lastSensorRead = currentTime;
  }

  // 3. LOW PRIORITY: Send status data at regular intervals
  if (currentTime - lastStatusSend >= STATUS_INTERVAL) {
    // Send sensor data back to RPi in expected format
    Serial.print("DIST,");
    Serial.print(frontDistance);
    Serial.print(",");
    Serial.print(leftDistance);
    Serial.print(",");
    Serial.print(rightDistance);
    Serial.println();
    Serial.flush(); // Ensure data is sent immediately
    
    lastStatusSend = currentTime;
  }

  // Small delay to prevent overwhelming the system
  delay(1); // Very small delay - allows for ~1000 Hz loop rate
}
