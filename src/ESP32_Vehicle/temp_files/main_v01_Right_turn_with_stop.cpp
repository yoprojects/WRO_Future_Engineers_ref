#include <Wire.h>
#include <MPU6050_tockn.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>

// ==== PIN Definitions ====
#define BUTTON_PIN 15  // GPIO pin for the push button

#define IN1 13
#define IN2 12
#define ENA 25

#define SERVO_PIN 4
#define SERVO_LEFT 55
#define SERVO_RIGHT 132
#define SERVO_STRAIGHT 92

// Front Ultrasonic
#define TRIG_FRONT 5
#define ECHO_FRONT 18

// Right Ultrasonic
#define TRIG_RIGHT 17
#define ECHO_RIGHT 16

// ==== Global Variables ====
MPU6050 mpu6050(Wire);
Servo steeringservo;

// Lap counting variables
int turn_count = 0;        // Count of 90-degree turns made
int lap_count = 0;         // Number of laps completed
int turns_per_lap = 4;     // Assume 4 turns per lap initially (will be auto-detected)
bool first_lap = true;     // Flag to detect first lap completion

// Starting position detection
float starting_front_distance = 0;  // Store front distance at start
bool starting_position_recorded = false;
bool returning_to_start = false;    // Flag when looking for starting position

// IMU drift compensation
float yawOffset = 0;
unsigned long lastDriftReset = 0;
const unsigned long DRIFT_RESET_INTERVAL = 30000; // Reset drift every 30 seconds

// ==== WiFi Credentials ====
const char* ssid = "Airtel_yolabs_apr";
const char* password = "yolabs4321";

// ==== Telnet Server ====
WiFiServer telnetServer(23);
WiFiClient telnetClient;

// ==== Setup Telnet Logging ====
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

// ==== Handle Telnet Clients (place in loop) ====
void handleTelnetClient() {
  if (telnetServer.hasClient()) {
    if (!telnetClient || !telnetClient.connected()) {
      if (telnetClient) telnetClient.stop();
      telnetClient = telnetServer.available();
      Serial.println("New Telnet client connected");
      telnetClient.println("Welcome to ESP32 Telnet Server");
    } else {
      telnetServer.available().stop();  // Reject multiple clients
    }
  }
}

// ==== Logging Function ====
void logPrint(String msg) {
  Serial.println(msg);
  if (telnetClient && telnetClient.connected()) {
    telnetClient.println(msg);
  }
}

float readUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000); // Wait for echo, timeout after 30ms
  if (duration == 0) return 999.0; // Return max distance if no echo
  
  float distance = duration * 0.034 / 2.0; // Convert to cm
  return (distance > 0 && distance < 400) ? distance : 999.0; // Valid range
}

// Get compensated yaw angle with drift correction
float getCompensatedYaw() {
  // Reset drift compensation every 30 seconds when robot is stationary
  if (millis() - lastDriftReset > DRIFT_RESET_INTERVAL) {
    // Only reset if we're going relatively straight (no major turns recently)
    float currentYaw = mpu6050.getAngleZ();
    if (abs(currentYaw - yawOffset) < 5.0) { // If drift is small, reset
      yawOffset = currentYaw;
      lastDriftReset = millis();
      Serial.println("Yaw drift reset");
    }
  }
  
  return mpu6050.getAngleZ() - yawOffset;
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

void moveBackwardToSafeDistance() {
  Serial.println("Too close! Moving backward to safe distance...");
  
  float frontDist;
  do {
    driveBackward(120); // Slow backward movement
    delay(100);
    
    // Read front distance
    frontDist = readUltrasonicDistance(TRIG_FRONT, ECHO_FRONT);
    Serial.print("Moving back... Front distance: ");
    Serial.println(frontDist);
    
  } while (frontDist < 40 && frontDist != 999.0); // Continue until 40-50cm range
  
  stopMotors();
  delay(200);
  Serial.println("Safe distance reached!");
}

void turnLeft() {
  steeringservo.write(SERVO_LEFT);
  driveForward(150); // Adjust speed as needed
  delay(500); // Allow time for the servo to turn
}

void turnRight90Degrees() {
  Serial.println("=== STARTING RIGHT TURN ===");
  stopMotors();
  delay(200); // Brief stop before turning
  
  // Test servo before starting turn
  Serial.println("Testing servo movement...");
  steeringservo.write(SERVO_RIGHT);
  Serial.print("Servo commanded to: ");
  Serial.println(SERVO_RIGHT);
  delay(500); // Give servo time to move
  
  // Initialize angle change tracking
  mpu6050.update();
  float startYaw = getCompensatedYaw();
  float angleChanged = 0;
  float lastYaw = startYaw;
  
  Serial.print("Starting relative turn from: ");
  Serial.print(startYaw, 1);
  Serial.println("°");
  
  // Turn right with steering and drive forward
  driveForward(150); // Slower speed for more precise turning
  Serial.println("Driving forward while turning...");
  
  unsigned long turnStartTime = millis();
  const unsigned long MAX_TURN_TIME = 10000; // Maximum 10 seconds for safety

  // Keep turning until we've changed 85-90 degrees (account for some error)
  while (millis() - turnStartTime < MAX_TURN_TIME && abs(angleChanged) < 151) {
    mpu6050.update();
    float currentYaw = getCompensatedYaw();
    
    // Calculate the change in angle from last reading
    float deltaAngle = currentYaw - lastYaw;
    
    // Handle angle wrapping for delta calculation
    if (deltaAngle > 180) {
      deltaAngle -= 360;
    } else if (deltaAngle < -180) {
      deltaAngle += 360;
    }
    
    // Accumulate the angle change
    angleChanged += deltaAngle;
    lastYaw = currentYaw;
    
    // Print progress every 200ms
    if (millis() % 200 < 50) {
      Serial.print("Angle changed: ");
      Serial.print(angleChanged, 1);
      Serial.print("° (Current: ");
      Serial.print(currentYaw, 1);
      Serial.println("°)");
    }
    
    delay(20); // Small delay for stability
  }
  
  // Stop and straighten steering
  Serial.println("Stopping motors and straightening servo...");
  Serial.print("Total angle changed: ");
  Serial.print(angleChanged, 1);
  Serial.println("°");
  
  stopMotors();
  delay(200);
  steeringservo.write(SERVO_STRAIGHT);
  Serial.print("Servo commanded to straight: ");
  Serial.println(SERVO_STRAIGHT);
  delay(300);
  
  // Increment turn counter for lap tracking
  turn_count++;
  Serial.print("Turn completed! Total turns: ");
  Serial.print(turn_count);
  
  // Check for lap completion
  if (first_lap) {
    // For first lap, we don't know how many turns per lap yet
    Serial.print(" (First lap in progress)");
  } else {
    Serial.print(" (Lap ");
    Serial.print(lap_count + 1);
    Serial.print(" in progress)");
  }
  Serial.println();
  
  Serial.println("=== Relative angle turn completed ===");
}

void testServo() {
  Serial.println("=== TESTING SERVO ===");
  Serial.println("Moving to RIGHT position...");
  steeringservo.write(SERVO_RIGHT);
  delay(1000);
  
  Serial.println("Moving to LEFT position...");
  steeringservo.write(SERVO_LEFT);
  delay(1000);
  
  Serial.println("Moving to STRAIGHT position...");
  steeringservo.write(SERVO_STRAIGHT);
  delay(1000);
  
  Serial.println("Servo test completed!");
}

bool isAtStartingPosition(float currentFrontDistance) {
  // Check if current front distance matches starting distance (within 5cm tolerance)
  float tolerance = 5.0; // cm
  return abs(currentFrontDistance - starting_front_distance) <= tolerance;
}

void recordStartingPosition() {
  // Record the starting front distance
  float frontDist = readUltrasonicDistance(TRIG_FRONT, ECHO_FRONT);
  starting_front_distance = frontDist;
  starting_position_recorded = true;
  
  Serial.println("========================");
  Serial.print("STARTING POSITION RECORDED: ");
  Serial.print(starting_front_distance, 1);
  Serial.println("cm from front obstacle");
  Serial.println("========================");
}

// Forward declaration to fix 'not declared in this scope' error
void completeFirstLap();

void checkLapCompletion() {
  // Auto-complete first lap after 4 turns (typical for rectangular track)
  if (first_lap && turn_count >= 4) {
    Serial.println("Auto-detecting first lap completion after 4 turns...");
    Serial.println("Continuing for more laps...");
    completeFirstLap();
    // Don't set returning_to_start here - continue lapping!
    return;
  }
  
  if (first_lap) {
    // Still in first lap, keep counting turns
    return;
  }
  
  // For subsequent laps, check if we've made the required number of turns
  if (turn_count >= (lap_count + 1) * turns_per_lap) {
    lap_count++;
    Serial.println("========================");
    Serial.print("LAP ");
    Serial.print(lap_count);
    Serial.println(" COMPLETED!");
    Serial.print("Total turns so far: ");
    Serial.println(turn_count);
    Serial.println("========================");
    
    // Check if we've completed the target number of laps
    if (lap_count >= 3) { // Complete 3 laps first
      Serial.println("TARGET LAPS COMPLETED! Now returning to starting position...");
      returning_to_start = true; // Start looking for starting position instead of stopping immediately
    }
  }
}
void completeFirstLap() {
  // Call this function when you determine the first lap is complete
  // This sets the turns_per_lap for future lap counting
  if (first_lap) {
    turns_per_lap = turn_count;
    first_lap = false;
    lap_count = 1;
    
    Serial.println("========================");
    Serial.println("FIRST LAP COMPLETED!");
    Serial.print("Detected ");
    Serial.print(turns_per_lap);
    Serial.println(" turns per lap");
    Serial.println("Starting lap counting...");
    Serial.println("========================");
  }
}

void setup() {
  Serial.begin(115200);
  setupWiFiTelnet();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  Serial.println("Waiting for button press to start...");
  while (digitalRead(BUTTON_PIN) == HIGH) {
    delay(10);
  }

  Serial.println("Button pressed, starting setup...");

  // Initialize Servo
  steeringservo.attach(SERVO_PIN);
  steeringservo.write(SERVO_STRAIGHT); // Set initial position to straight

  Wire.begin(21, 22); // SDA, SCL pins for I2C
  mpu6050.begin();
  
  Serial.println("Calibrating gyroscope... Keep robot STILL!");
  mpu6050.calcGyroOffsets(true); // Calculate gyro offsets - KEEP THIS!
  // DO NOT call mpu6050.setGyroOffsets(0, 0, 0) - it destroys calibration!
  
  Serial.println("IMU calibration complete!");
  delay(1000); // Wait for MPU6050 to stabilize
  
  // Initialize yaw offset for drift compensation
  mpu6050.update();
  yawOffset = mpu6050.getAngleZ();
  lastDriftReset = millis();
  
  // Record starting position after a brief delay
  delay(2000); // Wait 2 seconds for sensors to stabilize
  recordStartingPosition();
}


void loop() {
  // Handle telnet client and update sensors
  handleTelnetClient();
  mpu6050.update();
  
  // Check for Serial commands for manual control
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    
    if (command == "lap") {
      completeFirstLap();
    } else if (command == "stop") {
      stopMotors();
      Serial.println("Motors stopped by user command");
    } else if (command == "reset") {
      turn_count = 0;
      lap_count = 0;
      first_lap = true;
      returning_to_start = false;
      Serial.println("Lap counter reset");
    } else if (command == "status") {
      Serial.println("=== STATUS ===");
      Serial.print("Turns: ");
      Serial.print(turn_count);
      Serial.print(", Laps: ");
      Serial.print(lap_count);
      Serial.print(", First lap: ");
      Serial.print(first_lap ? "Yes" : "No");
      Serial.print(", Turns per lap: ");
      Serial.print(turns_per_lap);
      Serial.print(", Starting distance: ");
      Serial.print(starting_front_distance, 1);
      Serial.print("cm, Returning to start: ");
      Serial.println(returning_to_start ? "Yes" : "No");
    } else if (command == "servo") {
      testServo();
    } else if (command == "turn") {
      Serial.println("Manual turn test...");
      turnRight90Degrees();
    }
  }
  
  float yawNow = getCompensatedYaw(); // Use compensated yaw
  float distFront = readUltrasonicDistance(TRIG_FRONT, ECHO_FRONT);
  float distRight = readUltrasonicDistance(TRIG_RIGHT, ECHO_RIGHT);

  // Log sensor data for debugging
  String statusMsg = "Yaw:" + String(yawNow, 1) + ", Front:" + String(distFront, 1) + ", Right:" + String(distRight, 1) + 
           ", Turns:" + String(turn_count) + ", Laps:" + String(lap_count);
  
  if (returning_to_start) {
    statusMsg += ", RETURNING TO START (Target: " + String(starting_front_distance, 1) + "cm)";
  }
  
  logPrint(statusMsg);
  
  // Check if we're returning to start and have reached starting position
  if (returning_to_start && starting_position_recorded) {
    if (isAtStartingPosition(distFront)) {
      Serial.println("========================");
      Serial.println("STARTING POSITION REACHED!");
      Serial.print("Current front distance: ");
      Serial.print(distFront, 1);
      Serial.print("cm (Target: ");
      Serial.print(starting_front_distance, 1);
      Serial.println("cm)");
      Serial.println("MISSION COMPLETE - STOPPING VEHICLE");
      Serial.println("========================");
      
      stopMotors();
      while (true) {
        delay(1000);
        Serial.println("Vehicle stopped at starting position");
      }
    }
  }
  
  // Check lap completion
  checkLapCompletion();
  
  // Main navigation logic with enhanced distance handling
  if (distFront < 20) {
    // Too close - move backward to safe distance first
    Serial.println("EMERGENCY: Too close to obstacle!");
    moveBackwardToSafeDistance();
    return; // Skip rest of loop iteration
  }
  else if (distFront < 65) {
    // If front obstacle detected (<65cm), turn right 90 degrees - PRIORITY OVER RIGHT OPENING
    Serial.println("Front obstacle detected, turning right 90 degrees");
    turnRight90Degrees();
    
    // After turn, drive forward briefly
    driveForward(150);
    delay(300);
  }
  else if (distRight > 90) {
    // If right sensor detects opening (>90cm) AND no front obstacle, continue straight
    Serial.println("Right opening detected, going straight");
    steeringservo.write(SERVO_STRAIGHT);
    driveForward(150);
  } 
  else {
    // Default: drive forward
    Serial.println("Driving forward");
    steeringservo.write(SERVO_STRAIGHT);
    driveForward(150);
  }
  
  delay(100); // Control loop timing
}