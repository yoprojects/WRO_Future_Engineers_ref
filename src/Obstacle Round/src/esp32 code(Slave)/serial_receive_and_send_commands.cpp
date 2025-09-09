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
bool systemStarted = false;   // Only runs after START received

// ==============================
// Helper Functions
// ==============================
long readUltrasonic(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 25000); // 25ms timeout
  if (duration == 0) return 999;              // no echo
  return duration * 0.034 / 2;                // distance in cm
}

void setMotor(int speed) {
  if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speed);   // 0–255
  } else if (speed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -speed);  // reverse
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);       // stop
  }
}

void setSteering(int angle) {
  angle = constrain(angle, SERVO_LEFT, SERVO_RIGHT);
  steeringServo.write(angle);
}

// ==============================
// Setup
// ==============================
void setup() {
  Serial.begin(115200);

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

  Serial.println("ESP32 ready, waiting for START...");
}

// ==============================
// Main Loop
// ==============================
void loop() {
  if (!systemStarted) {
    if (Serial.available()) {
      String msg = Serial.readStringUntil('\n');
      msg.trim();
      if (msg == "START") {
        systemStarted = true;
        Serial.println("ACK_START");
      }
    }
    delay(50);
    return;
  }

  // Handle motor + steering commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.length() > 0) {
      int commaIndex = cmd.indexOf(',');
      if (commaIndex > 0) {
        int speed = cmd.substring(0, commaIndex).toInt();
        int steering = cmd.substring(commaIndex + 1).toInt();

        setMotor(speed);
        setSteering(steering);
      }
    }
  }

  // Read ultrasonic sensors
  long front = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long left  = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
  long right = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  // Send data back to RPi
  Serial.print(front); Serial.print(",");
  Serial.print(left);  Serial.print(",");
  Serial.print(right); Serial.println();

  delay(50);  // ~20Hz update rate
}
