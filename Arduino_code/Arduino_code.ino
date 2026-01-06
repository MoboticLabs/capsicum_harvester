#include <AccelStepper.h>
#include <Servo.h>  // Include standard Servo library

// --- PIN DEFINITIONS ---
// Steppers
#define M1_DIR 2
#define M1_STEP 3
#define M2_DIR 4
#define M2_STEP 5
#define M3_DIR 6
#define M3_STEP 7
#define M4_DIR 8
#define M4_STEP 9
#define M5_DIR 10
#define M5_STEP 11
#define M6_DIR 12
#define M6_STEP 13

// Limit Switches
#define LIMIT_1 22
#define LIMIT_2 23 
#define LIMIT_3 24
#define LIMIT_4 25
#define LIMIT_5 26

// --- NEW: SENSOR & SERVO PINS ---
#define TRIG_PIN 30
#define ECHO_PIN 31
#define SERVO_1_PIN 44
#define SERVO_2_PIN 45

// Interface Type
#define MOTOR_INTERFACE 1

// Objects
AccelStepper motor1(MOTOR_INTERFACE, M1_STEP, M1_DIR);
AccelStepper motor2(MOTOR_INTERFACE, M2_STEP, M2_DIR);
AccelStepper motor3(MOTOR_INTERFACE, M3_STEP, M3_DIR);
AccelStepper motor4(MOTOR_INTERFACE, M4_STEP, M4_DIR);
AccelStepper motor5(MOTOR_INTERFACE, M5_STEP, M5_DIR);
AccelStepper motor6(MOTOR_INTERFACE, M6_STEP, M6_DIR);

Servo servo1;
Servo servo2;

// Global Variables
String inputString = "";
boolean stringComplete = false;

// Homing Offsets
long offset1 = 4000;
long offset2 = 4000;
long offset3 = 4000;
long offset4 = 800;
long offset5 = 2000;

// Sensor Settings
int detectionDistance = 15; // Trigger if object is closer than 15cm
bool hasTriggered = false;  // To prevent double triggering

void setup() {
  Serial.begin(115200);
  inputString.reserve(200);
  
  // 1. Configure Switches
  pinMode(LIMIT_1, INPUT_PULLUP);
  pinMode(LIMIT_2, INPUT_PULLUP);
  pinMode(LIMIT_3, INPUT_PULLUP);
  pinMode(LIMIT_4, INPUT_PULLUP);
  pinMode(LIMIT_5, INPUT_PULLUP);

  // 2. Configure Sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // 3. Configure Servos
  servo1.attach(SERVO_1_PIN);
  servo2.attach(SERVO_2_PIN);
  
  // Set Initial Servo Positions (e.g., 0 degrees)
  servo1.write(0);
  servo2.write(0);

  // 4. Configure Motor Physics
  float maxSpeed = 1000.0;
  float accel = 500.0;
  motor1.setMaxSpeed(maxSpeed);    motor1.setAcceleration(accel);
  motor2.setMaxSpeed(maxSpeed);    motor2.setAcceleration(accel);
  motor3.setMaxSpeed(maxSpeed);    motor3.setAcceleration(accel);
  motor4.setMaxSpeed(maxSpeed);    motor4.setAcceleration(accel);
  motor5.setMaxSpeed(maxSpeed);    motor5.setAcceleration(accel);
  motor6.setMaxSpeed(maxSpeed);    motor6.setAcceleration(accel);

  motor3.setPinsInverted(true, false, false);

  // 5. HOMING SEQUENCE
  Serial.println("Starting Auto-Homing...");
  homeJoint(5, motor6, LIMIT_5, offset5);
  homeJoint(4, motor5, LIMIT_4, offset4);
  homeJoint(3, motor4, LIMIT_3, offset3);
  homeJoint(1, motor1, LIMIT_1, offset1);
  homeCoupledJoint(motor2, motor3, LIMIT_2, offset2);
  Serial.println("Homing Complete.");
}

void loop() {
  // A. Check for Serial Commands (from ROS/Python)
  if (stringComplete) {
    parseAndMove();
    inputString = "";
    stringComplete = false;
  }

  // B. Check Ultrasonic Sensor
  checkSensorAndAct();
}

// --- NEW: SENSOR FUNCTION ---
void checkSensorAndAct() {
  // Clear trig
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Send pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read Echo
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
  int distance = duration * 0.034 / 2;
  
  // Logic: If distance is valid (not 0) and less than threshold
  if (distance > 0 && distance < detectionDistance) {
    if (!hasTriggered) {
      Serial.println("Object Detected! Triggering Servos...");
      
      // 1. Turn Servo 1 to 90 degrees
      servo1.write(90);
      
      // 2. Wait 4 Seconds
      delay(4000); 
      
      // 3. Turn Servo 2 to 90 degrees
      servo2.write(90);
      
      // 4. Reset Flag / Wait / Reset Servos?
      // For now, let's wait 2 seconds then reset them so it can trigger again
      delay(2000);
      servo1.write(0);
      servo2.write(0);
      
      hasTriggered = false; 
    }
  }
}

// --- EXISTING HOMING & MOVEMENT FUNCTIONS BELOW ---
// (Keep the rest of your functions exactly as they were)

void homeJoint(int id, AccelStepper &m, int pin, long offset) {
  // ... (Paste your existing homeJoint code here) ...
  m.setCurrentPosition(0);
  m.moveTo(100000); 
  while (digitalRead(pin) == HIGH) { m.run(); if (m.currentPosition() > 30000) break; }
  m.stop();
  m.move(-offset);
  m.runToPosition();
  m.setCurrentPosition(0);
}

void homeCoupledJoint(AccelStepper &m2, AccelStepper &m3, int pin, long offset) {
  // ... (Paste your existing homeCoupledJoint code here) ...
   m2.setCurrentPosition(0); m3.setCurrentPosition(0);
   m2.moveTo(100000); m3.moveTo(100000);
   while (digitalRead(pin) == HIGH) { m2.run(); m3.run(); }
   m2.stop(); m3.stop();
   m2.move(-offset); m3.move(-offset);
   while(m2.distanceToGo() != 0) { m2.run(); m3.run(); }
   m2.setCurrentPosition(0); m3.setCurrentPosition(0);
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') stringComplete = true;
    else inputString += inChar;
  }
}

void parseAndMove() {
  // ... (Paste your existing parseAndMove code here) ...
  long joint_targets[5];
  int currentIndex = 0;
  int commaIndex = inputString.indexOf(',');
  for(int i=0; i<5; i++) {
    if(commaIndex == -1) {
       joint_targets[i] = inputString.substring(currentIndex).toInt();
    } else {
       joint_targets[i] = inputString.substring(currentIndex, commaIndex).toInt();
       currentIndex = commaIndex + 1;
       commaIndex = inputString.indexOf(',', currentIndex);
    }
  }
  motor1.moveTo(joint_targets[0]); motor1.runToPosition();
  motor2.moveTo(joint_targets[1]); motor3.moveTo(joint_targets[1]); 
  while (motor2.distanceToGo() != 0) { motor2.run(); motor3.run(); }
  motor4.moveTo(joint_targets[2]); motor4.runToPosition();
  motor5.moveTo(joint_targets[3]); motor5.runToPosition();
  motor6.moveTo(joint_targets[4]); motor6.runToPosition();
  Serial.println("Done.");
}
