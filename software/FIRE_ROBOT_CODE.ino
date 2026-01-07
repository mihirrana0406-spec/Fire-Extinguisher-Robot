#include <Wire.h>
#include <NewPing.h>
#include <Adafruit_PWMServoDriver.h>
#include <IRremote.hpp>
#include <Servo.h>

// Create PCA9685 object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); // Default I2C address is 0x40

// Define servo min/max pulse values (adjust for your servos)
#define SERVO_MIN  150  // 0 degrees
#define SERVO_MAX  600  // 180 degrees

// Define home positions
const int HOME_POSITIONS[] = {55, 120, 160}; // Home angles for servos 0, 1, 2
const int FLAME_POSITIONS[] = {55, 30, 160}; // FlameServo angles for servos 0, 1, 2

// ==================== Pin Definitions ====================
const int IR_RECEIVER_PIN = 14;

// Motor driver pins
#define IN1 3  
#define IN2 4
#define ENA 2  

#define IN3 5  
#define IN4 6
#define ENB 7  

#define IN5 9  
#define IN6 10
#define ENC 8  

#define IN7 11  
#define IN8 12
#define END 13  

#define IN9 24  
#define IN10 22
#define ENE 26 

// Flame sensors
#define FLAME_FRONT A2
#define FLAME_RIGHT A3
#define FLAME_BACK  A0
#define FLAME_LEFT  A1

#define TRIG_PIN 50  // Trig pin connected to digital pin 9
#define ECHO_PIN 52 // Echo pin connected to digital pin 10
#define MAX_DISTANCE 400

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

// IR remote command values
#define IR_FORWARD    0xE41BD6DE
#define IR_BACKWARD   0xE01FD6DE
#define IR_RIGHT      0xE619D6DE
#define IR_LEFT       0xE21DD6DE
#define IR_STOP       0x7F80D6DE
#define IR_SPEED_UP   0xE51AD6DE
#define IR_SPEED_DOWN 0xE11ED6DE

int speed = 150;
IRrecv irrecv(IR_RECEIVER_PIN);
IRData receivedData;  // Corrected type

// ==================== Function Prototypes ====================
void moveForward();
void moveBackward();
void turnRight();
void turnLeft();
void stopMotors();
void increaseSpeed();
void decreaseSpeed();
void applySpeed();
void faceFlame();
void setHomePosition();
void moveFlameServo();
void moveServo(int servoNum, int angle);

// ==================== Setup ====================
void setup() {
  Serial.begin(9600);
  Serial.println("PCA9685 Servo & Robot Setup");

  Wire.begin();  // Initialize I2C
  pwm.begin();
  pwm.setPWMFreq(50);  // Set frequency to 50Hz for servos
  delay(10);

  irrecv.enableIRIn();
  Serial.println("IR Receiver is ready");

  // Initialize motor control pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN5, OUTPUT); pinMode(IN6, OUTPUT); pinMode(ENC, OUTPUT);
  pinMode(IN7, OUTPUT); pinMode(IN8, OUTPUT); pinMode(END, OUTPUT);
  pinMode(IN9, OUTPUT); pinMode(IN10, OUTPUT); pinMode(ENE, OUTPUT);

  // Initialize flame sensor pins
  pinMode(FLAME_FRONT, INPUT);
  pinMode(FLAME_RIGHT, INPUT);
  pinMode(FLAME_BACK, INPUT);
  pinMode(FLAME_LEFT, INPUT);

  stopMotors();

  // Move servos to home position at start
  setHomePosition();
}

// ==================== Main Loop ====================
void loop() {
  // Check for flame detection first

  if (digitalRead(FLAME_FRONT) || digitalRead(FLAME_RIGHT) || digitalRead(FLAME_BACK) || digitalRead(FLAME_LEFT)) {
    Serial.println("Flame detected! Facing flame...");
    faceFlame();
  } 
  else {
    // Check for IR remote control input
    if (irrecv.decode()) {  
      unsigned long command = irrecv.decodedIRData.decodedRawData;  

      Serial.print("Received IR Code: ");
      Serial.println(command, HEX);

      switch (command) {
        case IR_FORWARD: moveForward(); break;
        case IR_BACKWARD: moveBackward(); break;
        case IR_RIGHT: turnRight(); break;
        case IR_LEFT: turnLeft(); break;
        case IR_STOP: stopMotors(); break;
        case IR_SPEED_UP: increaseSpeed(); break;
        case IR_SPEED_DOWN: decreaseSpeed(); break;
        //default: stopMotors(); break;
      }

      irrecv.resume(); // Resume receiver
    }
  }
}

// ==================== Motor Control Functions ====================
void moveForward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, speed);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, speed);
  digitalWrite(IN5, HIGH); digitalWrite(IN6, LOW); analogWrite(ENC, speed);
  digitalWrite(IN7, LOW); digitalWrite(IN8, HIGH); analogWrite(END, speed);
}

void moveBackward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, speed);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, speed);
  digitalWrite(IN5, LOW); digitalWrite(IN6, HIGH); analogWrite(ENC, speed);
  digitalWrite(IN7, HIGH); digitalWrite(IN8, LOW); analogWrite(END, speed);
}

void turnLeft() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, speed);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, speed);
  digitalWrite(IN5, LOW); digitalWrite(IN6, HIGH); analogWrite(ENC, speed);
  digitalWrite(IN7, HIGH); digitalWrite(IN8, LOW); analogWrite(END, speed);
  Serial.println("Turning Left");
}

void turnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, speed);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, speed);
  digitalWrite(IN5, HIGH); digitalWrite(IN6, LOW); analogWrite(ENC, speed);
  digitalWrite(IN7, LOW); digitalWrite(IN8, HIGH); analogWrite(END, speed);
  Serial.println("Turning Right");
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW); digitalWrite(IN6, LOW);
  digitalWrite(IN7, LOW); digitalWrite(IN8, LOW);
}

// ==================== Speed Control ====================
void increaseSpeed() { speed = min(speed + 10, 255); applySpeed(); }
void decreaseSpeed() { speed = max(speed - 10, 0); applySpeed(); }
void applySpeed() {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  analogWrite(ENC, speed);
  analogWrite(END, speed);
}

// ==================== Flame Detection ====================
void faceFlame() {
    Serial.println("Flame detected! Aligning robot to face it...");

    bool flameDetected = false;

    // Step 1: Locate the flame and turn towards it
    while (!flameDetected) {
        bool front = digitalRead(FLAME_FRONT);
        bool right = digitalRead(FLAME_RIGHT);
        bool left  = digitalRead(FLAME_LEFT);
        bool back  = digitalRead(FLAME_BACK);

        if (front) {
            Serial.println("Flame detected in front. Ready for distance adjustment.");
            flameDetected = true; // Now we can adjust distance
        } 
        else if (right) {
            Serial.println("Flame detected on the right. Turning right...");
            turnLeft();
        } 
        else if (left) {
            Serial.println("Flame detected on the left. Turning left...");
            turnLeft();
        } 
        else if (back) {
            Serial.println("Flame detected at the back. Turning left...");
            turnLeft(); // Longer turn if the flame is at the back
        } 
        else {
            Serial.println("No flame detected, stopping.");
            turnLeft();
            return; // Exit function if no flame is found
        }
    }

    // Step 2: Adjust distance only AFTER facing the flame
    Serial.println("Flame in front! Adjusting distance...");

    int distance = sonar.ping_cm();

    while (distance != 10) {
        distance = sonar.ping_cm(); // Get updated distance
        
        if (distance > 10) { 
            moveForward();
            Serial.print("Moving Forward, Distance: "); Serial.println(distance);
        } 
        else if (distance < 10) { 
            moveBackward();
            Serial.print("Moving Backward, Distance: "); Serial.println(distance);
        }
        
        delay(100); // Allow sensor readings to stabilize
    }

    stopMotors();
    Serial.println("Optimal distance (10 cm) reached. Activating FlameServo...");
    
    moveFlameServo();
    delay(5000); // Allow servo operation
    digitalWrite(IN9, HIGH); digitalWrite(IN10, LOW); analogWrite(ENE, 255);
    delay(5000);
    setHomePosition(); // Reset servos
    stopMotors();
}

// ==================== Servo Control ====================
void setHomePosition() {
  for (int i = 0; i < 3; i++) {
    moveServo(i, HOME_POSITIONS[i]);
  }
  Serial.println("Servos set to home position.");
}

void moveFlameServo() {
  for (int i = 0; i < 3; i++) {
    moveServo(i, FLAME_POSITIONS[i]);
  }
  Serial.println("Servos moved to Flame position.");
}

void moveServo(int servoNum, int angle) {
  int pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(servoNum, 0, pulse);
  Serial.print("Servo "); Serial.print(servoNum);
  Serial.print(" -> "); Serial.print(angle); Serial.println("°");
}
