#include <Servo.h>

// Constants for line sensors
const int leftSensor = A0;
const int rightSensor = A1;

// Constants for motor driver
const int motorA1 = 9; // Motor driver input A1
const int motorA2 = 8; // Motor driver input A2
const int motorB1 = 7; // Motor driver input B1
const int motorB2 = 6; // Motor driver input B2

const int enA = 10;
const int enB = 5;
// Servo motor setup
Servo steeringServo;
const int servoPin = 11;

// Bluetooth variables
char btCommand; // Command received from the Bluetooth module

void setup() {
  // Initialize the sensors as inputs
  pinMode(leftSensor, INPUT);
  pinMode(rightSensor, INPUT);

  // Initialize the motor driver pins as outputs
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  // Attach the servo motor to its control pin
  steeringServo.attach(servoPin);

  // Start serial communication with the Bluetooth module
  Serial.begin(9600);
}

void loop() {
  // Read the sensors
  int leftSensorValue = digitalRead(leftSensor);
  int rightSensorValue = digitalRead(rightSensor);

  // Line following logic
  if (leftSensorValue == LOW && rightSensorValue == HIGH) {
    // Turn left
    digitalWrite(enA, HIGH);
    digitalWrite(enB, HIGH);
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
     // Adjust the angle as necessary
  } else if (leftSensorValue == HIGH && rightSensorValue== LOW) {
    // Turn right
    digitalWrite(enA, HIGH);
    digitalWrite(enB, HIGH);
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, HIGH);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
     // Adjust the angle as necessary
  } else if(leftSensorValue == LOW && rightSensorValue==LOW) {
    // Go straight
    digitalWrite(enA, HIGH);
    digitalWrite(enB, HIGH);
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
    
  }
  else{
    digitalWrite(enA, HIGH);
    digitalWrite(enB, HIGH);
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, LOW);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, LOW);
  }

  // Check for Bluetooth commands
  if (Serial.available() > 0) {
    btCommand = Serial.read(); // Read the incoming byte

    // Perform actions based on the Bluetooth command
    switch (btCommand) {
      case 'L': // Turn servo left
        steeringServo.write(45);
        break;
      case 'R': // Turn servo right
        steeringServo.write(135);
        break;
      case 'S': // Center servo
        steeringServo.write(90);
        break;
      // Add more cases as needed for other commands
    }
  }

  // Add a small delay to prevent reading too quickly
  delay(100);
}