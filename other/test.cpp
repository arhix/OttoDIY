#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Every.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define PIN_RL 0   // Replace with your actual PCA9685 channel assignments
#define PIN_RR 1
#define PIN_YR 2
#define PIN_YL 3

#define SERVO_MIN 150        // Min pulse length out of 4096
#define SERVO_MAX 600        // Max pulse length out of 4096
const uint8_t servoChannel = 0;

int minVal = SERVO_MIN;
int maxVal = SERVO_MAX;

Every servoMover(1000);  // Create a task to move the servo every second
Every serialReader(1000);  // Create a task to read the serial input every second

uint8_t servoState = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Enter commands in the format 'min<number>' or 'max<number>'. Type 'done' to finish.");

  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~50 Hz

  pwm.setPWM(PIN_RR, 0, SERVO_MIN + (SERVO_MAX - SERVO_MIN) / 2);
  pwm.setPWM(PIN_RL, 0, SERVO_MIN + (SERVO_MAX - SERVO_MIN) / 2);
  pwm.setPWM(PIN_YR, 0, SERVO_MIN + (SERVO_MAX - SERVO_MIN) / 2);
  pwm.setPWM(PIN_YL, 0, SERVO_MIN + (SERVO_MAX - SERVO_MIN) / 2);

  delay(10);
}

void moveServo() {
  int midVal = (minVal + maxVal) / 2;
  // Cycle through servo positions
  switch (servoState) {
    case 0:
      pwm.setPWM(servoChannel, 0, midVal);
      break;
    case 1:
      pwm.setPWM(servoChannel, 0, minVal);
      break;
    case 2:
      pwm.setPWM(servoChannel, 0, midVal);
      break;
    case 3:
      pwm.setPWM(servoChannel, 0, maxVal);
      break;
  }
  servoState = (servoState + 1) % 4;
}

void processCommand(String input) {
  input.trim();  // Remove any leading or trailing whitespace

  if (input.equalsIgnoreCase("done")) {
    while (true);  // Stop the loop after finishing
  }

  if (input.startsWith("min")) {
    String numStr = input.substring(3);  // Extract the numeric part
    int num = numStr.toInt();
    if (num != 0 || numStr.equals("0")) {  // Ensure valid integer input
      minVal = num;
      Serial.println(minVal);
    } else {
      Serial.println("Invalid input for min. Please enter in the format 'min<number>'.");
    }
  } else if (input.startsWith("max")) {
    String numStr = input.substring(3);  // Extract the numeric part
    int num = numStr.toInt();
    if (num != 0 || numStr.equals("0")) {  // Ensure valid integer input
      maxVal = num;
      Serial.println(maxVal);
    } else {
      Serial.println("Invalid input for max. Please enter in the format 'max<number>'.");
    }
  } else {
    Serial.println("Invalid command. Please enter in the format 'min<number>' or 'max<number>'.");
  }

  Serial.print("Current Min: ");
  Serial.println(minVal);
  Serial.print("Current Max: ");
  Serial.println(maxVal);
}

void readSerialInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    processCommand(input);
  }
}

void loop() {
  if (servoMover()) moveServo();  // Check and run the servo mover task if it's time
  if (serialReader()) readSerialInput();  // Check and run the serial reader task if it's time
}
