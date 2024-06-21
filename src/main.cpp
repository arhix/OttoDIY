#include <Wire.h>
#include <Otto.h>
#include <Adafruit_PWMServoDriver.h>
#include "Oscillator.h"

#define N_SERVOS 4 // Number of servos
#define INTERVALTIME 10.0

Otto Otto;  //This is Otto!

#define LeftLeg 1
#define LeftFoot 0
#define RightLeg 3
#define RightFoot 2
#define Buzzer 13
#define Trigger 8 // ultrasonic sensor trigger pin
#define Echo 9 // ultrasonic sensor echo pin

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup()
{
    Serial.begin(9600);

    pwm.begin();
    pwm.setPWMFreq(60);

    Otto.init(pwm, LeftLeg, RightLeg, LeftFoot, RightFoot, Buzzer);
    pinMode(Trigger, OUTPUT); 
    pinMode(Echo, INPUT); 

    delay(10);
}

long ultrasound() {
   long duration, distance;
   digitalWrite(Trigger,LOW);
   delayMicroseconds(2);
   digitalWrite(Trigger, HIGH);
   delayMicroseconds(10);
   digitalWrite(Trigger, LOW);
   duration = pulseIn(Echo, HIGH);
   distance = duration/58;
   return distance;
}

void loop()
{
    int ult = ultrasound();
    Serial.println(ult);
    if (ult <= 15) {
      Otto.sing(S_surprise);
      Otto.playGesture(OttoConfused);
      Otto.walk(2,1000,-1); // BACKWARD x2
      Otto.turn(3,1000,1); // LEFT x3
        // delay(500);
    }

    Serial.println("walk");
    Otto.walk(1,1000,1); // FORWARD x1
}