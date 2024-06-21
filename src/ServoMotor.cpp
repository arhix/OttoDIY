#include <Adafruit_PWMServoDriver.h>
#include <math.h>

#define SERVO_MIN 150 // Min pulse length out of 4096
#define SERVO_MAX 600 // Max pulse length out of 4096

#ifndef DEG2RAD
#define DEG2RAD(g) ((g) * M_PI) / 180
#endif

class ServoMotor
{
private:
    Adafruit_PWMServoDriver pwm;
    int pwmId;
    int angle; // Current angle of the servo
    bool attachedFlag = false;

public:
    ServoMotor() {}

    void attach(Adafruit_PWMServoDriver pwmDriver, int id, int deg = 90)
    {
        pwm = pwmDriver;
        pwmId = id;
        angle = deg;
        attachedFlag = true;

        pwm.setPWM(pwmId, 0, angle);
    }

    void write(int deg)
    {
        // Ensure deg is within 0-180 range
        if (deg < 0)
            deg = 0;
        if (deg > 180)
            deg = 180;

        // Map rad (0-180) to pulse width (SERVO_MIN to SERVO_MAX)
        int pulseWidth = map(deg, 0, 180, SERVO_MIN, SERVO_MAX);
        pwm.setPWM(pwmId, 0, pulseWidth);
        angle = deg;
    }

    bool attached() {
        return attachedFlag;
    }

    void detach() {
        attachedFlag = false;
    }
};
