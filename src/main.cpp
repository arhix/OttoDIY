
#include <Adafruit_PWMServoDriver.h>
#include <Otto.h>
#include <Wire.h>
Otto Otto;

#define mLeftLeg 1
#define mLeftFoot 0
#define mRightLeg 3
#define mRightFoot 2

#define Buzzer 13

// ultrasonic sensor
#define Trigger 8
#define Echo 9

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int pause;

void setup() {
    Serial.begin(9600);

    pwm.begin();
    pwm.setPWMFreq(60);

    Otto.init(pwm, mLeftLeg, mRightLeg, mLeftFoot, mRightFoot, Buzzer);
    pinMode(Trigger, OUTPUT);
    pinMode(Echo, INPUT);

    Otto.home();
    delay(10);

    Serial.println("OTTO PROGRAM");
    Serial.println("PRESS a or z for moving Left Leg");
    Serial.println("PRESS s or x for moving Left Foot");
    Serial.println("PRESS k or m for moving Right Leg");
    Serial.println("PRESS j or n for moving Right Foot");
    Serial.println();
    Serial.println("PRESS w to test Otto walking");
    Serial.println("PRESS h to return servos to home position");
    Serial.println("PRESS d for dancing");
    Serial.println("PRESS c for crusaito");
    Serial.println("PRESS p for patada");
    Serial.println("PRESS u for upDown");
    Serial.println("PRESS l for kickLeft");
    Serial.println("PRESS o for kickRight");
    Serial.println("PRESS 1 for walk");
    Serial.println("PRESS 2 for run");
    Serial.println("PRESS 3 for backyard");
    Serial.println("PRESS 4 for moonWalkRight");
    Serial.println("PRESS 5 for moonWalkLeft");
}

long ultrasound()
{
    long duration, distance;
    digitalWrite(Trigger, LOW);
    delayMicroseconds(2);
    digitalWrite(Trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trigger, LOW);
    duration = pulseIn(Echo, HIGH);
    distance = duration / 58;
    return distance;
}

void ultrasound_loop()
{
    int ult = ultrasound();
    Serial.println(ult);

    if (ult <= 15)
    {
        Otto.sing(S_surprise);
        Otto.playGesture(OttoConfused);
        Otto.walk(2, 1000, -1); // BACKWARD x2
        Otto.turn(3, 1000, 1);  // LEFT x3
        // delay(500);
    }

    Serial.println("walk");
    Otto.walk(1, 1000, 1); // FORWARD x1
}

void dance(int tempo);
void drunk(int tempo);
void patada(int tempo);
void kickLeft(int tempo);
void kickRight(int tempo);
void pasitos(int steps, int tempo);
void run(int steps, int T = 500);
void walk(int steps, int T = 1000);
void backyard(int steps, int T = 3000);
void moonWalkLeft(int steps, int T = 1000);
void moonWalkRight(int steps, int T = 1000);
void crusaito(int steps, int T = 1000);
void swing(int steps, int T = 1000);
void upDown(int steps, int tempo);
void flapping(int steps, int T = 1000);

void loop() {
    int charRead = 0;

    if ((Serial.available()) > (0)) {
        charRead = Serial.read();
    }
    switch (charRead) {
        case 'h':
            Serial.println("home");
            Otto.home();
            break;
        case 'w':
            Serial.println("walk");
            Otto.walk(1, 1000, 1);
            break;
        case 'a':
            Serial.println("Left Leg +");
            Otto._moveSingle(120, Otto.LeftLeg);
            break;
        case 'z':
            Serial.println("Left Leg -");
            Otto._moveSingle(30, Otto.LeftLeg);
            break;
        case 's':
            Serial.println("Left Foot +");
            Otto._moveSingle(120, Otto.LeftFoot);
            break;
        case 'x':
            Serial.println("Left Foot -");
            Otto._moveSingle(30, Otto.LeftFoot);
            break;
        case 'k':
            Serial.println("Right Leg +");
            Otto._moveSingle(120, Otto.RightLeg);
            break;
        case 'm':
            Serial.println("Right Leg -");
            Otto._moveSingle(30, Otto.RightLeg);
            break;
        case 'j':
            Serial.println("Right Foot +");
            Otto._moveSingle(120, Otto.RightFoot);
            break;
        case 'n':
            Serial.println("Right Foot -");
            Otto._moveSingle(30, Otto.RightFoot);
            break;
        case 'c':
            Serial.println("Crusaito");
            crusaito(1, 620);
            break;
        case 'p':
            Serial.println("patada");
            patada(620);
            break;
        case 'u':
            Serial.println("upDown");
            upDown(1, 620);
            break;
        case 'l':
            Serial.println("kickLeft");
            kickLeft(620);
            break;
        case 'o':
            Serial.println("kickRight");
            kickRight(620);
            break;
        case 'd':
            Serial.println("dance");
            dance(620);
            break;
        case '1':
            Serial.println("walk");
            walk(2, 620);
            break;
        case '2':
            Serial.println("run");
            run(3, 620);
            break;
        case '3':
            Serial.println("backyard");
            backyard(3, 620);
            break;
        case '4':
            Serial.println("moonWalkRight");
            moonWalkRight(3, 620);
            break;
        case '5':
            Serial.println("moonWalkLeft");
            moonWalkLeft(3, 620);
            break;
    }
}

void adjustServoTargets(const int original[4], int adjusted[4]) {
    adjusted[Otto.RightFoot] = original[0];
    adjusted[Otto.LeftFoot] = original[1];
    adjusted[Otto.RightLeg] = original[2];
    adjusted[Otto.LeftLeg] = original[3];
}

void adjustServoTargets(const double original[4], double adjusted[4]) {
    adjusted[Otto.RightFoot] = original[0];
    adjusted[Otto.LeftFoot] = original[1];
    adjusted[Otto.RightLeg] = original[2];
    adjusted[Otto.LeftLeg] = original[3];
}

void moveServos(int time, int move[]) {
    int newMove[4] = {0, 0, 0, 0};
    adjustServoTargets(move, newMove);
    Otto._moveServos(time, newMove);
}

void oscillateServos(int A[4], int O[4], int T, double phaseDiff[4],
                     float cycle = 1) {
    int newA[4] = {0, 0, 0, 0};
    int newO[4] = {0, 0, 0, 0};
    double newPhaseDiff[4] = {0.0, 0.0, 0.0, 0.0};

    // Adjust the targets
    adjustServoTargets(A, newA);
    adjustServoTargets(O, newO);
    adjustServoTargets(phaseDiff, newPhaseDiff);

    Otto.oscillateServos(newA, newO, T, newPhaseDiff, cycle);
}

void crusaito(int steps, int T) {
    Serial.println("crusaito");
    int A[4] = {25, 25, 30, 30};
    int O[4] = {-15, 15, 0, 0};
    double phaseDiff[4] = {DEG2RAD(0), DEG2RAD(180 + 120), DEG2RAD(90),
                           DEG2RAD(90)};

    oscillateServos(A, O, T, phaseDiff, steps);
}

void pasitos(int steps, int tempo) {
    Serial.println("pasitos");
    int move1[4] = {120, 60, 90, 90};  // Example servo positions for 'pasitos'
    int move2[4] = {90, 90, 90, 90};

    for (int i = 0; i < steps; i++) {
        moveServos(tempo / 4, move1);
        moveServos(tempo / 4, move2);
        moveServos(tempo / 4, move1);
        moveServos(tempo / 4, move2);
        delay(tempo / 2);
    }
}

void upDown(int steps, int tempo) {
    Serial.println("upDown");
    int move1[4] = {50, 130, 90, 90};
    int move2[4] = {90, 90, 90, 90};

    for (int x = 0; x < steps; x++) {
        int startTime = millis();

        moveServos(tempo * 0.2, move1);
        delay(tempo * 0.4);
        moveServos(tempo * 0.2, move2);

        int elapsedTime = millis() - startTime;
        if (elapsedTime < tempo) {
            delay(tempo - elapsedTime);
        }
    }
}

void kickLeft(int tempo) {
    Serial.println("kickLeft");
    Otto.home();
    delay(tempo);
    Otto._moveSingle(50, Otto.RightFoot);
    Otto._moveSingle(70, Otto.LeftFoot);
    delay(tempo);
    Otto._moveSingle(80, Otto.RightFoot);
    Otto._moveSingle(70, Otto.LeftFoot);
    delay(tempo / 4);
    Otto._moveSingle(30, Otto.RightFoot);
    Otto._moveSingle(70, Otto.LeftFoot);
    delay(tempo / 4);
    Otto._moveSingle(80, Otto.RightFoot);
    Otto._moveSingle(70, Otto.LeftFoot);
    delay(tempo / 4);
    Otto._moveSingle(30, Otto.RightFoot);
    Otto._moveSingle(70, Otto.LeftFoot);
    delay(tempo / 4);
    Otto._moveSingle(80, Otto.RightFoot);
    Otto._moveSingle(70, Otto.LeftFoot);
    delay(tempo);
}

void kickRight(int tempo) {
    Serial.println("kickRight");
    Otto.home();
    delay(tempo);
    Otto._moveSingle(110, Otto.RightFoot);
    Otto._moveSingle(130, Otto.LeftFoot);
    delay(tempo);
    Otto._moveSingle(110, Otto.RightFoot);
    Otto._moveSingle(100, Otto.LeftFoot);
    delay(tempo / 4);
    Otto._moveSingle(110, Otto.RightFoot);
    Otto._moveSingle(150, Otto.LeftFoot);
    delay(tempo / 4);
    Otto._moveSingle(110, Otto.RightFoot);
    Otto._moveSingle(80, Otto.LeftFoot);
    delay(tempo / 4);
    Otto._moveSingle(110, Otto.RightFoot);
    Otto._moveSingle(150, Otto.LeftFoot);
    delay(tempo / 4);
    Otto._moveSingle(110, Otto.RightFoot);
    Otto._moveSingle(100, Otto.LeftFoot);
    delay(tempo);
}

void walk(int steps, int T) {
    Serial.println("walk");
    int A[4] = {15, 15, 30, 30};
    int O[4] = {0, 0, 0, 0};
    double phase_diff[4] = {DEG2RAD(0), DEG2RAD(0), DEG2RAD(90), DEG2RAD(90)};

    oscillateServos(A, O, T, phase_diff, steps);
}

void run(int steps, int T) {
    Serial.println("run");
    int A[4] = {10, 10, 10, 10};
    int O[4] = {0, 0, 0, 0};
    double phase_diff[4] = {DEG2RAD(0), DEG2RAD(0), DEG2RAD(90), DEG2RAD(90)};

    oscillateServos(A, O, T, phase_diff, steps);
}

void backyard(int steps, int T) {
    Serial.println("backyard");
    int A[4] = {15, 15, 30, 30};
    int O[4] = {0, 0, 0, 0};
    double phase_diff[4] = {DEG2RAD(0), DEG2RAD(0), DEG2RAD(-90), DEG2RAD(-90)};

    oscillateServos(A, O, T, phase_diff, steps);
}

void moonWalkRight(int steps, int T) {
    Serial.println("moonWalkRight");
    int A[4] = {25, 25, 0, 0};
    int O[4] = {-15, 15, 0, 0};
    double phase_diff[4] = {DEG2RAD(0), DEG2RAD(180 + 120), DEG2RAD(90),
                            DEG2RAD(90)};

    oscillateServos(A, O, T, phase_diff, steps);
}

void moonWalkLeft(int steps, int T) {
    Serial.println("moonWalkLeft");
    int A[4] = {25, 25, 0, 0};
    int O[4] = {-15, 15, 0, 0};
    double phase_diff[4] = {DEG2RAD(0), DEG2RAD(180 - 120), DEG2RAD(90),
                            DEG2RAD(90)};

    oscillateServos(A, O, T, phase_diff, steps);
}

void swing(int steps, int T) {
    Serial.println("swing");
    int A[4] = {25, 25, 0, 0};
    int O[4] = {-15, 15, 0, 0};
    double phase_diff[4] = {DEG2RAD(0), DEG2RAD(0), DEG2RAD(90), DEG2RAD(90)};

    oscillateServos(A, O, T, phase_diff, steps);
}

void flapping(int steps, int T) {
    Serial.println("flapping");
    int A[4] = {15, 15, 8, 8};
    int O[4] = {-A[0], A[1], 0, 0};
    double phase_diff[4] = {DEG2RAD(0), DEG2RAD(180), DEG2RAD(90),
                            DEG2RAD(-90)};

    oscillateServos(A, O, T, phase_diff, steps);
}

void drunk(int tempo) {
    Serial.println("drunk");
    int startTime = millis();

    int move1[] = {60, 70, 90, 90};
    int move2[] = {110, 120, 90, 90};
    int move3[] = {60, 70, 90, 90};
    int move4[] = {110, 120, 90, 90};
    int move5[] = {90, 90, 90, 90};

    moveServos(tempo * 0.235, move1);
    moveServos(tempo * 0.235, move2);
    moveServos(tempo * 0.235, move3);
    moveServos(tempo * 0.235, move4);
    moveServos(tempo * 0.06, move5);

    int elapsedTime = millis() - startTime;
    if (elapsedTime < tempo) {
        delay(tempo - elapsedTime);
    }
}

void reverencia1(int steps, int tempo) {
    Serial.println("reverencia1");
    int move1[4] = {130, 50, 90, 90};
    int move2[4] = {90, 90, 90, 90};

    for (int x = 0; x < steps; x++) {
        int startTime = millis();

        Otto.home();
        moveServos(tempo * 0.3, move1);
        delay(tempo * 0.2);
        moveServos(tempo * 0.3, move2);

        int elapsedTime = millis() - startTime;
        if (elapsedTime < tempo) {
            delay(tempo - elapsedTime);
        }
    }
}

void reverencia2(int steps, int tempo) {
    Serial.println("reverencia2");
    int move1[4] = {130, 50, 90, 90};
    int move2[4] = {130, 50, 60, 120};
    int move3[4] = {90, 90, 90, 90};

    for (int x = 0; x < steps; x++) {
        int startTime = millis();

        Otto.home();
        delay(tempo * 0.2);
        moveServos(tempo * 0.05, move1);
        moveServos(tempo * 0.05, move2);
        moveServos(tempo * 0.05, move1);
        moveServos(tempo * 0.05, move2);
        delay(tempo * 0.2);
        moveServos(tempo * 0.1, move3);

        int elapsedTime = millis() - startTime;
        if (elapsedTime < tempo) {
            delay(tempo - elapsedTime);
        }
    }
}

void patada(int tempo) {
    Serial.println("patada");
    Otto.home();

    Otto._moveSingle(115, Otto.RightFoot);
    Otto._moveSingle(120, Otto.LeftFoot);
    delay(tempo / 4);

    Otto._moveSingle(115, Otto.RightFoot);
    Otto._moveSingle(70, Otto.LeftFoot);
    delay(tempo / 4);

    Otto._moveSingle(100, Otto.RightFoot);
    Otto._moveSingle(80, Otto.LeftFoot);
    delay(tempo / 4);

    Otto._moveSingle(90, Otto.RightFoot);
    Otto._moveSingle(90, Otto.LeftFoot);
    delay(tempo / 4);
}

void twist(int steps, int tempo) {
    Serial.println("twist");
    int move1[4] = {90, 90, 50, 130};
    int move2[4] = {90, 90, 90, 90};

    for (int x = 0; x < steps; x++) {
        int startTime = millis();

        moveServos(tempo * 0.1, move1);
        moveServos(tempo * 0.1, move2);

        int elapsedTime = millis() - startTime;
        if (elapsedTime < tempo) {
            delay(tempo - elapsedTime);
        }
    }
}

void lateral_fuerte(boolean side, int tempo) {
    Serial.println("lateral_fuerte");
    Otto.home();
    if (side) {
        Otto._moveSingle(40, Otto.RightFoot);
    } else {
        Otto._moveSingle(140, Otto.RightFoot);
    }
    delay(tempo / 2);
    Otto._moveSingle(90, Otto.RightFoot);
    Otto._moveSingle(90, Otto.LeftFoot);
    delay(tempo / 2);
}

void saludo(int steps, int tempo) {
    Serial.println("saludo");
    int move1[4] = {60, 60, 90, 90};
    int move2[4] = {120, 60, 90, 90};

    for (int x = 0; x < steps; x++) {
        int startTime = millis();

        Otto.home();
        moveServos(tempo * 0.25, move1);
        moveServos(tempo * 0.25, move2);
        moveServos(tempo * 0.25, move1);
        moveServos(tempo * 0.25, move2);

        int elapsedTime = millis() - startTime;
        if (elapsedTime < tempo) {
            delay(tempo - elapsedTime);
        }
    }
}

void dance(int t) {
    pasitos(8, t * 2);
    crusaito(1, t);
    patada(t);
    delay(t);
    twist(2, t);
    twist(3, t / 2);
    upDown(1, t * 2);
    patada(t * 2);
    drunk(t * 2);
    flapping(1, t * 2);
    walk(2, t);
    walk(1, t * 2);
    backyard(2, t);
    patada(t * 2);
    flapping(1, t * 2);
    patada(t * 2);
    twist(8, t / 2);
    moonWalkLeft(2, t);
    crusaito(1, t * 2);

    for (int i = 0; i < 2; i++) {
        lateral_fuerte(0, t);
        lateral_fuerte(1, t);
        upDown(1, t * 2);
    }

    saludo(1, t * 2);
    saludo(1, t);
    delay(t);
    swing(3, t);

    Otto.home();
    delay(t);

    lateral_fuerte(0, t);
    lateral_fuerte(1, t);
    lateral_fuerte(0, t / 2);
    lateral_fuerte(1, t / 2);
    lateral_fuerte(0, t / 2);
    delay(t / 2);
    lateral_fuerte(0, t);
    lateral_fuerte(1, t);
    lateral_fuerte(0, t / 2);
    lateral_fuerte(1, t / 2);
    delay(t);

    pasitos(1, t * 2);
    pasitos(1, t);
    delay(t / 2);
    pasitos(1, t * 2);
    pasitos(1, t);
    delay(t / 2);

    crusaito(2, t);
    crusaito(1, t * 2);
    crusaito(2, t);
    crusaito(1, t * 2);
    crusaito(2, t);
    crusaito(1, t * 2);

    upDown(2, t);
    crusaito(1, t * 2);

    Otto.home();
    delay(t / 2);

    pasitos(2, t * 2);
    pasitos(2, t);
    flapping(1, t * 2);
    upDown(2, t);
    upDown(1, t * 2);

    for (int i = 0; i < 4; i++) {
        pasitos(1, t);
        delay(t);
    }
    reverencia1(1, t * 4);
    reverencia2(1, t * 4);
    upDown(1, t);
    run(2, t / 2);
    patada(t * 2);

    lateral_fuerte(0, t);
    lateral_fuerte(1, t);
    upDown(2, t);
    lateral_fuerte(0, t);
    lateral_fuerte(1, t);
    upDown(2, t);
    pasitos(4, t);
    lateral_fuerte(0, t);
    lateral_fuerte(1, t);
    upDown(2, t);

    patada(t * 2);
    pasitos(2, t);
    patada(t * 2);
    pasitos(2, t);
    swing(2, t * 2);
    pasitos(4, t);

    for (int i = 0; i < 4; i++) {
        lateral_fuerte(0, t);
        lateral_fuerte(1, t);
        lateral_fuerte(0, t / 2);
        lateral_fuerte(1, t / 2);
        lateral_fuerte(0, t / 2);
        delay(t / 2);
    }

    pasitos(6, t);
    delay(t);
    pasitos(1, t);
    delay(t / 2);
    pasitos(3, t);
    delay(t / 2);
    swing(4, t);

    twist(2, t / 2);
    delay(t / 2);
    twist(2, t / 2);
    delay(t / 2);

    drunk(t * 2);
    drunk(t / 2);
    drunk(t * 2);
    delay(t / 2);
    walk(1, t);
    backyard(1, t);

    Otto._moveSingle(110, Otto.RightFoot);
    Otto._moveSingle(130, Otto.LeftFoot);
    delay(t);

    crusaito(3, t);
    crusaito(1, 2 * t);
    upDown(1, t * 2);
    upDown(2, t / 2);

    kickLeft(t / 2);
    kickRight(t / 2);
    moonWalkLeft(1, t * 2);
    moonWalkLeft(2, t);
    moonWalkRight(1, t * 2);
    moonWalkRight(2, t);

    walk(4, t);
    backyard(4, t);

    lateral_fuerte(0, t);
    lateral_fuerte(0, t);
    lateral_fuerte(1, t);
    lateral_fuerte(1, t);
    walk(2, t);
    backyard(2, t);

    pasitos(6, t * 2);
    swing(1, t);
    upDown(1, t);
    delay(t);
    upDown(6, t);
    delay(t);

    for (int i = 0; i < 4; i++) {
        lateral_fuerte(0, t);
        lateral_fuerte(1, t);
    }

    delay(t);
    for (int i = 0; i < 7; i++) {
        pasitos(2, t);
        swing(2, t);
    }

    pasitos(1, t);
    crusaito(1, t * 2);
    upDown(1, t);

    delay(2000);
}