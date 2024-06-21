#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#ifndef DEG2RAD
  #define DEG2RAD(g) ((g)*M_PI)/180
#endif

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // Create PWM instance

#define PCA9685_ADDRESS 0x40 // Address of PCA9685
#define PWM_FREQ 60          // PWM frequency in Hz

#define N_SERVOS 4           // Number of servos
#define SERVO_MIN 150        // Min pulse length out of 4096
#define SERVO_MAX 600        // Max pulse length out of 4096

// Define PCA9685 channels for your servos
#define PIN_RL 0   // Replace with your actual PCA9685 channel assignments
#define PIN_RR 1
#define PIN_YR 2
#define PIN_YL 3

void setup() {
  Serial.begin(9600);

  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);

  // Set initial servo positions
  pwm.setPWM(PIN_RR, 0, SERVO_MIN + (SERVO_MAX - SERVO_MIN) / 2); // Example initial position
  pwm.setPWM(PIN_RL, 0, SERVO_MIN + (SERVO_MAX - SERVO_MIN) / 2);
  pwm.setPWM(PIN_YR, 0, SERVO_MIN + (SERVO_MAX - SERVO_MIN) / 2);
  pwm.setPWM(PIN_YL, 0, SERVO_MIN + (SERVO_MAX - SERVO_MIN) / 2);
}

////////////////////////////////// Control Functions ////////////////////////////////////////

void oscillate(int A[N_SERVOS], int O[N_SERVOS], int T, double phase_diff[N_SERVOS]) {
  unsigned long ref = millis();
  
  for (unsigned long x = ref; x < T + ref; x = millis()) {
    for (int i = 0; i < N_SERVOS; i++) {
      double val = A[i] * sin(2 * PI * ((x - ref) / (double)T) + phase_diff[i]) + O[i];
      int pulse = map(val, 0, 180, SERVO_MIN, SERVO_MAX);
      pwm.setPWM(i, 0, pulse);
    }
  }
}

void moveNServos(int time, int newPosition[]) {
  for (int i = 0; i < N_SERVOS; i++) {
    int pulse = map(newPosition[i], 0, 180, SERVO_MIN, SERVO_MAX); // Map angle to PWM range
    pwm.setPWM(i, 0, pulse);
  }
  delay(time);
}

////////////////////////////////// Dance Steps ////////////////////////////////////////

void pasitos(int steps, int tempo) {
  int move1[4] = {120, 60, 90, 90}; // Example servo positions for 'pasitos'
  int move2[4] = {90, 90, 90, 90};

  for (int i = 0; i < steps; i++) {
    moveNServos(tempo / 4, move1);
    moveNServos(tempo / 4, move2);
    moveNServos(tempo / 4, move1);
    moveNServos(tempo / 4, move2);
    delay(tempo / 2);
  }
}

void crusaito(int steps, int T) {
  int A[4] = {25, 25, 30, 30};
  int O[4] = {- 15, 15, 0, 0};
  double phase_diff[4] = {DEG2RAD(0), DEG2RAD(180 + 120), DEG2RAD(90), DEG2RAD(90)};

  for (int i = 0; i < steps; i++) oscillate(A, O, T, phase_diff);
}

void kickLeft(int tempo) {
  int positions[4][2] = {
    {115, 120},
    {115, 70},
    {100, 80},
    {90, 90}
  };
  
  for (int i = 0; i < 4; ++i) {
    pwm.setPWM(i, 0, map(positions[i][0], 0, 180, SERVO_MIN, SERVO_MAX));
  }
  delay(tempo / 4);
  
  for (int i = 0; i < 4; ++i) {
    pwm.setPWM(i, 0, map(positions[i][0], 0, 180, SERVO_MIN, SERVO_MAX));
  }
  delay(tempo / 4);

  for (int i = 0; i < 4; ++i) {
    pwm.setPWM(i, 0, map(positions[i][0], 0, 180, SERVO_MIN, SERVO_MAX));
  }
  delay(tempo / 4);
  
  for (int i = 0; i < 4; ++i) {
    pwm.setPWM(i, 0, map(positions[i][0], 0, 180, SERVO_MIN, SERVO_MAX));
  }
  delay(tempo / 4);
}

void kickRight(int tempo) {
  int positions[4][2] = {
    {120, 130},
    {120, 100},
    {120, 140},
    {120, 80}
  };

  for (int i = 0; i < 4; ++i) {
    pwm.setPWM(i, 0, map(positions[i][0], 0, 180, SERVO_MIN, SERVO_MAX));
  }
  delay(tempo / 4);

  for (int i = 0; i < 4; ++i) {
    pwm.setPWM(i, 0, map(positions[i][0], 0, 180, SERVO_MIN, SERVO_MAX));
  }
  delay(tempo / 4);

  for (int i = 0; i < 4; ++i) {
    pwm.setPWM(i, 0, map(positions[i][0], 0, 180, SERVO_MIN, SERVO_MAX));
  }
  delay(tempo / 4);

  for (int i = 0; i < 4; ++i) {
    pwm.setPWM(i, 0, map(positions[i][0], 0, 180, SERVO_MIN, SERVO_MAX));
  }
  delay(tempo / 4);
}

void moonWalkRight(int steps, int T) {
  int A[4] = {25, 25, 0, 0};
  int O[4] = {-15 ,15, 0, 0};
  double phase_diff[4] = {DEG2RAD(0), DEG2RAD(180 + 120), DEG2RAD(90), DEG2RAD(90)};

  for (int i = 0; i < steps; i++) oscillate(A, O, T, phase_diff);
}

void moonWalkLeft(int steps, int T) {
  int A[4] = {25, 25, 0, 0};
  int O[4] = {-15, 15, 0, 0};
  double phase_diff[4] = {DEG2RAD(0), DEG2RAD(180 - 120), DEG2RAD(90), DEG2RAD(90)};

  for (int i = 0; i < steps; i++) oscillate(A, O, T, phase_diff);
}

void swing(int steps, int T) {
  int A[4] = {25, 25, 0, 0};
  int O[4] = {-15, 15, 0, 0};
  double phase_diff[4] = {DEG2RAD(0), DEG2RAD(0), DEG2RAD(90), DEG2RAD(90)};

  for (int i = 0; i < steps; i++) oscillate(A, O, T, phase_diff);
}

void flapping(int steps, int T) {
  int A[4] = {15, 15, 8, 8};
  int O[4] = {-A[0], A[1], 0, 0};
  double phase_diff[4] = {DEG2RAD(0), DEG2RAD(180), DEG2RAD(90), DEG2RAD(-90)};

  for (int i = 0; i < steps; i++) oscillate(A, O, T, phase_diff);
}

void loop() {
  pasitos(4, 1000);  // Example: Call 'pasitos' function with 4 steps and 1000ms tempo
  delay(1000);       // Delay between actions
}