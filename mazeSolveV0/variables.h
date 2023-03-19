#include "Arduino.h"

#define IR1 A0
#define IR2 A1
#define IR3 A2
#define IR4 A3
#define IR5 A4
#define IR6 A5
#define IR7 A6
#define IR8 A7

#define LL A8       //Left sensor panel
#define LM A9
#define LR A10

#define RL A11      //Right sensor panel
#define RM A12
#define RR A13

#define IRColorSensor A15

#define RMotorA 2
#define RMotorB 3
#define RMotorPWM 9

#define LMotorA 4
#define LMotorB 5
#define LMotorPWM 10

#define MAX_SPEED 130



int MotorBaseSpeed = 100;
int IR_val[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
float IR_weights[8] = { -8, -4, -1.5, -1, 1, 1.5, 4, 8};
int LMotorSpeed = 0;
int RMotorSpeed = 0;
int speedAdjust = 0;

float P, I, D;
float error = 0;
float previousError = 0;
float Kp = 4;
float Kd = 1;
float Ki = 0;

int IR_sensors[6] = {0, 0, 0, 0, 0, 0};

int squareCount = 0;  // Example value between 0 and 10

int cubePattern[6] = {0, 0, 0, 0, 0, 0};




int validPatterns[20][6] = {
  {0, 0, 1, 0, 1, 1},
  {0, 0, 1, 1, 0, 1},
  {0, 0, 1, 1, 1, 0},
  {0, 1, 0, 0, 1, 1},
  {0, 1, 0, 1, 0, 1},
  {0, 1, 0, 1, 1, 0},
  {0, 1, 1, 0, 0, 1},
  {0, 1, 1, 0, 1, 0},
  {0, 1, 1, 1, 0, 0},
  {1, 0, 0, 0, 1, 1},
  {1, 0, 0, 1, 0, 1},
  {1, 0, 0, 1, 1, 0},
  {1, 0, 1, 0, 0, 1},
  {1, 0, 1, 0, 1, 0},
  {1, 0, 1, 1, 0, 0},
  {1, 1, 0, 0, 0, 1},
  {1, 1, 0, 0, 1, 0},
  {1, 1, 0, 1, 0, 0},
  {1, 1, 1, 0, 0, 0},
  {1, 1, 0, 1, 1, 0},
};





