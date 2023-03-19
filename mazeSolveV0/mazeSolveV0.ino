/*
TODO: 
Sensor defines                                                   Done
linefollow to lefthand rule                                      Done
record the color of the box ONLY in 15cm square                   
if 6 squares detected you have to follow instructions
*/
#include "variables.h"

void PID_control();
void read_IR();
void set_speed();
void set_forward();

////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
  pinMode(IR6, INPUT);
  pinMode(IR7, INPUT);
  pinMode(IR8, INPUT);

  //pins for IR that takes decisions
  pinMode(LL, INPUT);
  pinMode(LM, INPUT);
  pinMode(LR, INPUT);
  pinMode(RL, INPUT);
  pinMode(RM, INPUT);
  pinMode(RR, INPUT);

  // LL LM LR IR1 IR2 IR3 IR4 IR5 IR6 IR7 IR8 RL RM RR

  pinMode(IRColorSensor, INPUT);

  pinMode(LMotorA, OUTPUT);
  pinMode(LMotorB, OUTPUT);
  pinMode(LMotorPWM, OUTPUT);

  pinMode(RMotorA, OUTPUT);
  pinMode(RMotorB, OUTPUT);
  pinMode(RMotorPWM, OUTPUT);

  set_forward();  // This is like a headstart. replace it with moveOneInch()
  delay(2000);
}




void loop() {
  read_IR();
  if (IR_val[0] == 0 && IR_val[1] == 0 && IR_val[2] == 0 && IR_val[3] == 0 && IR_val[4] == 0 && IR_val[5] == 0 && IR_val[6] == 0 && IR_val[7] == 0) {
    Serial.print("Stoped");
    stop();
    while (1) {}
  }

  lineFollow();
}

void lineFollow(){
  PID_control();
  set_speed();
}

void PID_control() {
  error = 0;

  for (int i = 0; i < 8; i++) {
    error += IR_weights[i] * IR_val[i];
  }

  P = error;
  I = I + error;
  D = error - previousError;

  previousError = error;

  speedAdjust = (Kp * P + Ki * I + Kd * D);

  LMotorSpeed = MotorBaseSpeed - speedAdjust;  // Need to Test IT
  RMotorSpeed = MotorBaseSpeed + speedAdjust;  // Change Positive or Negative

  if (LMotorSpeed < 0) {
    LMotorSpeed = 0;
  }

  if (RMotorSpeed < 0) {
    RMotorSpeed = 0;
  }

  if (LMotorSpeed > MAX_SPEED) {
    LMotorSpeed = MAX_SPEED;
  }

  if (RMotorSpeed > MAX_SPEED) {
    RMotorSpeed = MAX_SPEED;
  }

  // Debug Purpose
  for (int i = 0; i < 6; i++) {
    Serial.print(IR_val[i]);
    Serial.print(" ");
  }

  Serial.print("Error");
  Serial.print('\t');
  Serial.print(error);
  Serial.print('\t');

  Serial.print('P');
  Serial.print('\t');
  Serial.print(P);
  Serial.print('\t');

  Serial.print('I');
  Serial.print('\t');
  Serial.print(I);
  Serial.print('\t');

  Serial.print('D');
  Serial.print('\t');
  Serial.print(D);
  Serial.print('\t');

  Serial.print("Left");
  Serial.print('\t');
  Serial.print(LMotorSpeed);
  Serial.print('\t');

  Serial.print("Right");
  Serial.print('\t');
  Serial.print(RMotorSpeed);
  Serial.print('\t');

  Serial.print('\n');
}

void read_IR() {
  IR_val[0] = digitalRead(IR1);
  IR_val[1] = digitalRead(IR2);
  IR_val[2] = digitalRead(IR3);
  IR_val[3] = digitalRead(IR4);
  IR_val[4] = digitalRead(IR5);
  IR_val[5] = digitalRead(IR6);
  IR_val[6] = digitalRead(IR7);
  IR_val[7] = digitalRead(IR8);

  IR_sensors[0] = digitalRead(LL);
  IR_sensors[1] = digitalRead(LM);
  IR_sensors[2] = digitalRead(LR);
  IR_sensors[3] = digitalRead(RL);
  IR_sensors[4] = digitalRead(RM);
  IR_sensors[5] = digitalRead(RR);

}

void straight(){
  while (1) {
    Serial.println("Going Forward and Following the Line");
    lineFollow();
    if (IR_sensors[0] == 0 && IR_sensors[1] == 0 && IR_sensors[2] == 0 && IR_sensors[3] == 0 && IR_sensors[4] == 0 && IR_sensors[5] == 0) {
      break;
    }
  }
}

void mazeFollow(){
  while (squareCount != 6) {
    if (1 /* condition to detect ONLY 15cm square */) {
      stop();
      detectColor();
      turnback180();
      squareCount++; 
    } else if (1 /* condition to detect 20cm square or a cross (All High)*/) {
      moveOneInch();
      if (1 /* condition to detect 20cm square (All High)*/) {
        turnback180();
      } else {
        turnLeft90(); // because we use left-Hand Rule
      }
    } else {
      lineFollow();
    }
  }
}

void solveMaze(){
  
}

void pickTheCube(){

}

void replaceCube(){

}

void dropCube(){

}

void set_speed() {
  analogWrite(LMotorPWM, LMotorSpeed);
  analogWrite(RMotorPWM, RMotorSpeed);
}

void set_forward() {
  digitalWrite(LMotorB, HIGH);
  digitalWrite(LMotorA, LOW);

  digitalWrite(RMotorB, HIGH);
  digitalWrite(RMotorA, LOW);
}

void stop() {
  digitalWrite(LMotorB, LOW);
  digitalWrite(LMotorA, LOW);

  digitalWrite(RMotorB, LOW);
  digitalWrite(RMotorA, LOW);
}

void turnLeft90(){

}

void turnRight90(){

}

void turnback180() {
  turnLeft90();
  turnLeft90();
}

void moveOneInch() {

}

void detectColor() {
  // using the servo arm detect the color
  // store it in an array

  //code for sevo movements
  //read the Ir sensor
  //if black -> store "0" in the array in corresponding index
  //if white -> store "1" in the array in corresponding index
  delay(100);
  int irValue = digitalRead(IRColorSensor);
  if (irValue == LOW) {              // if black
    cubePattern[squareCount] = 0;                  // store 0 in the array
  } else {                           // if white
    cubePattern[squareCount] = 1;                  // store 1 in the array
  }
  //code for servo movements
  // Serial.println(squareCount);
  delay(500);  
}
