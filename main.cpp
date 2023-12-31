#include <SparkFun_TB6612.h>

// Motor configuration pins
#define AIN1 0
#define BIN1 4
#define AIN2 16
#define BIN2 17
#define PWMA 5
#define PWMB 18
#define STBY 19

// QTR Sensor pins
const int sensorPins[7] = {13,12,14,27,26,25,33}; // sensor pins
const int numSensors = 7;

// Initializing motors
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// PID constants
float Kp = 0.087;
float Ki = 0;
float Kd = 0;

// Sensor data
int minValues[numSensors];
int maxValues[numSensors];
int threshold[numSensors];
int sensorValue[numSensors];

// Line-following variables
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 250;

void setup() {
  Serial.begin(115200);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, OUTPUT);

  for (int i = 0; i < numSensors; i++) {
    pinMode(sensorPins[i], INPUT);
  }
}

void loop() {
  delay(1000);
  calibrate();
  delay(1000);

  while (1) {
    readLine();
  if (sensorValue[0] > 900 && sensorValue[1] < 300 && sensorValue[3] > 800 && 
      digitalWrite(13, HIGH);
      motor1.drive(100);
      motor2.drive(100);
   
      delay(235);
       brake(motor1,motor2);
      while (analogRead(0) < threshold[0]) {
        motor1.drive(100);
        motor2.drive(-100);
      }
      digitalWrite(13, LOW);

    }

 
    else if (sensorValue[6] > 900 && sensorValue[5] < 300 && sensorValue[3] > 800 && sensorValue[0] < 500) {
      digitalWrite(13, HIGH);
      motor1.drive(100);
      motor2.drive(100);
       
      delay(235);
       brake(motor1,motor2);
      while (analogRead(6) < threshold[6]) {
        motor1.drive(-100);
        motor2.drive(100);
      }
      digitalWrite(13, LOW);
    }

    else if (sensorValue[0] > 500 && sensorValue[6] < 500) {
      motor1.drive(0);
      motor2.drive(lfspeed);
    }

    else if (sensorValue[1] > 500 && sensorValue[5] < 500) {
      motor1.drive(30);
      motor2.drive(lfspeed);
    }

    else if (sensorValue[6] > 500 && sensorValue[0] < 500) {
      motor1.drive(lfspeed);
      motor2.drive(0);
      digitalWrite(13, LOW);
    }

    else if (sensorValue[5] > 500 && sensorValue[1] < 500) {
      motor1.drive(lfspeed);
      motor2.drive(30);
      digitalWrite(13, LOW);
    }

    else if (sensorValue[3] > 500) {
      linefollow();
      digitalWrite(13, LOW);
    }
          else if (sensorValue[0]<500&&sensorValue[1]<500&&sensorValue[2]<500&&sensorValue[3]<500&&sensorValue[4]<500&&sensorValue[5]<500&&sensorValue[6]<500&&sensorValue[7]<500){
        digitalWrite(13, HIGH);
   
             delay(60);
               brake(motor1,motor2);
            motor1.drive(100);
             motor2.drive(-100);
           
             
              digitalWrite(13, LOW);
           }
   
      }
   
  }

  }
}

void linefollow() {
  int error = (sensorValue[2] - sensorValue[4]);

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < 0) {
    rsp = 0;
  }
  motor1.drive(lsp);
  motor2.drive(rsp);
}



void calibrate() {
  for (int i = 0; i < 7; i++) {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }

  for (int i = 0; i < 1500; i++) {
    motor1.drive(50);
    motor2.drive(-50);

    for (int i = 0; i < 7; i++) {
      if (analogRead(i) < minValues[i]) {
        minValues[i] = analogRead(i);
      }
      if (analogRead(i) > maxValues[i]) {
        maxValues[i] = analogRead(i);
      }
    }
  }

  for (int i = 0; i < 7; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print("   ");
  }
  Serial.println();

  motor1.drive(0);
  motor2.drive(0);
}

void readLine() {
  for (int i = 0; i < 7; i++) {
    sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1024);
    sensorValue[i] = constrain(sensorValue[i], 0, 1024);
  }
}
