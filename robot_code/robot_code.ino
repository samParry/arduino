/***********************************************************
 Author: Sam Parry u1008557

 Pin Usage:  pin type/number     hardware
            ----------------   -----------------------------
            
 **********************************************************/

/****************************
 ** #defines and #includes **
 ****************************/
#include <Servo.h>
#include "DualTB9051FTGMotorShieldUnoMega.h"

/***********************
 ** Global Variables ***
 ***********************/
// *Declare pins*
const int pin_motor_w1 = 9;
const int pin_motor_w2 = 10;

const int pin_servo_w = 3;
const int pin_servo_c = 5;
const int pin_servo_h = 11;

// *Servo variables*
Servo servoW;
Servo servoC;
Servo servoH;
 
// *Motor variables*
DualTB9051FTGMotorShieldUnoMega mshield;

void setup() {

  // *Configure pins*
  servoW.attach(pin_servo_w);
  servoC.attach(pin_servo_c);
  servoH.attach(pin_servo_h);

  pinMode(pin_motor_w1, OUTPUT);
  pinMode(pin_motor_w2, OUTPUT);
  
  // *Initialize communication*
  Serial.begin(9600);
  Serial.write('Lets Begin');

// *Initialize motor driver*
  mshield.init();
  mshield.enableDrivers();
  mshield.flipM2(true);
}

void loop() {
//  test_instrument_motor(pin_motor_w1, pin_motor_w2);
//  test_servos(servoW, servoC, servoH);
//  test_drive_motors(mshield);
  test_turn(mshield);
}

/****************************
 ** User-Defined Functions **
 ****************************/

void test_instrument_motor(int pin_motor_w1, int pin_motor_w2) {
  instrument_motor_d1(pin_motor_w1, pin_motor_w2);
  delay(2000);
  instrument_motor_d2(pin_motor_w1, pin_motor_w2);
  delay(2000);
}

void instrument_motor_d1(int pin_motor_w1, int pin_motor_w2) {
  digitalWrite(pin_motor_w1, HIGH);
  digitalWrite(pin_motor_w2, LOW);
}

void instrument_motor_d2(int pin_motor_w1, int pin_motor_w2) {
  digitalWrite(pin_motor_w1, LOW);
  digitalWrite(pin_motor_w2, HIGH);
}

void test_servos(Servo servo1, Servo servo2, Servo servo3){
  int angle1 = 90;
  int angle2 = 170;
  
  float t = micros() / 1000000.0;
  if (0 < t && t < 2) {
    servo1.write(angle2);
  }
  if (3 < t && t < 5) {
    servo1.write(angle1);
  }
  if (6 < t && t < 8) {
    servo2.write(angle2);
  }
  if (9 < t && t < 11) {
    servo2.write(angle1);
  }
  if (12 < t && t < 14) {
    servo3.write(angle2);
  }
  if (15 < t && t < 17) {
    servo3.write(angle1);
  }
}

void test_drive_motors(DualTB9051FTGMotorShieldUnoMega mshield){
  // Turn drive motors in both directions for 1 second
  double t = micros() / 1000000.0; // current time
  int motor_power = 400;

  // Turn motor 1 on in both directions
  if (0 < t && t < 1) {
    mshield.setM1Speed(motor_power);
  }
  if (1 < t && t < 2) {
    mshield.setM1Speed(0);
  }
  if (2 < t && t < 3) {
    mshield.setM1Speed(-motor_power);
  }
  if (3 < t && t < 4) {
    mshield.setM1Speed(0);
  }

  // Turn motor 2 on in both directions
  if (4 < t && t < 5) {
    mshield.setM2Speed(motor_power);
  }
  if (5 < t && t < 6) {
    mshield.setM2Speed(0);
  }
  if (6 < t && t < 7) {
    mshield.setM2Speed(-motor_power);
  }
  if (7 < t && t < 8) {
    mshield.setM2Speed(0);
  }
}

void test_turn(DualTB9051FTGMotorShieldUnoMega mshield) {
  double t = micros() / 1000000.0; // current time
  int power = 400;

  // Turn 1
  if (1 < t && t < 2){
    mshield.setM1Speed(power);
    mshield.setM2Speed(-power);
  } 
  if (2.1 < t && t < 2.9) {
    mshield.setM1Speed(0);
    mshield.setM2Speed(0);
  }
  
  // Turn Back
  if (3 < t && t < 4){
    mshield.setM1Speed(-power);
    mshield.setM2Speed(power);
  } 
  if (4.1 < t && t < 5) {
    mshield.setM1Speed(0);
    mshield.setM2Speed(0);
  }
}




 
