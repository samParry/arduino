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
const int pin_motor_w1 = 31;
const int pin_motor_w2 = 30;

const int pin_servo_w = 45;
const int pin_servo_c = 44;
const int pin_servo_h = 46;

// *Servo variables*
Servo servoW;
Servo servoC;
Servo servoH;
 
// *Motor variables*
DualTB9051FTGMotorShieldUnoMega mshield;

// *State Variables*
String state = "instrument";

void setup() {

  // *Configure pins*
  servoW.attach(pin_servo_w);
  servoC.attach(pin_servo_c);
  servoH.attach(pin_servo_h);
  servoW.write(0);
  servoC.write(180);  // 180 - 110 degrees
  servoH.write(0);

  pinMode(pin_motor_w1, OUTPUT);
  pinMode(pin_motor_w2, OUTPUT);
  
  // *Initialize communication*
//  Serial.begin(9600);

// *Initialize motor driver*
  mshield.init();
  mshield.enableDrivers();
  mshield.flipM2(true);
}

void loop() {
  servoC.write(110);
  delay(1000);
  servoC.write(180);
  delay(1000);
//  if (state == "instrument") {
//    test_instrument_motor(pin_motor_w1, pin_motor_w2);
//    state = "drive";
//  }
//  if (state == "drive") {
//    test_drive_motors(mshield);
//    state = "turn";
//  }
//  if (state == "turn") {
//    test_turn(mshield);
//    state = "servos";
//  }
//  if (state == "servos") {
//    test_servos(servoW, servoC, servoH);
//    state = "instrument";
//  }
}

/****************************
 ** User-Defined Functions **
 ****************************/

void test_instrument_motor(int pin_motor_w1, int pin_motor_w2) {
  instrument_motor_d1(pin_motor_w1, pin_motor_w2);
  delay(1000);
  instrument_motor_d2(pin_motor_w1, pin_motor_w2);
  delay(1000);
  digitalWrite(pin_motor_w1, LOW);
  digitalWrite(pin_motor_w2, LOW);
}

void instrument_motor_d1(int pin_motor_w1, int pin_motor_w2) {
  digitalWrite(pin_motor_w1, HIGH);
  digitalWrite(pin_motor_w2, LOW);
}

void instrument_motor_d2(int pin_motor_w1, int pin_motor_w2) {
  digitalWrite(pin_motor_w1, LOW);
  digitalWrite(pin_motor_w2, HIGH);
}

void test_servos(Servo servo1, Servo servoC, Servo servo3){
  int angle1 = 0;
  int angle2 = 90;
  
  servo1.write(angle2);
  delay(1000);
  servo1.write(angle1);
  delay(1000);
  servoC.write(150);
  delay(1000);
  servoC.write(180);
  delay(1000);
  servo3.write(angle2);
  delay(1000);
  servo3.write(angle1);
  delay(1000);
}

void test_drive_motors(DualTB9051FTGMotorShieldUnoMega mshield){
  // Turn drive motors in both directions for 1 second
  int motor_power = 400;
  mshield.setM1Speed(motor_power);
  delay(1000);
  mshield.setM1Speed(0);
  delay(1000);
  mshield.setM1Speed(-motor_power);
  delay(1000);
  mshield.setM1Speed(0);
  delay(1000);
  mshield.setM2Speed(motor_power);
  delay(1000);
  mshield.setM2Speed(0);
  delay(1000);
  mshield.setM2Speed(-motor_power);
  delay(1000);
  mshield.setM2Speed(0);
  delay(1000);
}

void test_turn(DualTB9051FTGMotorShieldUnoMega mshield) {
  int power = 400;
  
  // Turn 1
  mshield.setM1Speed(power);
  mshield.setM2Speed(-power);
  delay(1000);
  mshield.setM1Speed(0);
  mshield.setM2Speed(0);
  delay(1000);
  
  // Turn 2
  mshield.setM1Speed(-power);
  mshield.setM2Speed(power);
  delay(1000);
  mshield.setM1Speed(0);
  mshield.setM2Speed(0);
  delay(1000);
}




 
