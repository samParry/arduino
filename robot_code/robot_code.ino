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
const int pin_motor_w1 = 3;
const int pin_motor_w2 = 5;

const int pin_servo_w = 46;
const int pin_servo_c = 45;
const int pin_servo_h = 44;

// *Servo variables*
Servo servoW;
Servo servoC;
Servo servoH;
 
// *Motor variables*
DualTB9051FTGMotorShieldUnoMega mshield;

// *State Variables*
String state = "Default";

// *Time Variable*
double t;

void setup() {

  // *Configure pins*
  servoW.attach(pin_servo_w);
  servoC.attach(pin_servo_c);
  servoH.attach(pin_servo_h);
  servoW.write(0);
  servoC.write(170);  // 180 - 110 degrees
  servoH.write(90);

  pinMode(pin_motor_w1, OUTPUT);
  pinMode(pin_motor_w2, OUTPUT);s
  
  // *Initialize communication*
  Serial2.begin(9600);

  // *Initialize motor driver*
  mshield.init();
  mshield.enableDrivers();
  mshield.flipM2(true);
}

void loop() {
  if (Serial2.available()){                   // message sent from Uno
    state = Serial2.readStringUntil('\n');    // read a single character sent from Uno
    Serial2.read();                           // clear new line character from buffer
    delay(10);
  }
  if (state == "instrument") {
    test_instrument_motor(pin_motor_w1, pin_motor_w2);
    //state = "drive";
  }
  else if (state == "drive") {
    test_drive_motors(mshield);
    //state = "turn";
  }
  else if (state == "turn") {
    test_turn(mshield);
    //state = "servos";
  }
  else if (state == "servos") {
    test_servos(servoW, servoC, servoH);
    //state = "instrument";
  }
}

/****************************
 ** User-Defined Functions **
 ****************************/

void test_instrument_motor(int pin_motor_w1, int pin_motor_w2) {
  instrument_motor_d1(pin_motor_w1, pin_motor_w2);
  delay(2000);
  instrument_motor_d2(pin_motor_w1, pin_motor_w2);
  delay(2000);
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

void test_servos(Servo servoW, Servo servoC, Servo servoH){
  servoW.write(45);
  delay(1000);
  servoW.write(0);
  delay(1000);
  servoC.write(110);
  delay(1000);
  servoC.write(170);
  delay(1000);
  servoH.write(120);
  delay(1000);
  servoH.write(90);
  delay(1000);
}

void test_drive_motors(DualTB9051FTGMotorShieldUnoMega mshield){
  // Turn drive motors in both directions for 1 second
  int motor_power = 250;
  mshield.setM1Speed(motor_power);
  mshield.setM2Speed(motor_power);
  delay(1000);
  mshield.setM1Speed(0);
  mshield.setM2Speed(0);
  delay(1000);
  mshield.setM1Speed(-motor_power);
  mshield.setM2Speed(-motor_power);
  delay(1000);
  mshield.setM1Speed(0);
  mshield.setM2Speed(0);
  delay(1000);
}

void test_turn(DualTB9051FTGMotorShieldUnoMega mshield) {
  int power = 250;
  
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
  delay(2000);
  mshield.setM1Speed(0);
  mshield.setM2Speed(0);
  delay(1000);
}
 
