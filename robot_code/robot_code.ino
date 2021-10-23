/***********************************************************
 Author: Sam Parry u1008557

 Pin Usage:  pin type/number     hardware
            ----------------   -----------------------------
            
 **********************************************************/

/****************************
 ** #defines and #includes **
 ****************************/
#include <Servo.h>;
#include <SharpDistSensor.h>;
#include <QTRSensors.h>;
#include "DualTB9051FTGMotorShieldUnoMega.h";

/***********************
 ** Global Variables ***
 ***********************/
// *Declare pins*
const int pin_motor_w1 = 3;
const int pin_motor_w2 = 5;

const int pin_servo_w = 46;
const int pin_servo_c = 45;
const int pin_servo_h = 44;

const uint8_t pins_qtr1[] = {22, 24, 26, 30, 32, 34, 38, 40};
const uint8_t pins_qtr2[] = {23, 25, 27, 31, 33, 35, 39, 41};

const byte pin_sharpL = A10;
const byte pin_sharpR = A11;
const byte pin_sharpF = A12;

const byte pin_color = A5;
const int pin_red = 48;
const int pin_green = 50;
const int pin_blue = 52;

// *Reflectant Sensor Variables*
QTRSensors qtr1;
QTRSensors qtr2;
int16_t qtr1_biases[] = {1205, 968, 968, 917, 1014, 1004, 1110, 1251};
int16_t qtr2_biases[] = {1250, 1076, 1059, 918, 968, 1098, 1011, 1158};
int16_t qtr1_vals[8];
int16_t qtr2_vals[8];

// *Sharp IR Variables*
bool stop_ir = false;
double last_error = 0, total_error = 0;
SharpDistSensor sharpL(pin_sharpL, 1);
SharpDistSensor sharpR(pin_sharpR, 1);
SharpDistSensor sharpF(pin_sharpF, 1);

// *Servo variables*
Servo servoW;
Servo servoC;
Servo servoH;
 
// *Motor variables*
DualTB9051FTGMotorShieldUnoMega mshield;
double base_speed = 100;
double pid_scaling = base_speed/100;
double m1_speed = 250;
double m2_speed = 250;

// *State Variables*
String state = "";
bool going_forward = true;

// *Time Variable*
double t;
unsigned long last_time = 0;

void setup() {

  // *Configure pins*
  servoW.attach(pin_servo_w);
  servoC.attach(pin_servo_c);
  servoH.attach(pin_servo_h);
  servoW.write(0);
  servoC.write(170);  // 180 - 110 degrees
  servoH.write(90);
  pinMode(pin_motor_w1, OUTPUT);
  pinMode(pin_motor_w2, OUTPUT);
  pinMode(pin_red, OUTPUT);
  pinMode(pin_green, OUTPUT);
  pinMode(pin_blue, OUTPUT);
  
  // *Initialize communication*
  Serial2.begin(9600);

  // *Initialize Sensors*
  qtr1.setTypeRC();
  qtr2.setTypeRC();
  qtr1.setSensorPins(pins_qtr1, 8);
  qtr2.setSensorPins(pins_qtr2, 8);
  sharpL.setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);
  sharpR.setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);
  sharpF.setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);

  // *Initialize motor driver*
  mshield.init();
  mshield.enableDrivers();
  mshield.flipM2(true);
  Serial2.println("Mega Ready");
//  calibrate_qtrs();
}

void loop() {
  if (Serial2.available()){                       // message sent from Uno
    state = Serial2.readStringUntil('\n');        // read a String sent from Uno
    state = state.substring(0, state.length()-1); // trim trailing white space
    Serial2.read();                               // clear new line character from buffer
    delay(10);
    Serial2.print("state set to: ");
    Serial2.println(state);
  }

  // Color sensor
  if (state == "c") {
    read_color();
    state = "";
  }

  // Sharp IR
  if (state == "ir") {
   going_forward = true;
   follow_wall(0.9, 0.9);
  }

  float Kp = 40;
  float Kd = 20;
  // Line Following
  if (state == "linef") {
    going_forward = true;
    follow_line(Kp, Kd);
  }
  else if (state == "lineb") {
    going_forward = false;
    follow_line(Kp, Kd);
  }

  // Kill switch
  if (state == "s") {
    stop_motors();
    state = "";
  }
}

/****************************
 ** User-Defined Functions **
 ****************************/

void read_color() {
  String color;
  int num_reads = 30;
  int red=0, green=0, blue=0;

  // collect color readings
  for (int i = 0; i < num_reads; i++) {

    // read red
    digitalWrite(pin_red, LOW);
    digitalWrite(pin_green, HIGH);
    digitalWrite(pin_blue, HIGH);
    delay(5);
    red += analogRead(pin_color);

    // read green
    digitalWrite(pin_red, HIGH);
    digitalWrite(pin_green, LOW);
    digitalWrite(pin_blue, HIGH);
    delay(5);
    green += analogRead(pin_color);

    // read blue
    digitalWrite(pin_red, HIGH);
    digitalWrite(pin_green, HIGH);
    digitalWrite(pin_blue, LOW);
    delay(5);
    blue += analogRead(pin_color);

    // reset
    digitalWrite(pin_red, LOW);
    digitalWrite(pin_green, LOW);
    digitalWrite(pin_blue, LOW);
  }

  // average the readings
  red = red/num_reads;
  green = green/num_reads;
  blue = blue/num_reads;

  // determine color
  if (red >= green && red >= (blue-10)) {
    color = "red";
  }
  else if (green >= red && green >= (blue-10)) {
    color = "green";
  }
  else if (blue >= red && blue >= (green-10)) {
    color = "blue";
  }
  Serial2.print("color: ");
  Serial2.println(color);
}

void calibrate_qtrs() {
  float sums1[] = {0, 0, 0, 0, 0, 0, 0, 0};
  float sums2[] = {0, 0, 0, 0, 0, 0, 0, 0};
  float error1, error2;
  int num_reads = 20;

  // record sensor values
  for (int i = 0; i < num_reads; i++) {
    going_forward = true;
    error1 = line_error();
    going_forward = false;
    error2 = line_error();
    for (int i = 0; i < 8; i++) {
      sums1[i] += qtr1_vals[i];
      sums2[i] += qtr2_vals[i];
    }
    delay(50);
  }

  // compute averages
  for (int i = 0; i < 8; i++) {
    qtr1_biases[i] = sums1[i]/num_reads;
    qtr2_biases[i] = sums2[i]/num_reads;
  }

  // print biases
  Serial2.println("qtr1 biases");
  for (int i = 0; i < 8; i++) {
    Serial2.print(qtr1_biases[i]);
    Serial2.print('\t');
  }
  Serial2.println();
  
  Serial2.println("qtr2 biases");
  for (int i = 0; i < 8; i++) {
    Serial2.print(qtr2_biases[i]);
    Serial2.print('\t');
  }
  Serial2.println();
}

void stop_motors() {
  mshield.setM1Speed(0);
  mshield.setM2Speed(0);
  digitalWrite(pin_motor_w1, LOW);
  digitalWrite(pin_motor_w2, LOW);
}

void follow_wall(float Kp, float Kd) {
  int error_p, error_d, error;
  unsigned long current_time = millis();  // # of milliseconds since program start
  int delta_time = current_time - last_time;
  int optimal_dist = 50;   // 50 mm
  
  if (going_forward) {
    error_p = sharpL.getDist() - optimal_dist;
    error_d = (error_p - last_error)/delta_time;
    error = Kp*error_p + Kd*error_d;
    total_error += error;
    
    m1_speed = base_speed + error;
    m2_speed = base_speed - error;
  }
  else {
    error_p = sharpR.getDist() - optimal_dist;
    error_d = (error_p - last_error)/delta_time;
    error = Kp*error_p + Kd*error_d;
    total_error += error;
    
    m1_speed = -(base_speed - error);
    m2_speed = -(base_speed + error);
  }
  Serial2.println(error);
  delay(10);
  mshield.setM1Speed(m1_speed);
  mshield.setM2Speed(m2_speed);
  last_time = current_time;
}

void follow_line(float Kp, float Kd) {
  double error_p, error_d, error;
  unsigned long current_time = millis();  // # of milliseconds since program start
  int delta_time = current_time - last_time;

  // apply pid error scaling factor
  Kp *= pid_scaling;
  Kd *= pid_scaling;

  // read errors
  error_p = line_error();
  error_d = (error_p - last_error)/delta_time;
  error = Kp*error_p + Kd*error_d;
  total_error += error;
  Serial2.print("error_p:\t");
  Serial2.print(error_p);
  Serial2.print("\terror_i\t");
  Serial2.print(error_d);
  Serial2.print("\terror:\t");
  Serial2.print(error);
  Serial2.print("\tdt:\t");
  Serial2.println(delta_time);

  // correct motors by varying speed
  if (going_forward) {
    m1_speed = base_speed - error;
    m2_speed = base_speed + error;
  }
  else {
    m1_speed = -(base_speed + error);
    m2_speed = -(base_speed - error);
  }
  mshield.setM1Speed(m1_speed);
  mshield.setM2Speed(m2_speed);
  last_time = current_time;
}

float line_error() {

  // read front or rear values
  float value_center;
  if (going_forward) {
    qtr1.read(qtr1_vals);
    value_center = 4.25;
  }
  else {
    qtr2.read(qtr2_vals);
    value_center = 4.55;
  }

  // process qtr readings
  float numer = 0;
  float denom = 0;
  float diff = 0;
  for (int i = 0; i < 8; i++) {
    if (going_forward) {
      diff = qtr1_vals[i] - qtr1_biases[i];
    }
    else {
      diff = qtr2_vals[i] - qtr2_biases[i];
    }

    // diff must be positive
    if (diff < 0.01) {
      diff = 0.01;
    }
    numer += diff * (i+1);
    denom += diff;
  }
  float line_local = numer / denom;
  float error = line_local - value_center;
  return error;
}

void test_instrument_motor() {
  instrument_motor_d1();
  delay(2000);
  instrument_motor_d2();
  delay(2000);
  digitalWrite(pin_motor_w1, LOW);
  digitalWrite(pin_motor_w2, LOW);
}

void instrument_motor_d1() {
  digitalWrite(pin_motor_w1, HIGH);
  digitalWrite(pin_motor_w2, LOW);
}

void instrument_motor_d2() {
  digitalWrite(pin_motor_w1, LOW);
  digitalWrite(pin_motor_w2, HIGH);
}

void test_servos(){
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

void test_drive_motors(){
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

void test_turn() {
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
 
