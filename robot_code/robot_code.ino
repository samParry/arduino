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

const uint8_t pins_qtr1[] = {23, 25, 27, 29, 31, 33, 35, 37};
const uint8_t pins_qtr2[] = {22, 24, 26, 28, 30, 32, 34, 36};

const byte pin_sharpL = A10;
const byte pin_sharpR = A11;
const byte pin_sharpF = A12;

const byte pin_color = A5;
const int pin_red = 41;
const int pin_green = 42;
const int pin_blue = 43;

// *Reflectant Sensor Variables*
QTRSensors qtr1;
QTRSensors qtr2;
int16_t qtr1_biases[] = {1702, 1413, 1416, 1326, 1461, 1492, 1624, 1750};
int16_t qtr2_biases[] = {2169, 1811, 1710, 1518, 1560, 1793, 1746, 2215};
int16_t qtr1_vals[8];
int16_t qtr2_vals[8];
float ys[] = {1702, 1413, 1416, 1326, 1461, 1492, 1624, 1750};

// *Sharp IR Variables*
bool stop_ir = false;
double last_error = 0, total_error = 0;
unsigned long last_time = 0;
SharpDistSensor sharpL(pin_sharpL, 1);
SharpDistSensor sharpR(pin_sharpR, 1);
SharpDistSensor sharpF(pin_sharpF, 1);

// *Servo variables*
Servo servoW;
Servo servoC;
Servo servoH;
 
// *Motor variables*
DualTB9051FTGMotorShieldUnoMega mshield;
const double base_speed = 100;
double m1_speed = 250;
double m2_speed = 250;

// *State Variables*
String state = "";
bool going_forward = true;

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
}

void loop() {
//  float Kp = 0.9;
//  float Ki = 0.0;
//  float Kd = 0.9;
  if (Serial2.available()){                       // message sent from Uno
    state = Serial2.readStringUntil('\n');        // read a String sent from Uno
    state = state.substring(0, state.length()-1); // trim trailing white space
    Serial2.read();                               // clear new line character from buffer
    delay(10);
    Serial2.print("state set to: ");
    Serial2.println(state);
//    Kp = Serial2.readStringUntil(' ').toFloat();
//    Ki = Serial2.readStringUntil(' ').toFloat();
//    Kd = Serial2.readStringUntil(' ').toFloat();
  }

  // Color sensor
  if (state == "color") {
    read_color();
    state = "";
  }

  // Sharp IR
  if (state == "ir") {
   going_forward = true;
   test_sharpIR();
  }

  // Line Following
  if (state == "line") {
    follow_line(20, 0.0, 20);
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
  int rgb_vals[3] = {0, 0, 0};

  // collect color readings
  for (int i = 0; i < 30; i++) {

    // read red
    digitalWrite(pin_red, LOW);
    digitalWrite(pin_green, HIGH);
    digitalWrite(pin_blue, HIGH);
    delay(1);
    rgb_vals[0] += analogRead(pin_color);

    // read green
    digitalWrite(pin_red, HIGH);
    digitalWrite(pin_green, LOW);
    digitalWrite(pin_blue, HIGH);
    delay(1);
    rgb_vals[1] += analogRead(pin_color);

    // read blue
    digitalWrite(pin_red, HIGH);
    digitalWrite(pin_green, HIGH);
    digitalWrite(pin_blue, LOW);
    delay(1);
    rgb_vals[2] += analogRead(pin_color);
  }

  // average color readings
  for (int i = 0; i < 3; i++) {
    rgb_vals[i] = rgb_vals[i]/30;
  }

  Serial2.println("Red\tGreen\tBlue");
  Serial2.print(rgb_vals[0]);
  Serial2.print("\t");
  Serial2.print(rgb_vals[1]);
  Serial2.print("\t");
  Serial2.println(rgb_vals[2]);
}

void filter_qtr() {
  double alpha = 0.2;
  for (int i = 0; i < 8; i++) {
    ys[i] = alpha*qtr1_vals[i] + (1-alpha)*ys[i];
  }
}

void test_sharpIR() {
  float Kp = 0.9;
  float Ki = 0.0;
  float Kd = 0.9;
  if (state == "stop") {
    stop_motors();
  }
  else {
    follow_wall(Kp, Ki, Kd);
  }
}

void calibrate_qtrs() {
  float sums1[] = {0, 0, 0, 0, 0, 0, 0, 0};
  float sums2[] = {0, 0, 0, 0, 0, 0, 0, 0};
  float error1, error2;
  int num_reads = 20;

  // record sensor values
  for (int i = 0; i < num_reads; i++) {
    error1 = line_error_qtr1();
    error2 = line_error_qtr2();
    for (int i = 0; i < 8; i++) {
      sums1[i] += qtr1_vals[i];
      sums2[i] += qtr2_vals[i];
    }
    delay(100);
  }

  // compute averages
  for (int i = 0; i < 8; i++) {
    qtr1_biases[i] = sums1[i]/num_reads;
    ys[i] = sums1[i]/num_reads;
    qtr2_biases[i] = sums2[i]/num_reads;
  }

  // print biases
  Serial2.println("qtr1 biases");
  for (int i = 0; i < 8; i++) {
    Serial2.print(qtr1_biases[i]);
    Serial2.print('\t');
  }
  Serial2.println();
//  Serial2.println("qtr2 biases");
//  for (int i = 0; i < 8; i++) {
//    Serial2.print(qtr2_biases[i]);
//    Serial2.print('\t');
//  }
}

void stop_motors() {
  mshield.setM1Speed(0);
  mshield.setM2Speed(0);
  digitalWrite(pin_motor_w1, LOW);
  digitalWrite(pin_motor_w2, LOW);
}

void follow_wall(float Kp, float Ki, float Kd) {
  int error_p, error_i, error_d, error;
  unsigned long current_time = millis();  // # of milliseconds since program start
  int delta_time = current_time - last_time;
  int optimal_dist = 50;   // 50 mm
  
  if (going_forward) {
    error_p = sharpL.getDist() - optimal_dist;
    error_i = total_error * delta_time;
    error_d = (error_p - last_error)/delta_time;
    error = Kp*error_p + Ki*error_i + Kd*error_d;
    total_error += error;
    
    m1_speed = base_speed + error;
    m2_speed = base_speed - error;
  }
  else {
    error_p = sharpR.getDist() - optimal_dist;
    error_i = total_error * delta_time;
    error_d = (error_p - last_error)/delta_time;
    error = Kp*error_p + Ki*error_i + Kd*error_d;
    total_error += error;
    
    m1_speed = -(base_speed - error);
    m2_speed = -(base_speed + error);
  }
  Serial2.println(error);
  delay(10);
  mshield.setM1Speed(m1_speed);
  mshield.setM2Speed(m2_speed);
}

void follow_line(float Kp, float Ki, float Kd) {
  int error_p, error_i, error_d, error;
  unsigned long current_time = millis();  // # of milliseconds since program start
  int delta_time = current_time - last_time;
  if (going_forward) {
    error_p = line_error_qtr1();
    error_i = total_error * delta_time;
    error_d = (error_p - last_error)/delta_time;
    error = Kp*error_p + Ki*error_i + Kd*error_d;
    total_error += error;
    
    m1_speed = base_speed - error;
    m2_speed = base_speed + error;
  }
  else {
    error_p = line_error_qtr2();
    error_i = total_error * delta_time;
    error_d = (error_p - last_error)/delta_time;
    error = Kp*error_p + Ki*error_i + Kd*error_d;
    total_error += error;
    
    m1_speed = -(base_speed - error);
    m2_speed = -(base_speed + error);
  }
  
  mshield.setM1Speed(m1_speed);
  mshield.setM2Speed(m2_speed);
}

float line_error_qtr1() {
  qtr1.read(qtr1_vals);
//  filter_qtr();
  float value_range = 6.32;
  float value_center = 3.5;

  float numer = 0;
  float denom = 0;
  float diff = 0;
  for (int i = 0; i < 8; i++) {
//    diff = ys[i] - qtr1_biases[i];
    diff = qtr1_vals[i] - qtr1_biases[i];
    if (diff < 0.01) {
      diff = 0.01;
    }
    numer += diff * (i+1);
    denom += diff;
  }
  float line_local = numer / denom;
  float error = line_local - value_center;
  
  Serial2.print("qtr1:\t");
  Serial2.print(line_local);
  Serial2.print("\t error\t");
  Serial2.println(error);
  delay(50);
  return error;
}

float line_error_qtr2() {
  // read the raw sensor values
  qtr2.read(qtr2_vals);
  float value_range = 0;    // TODO: max value read from sensor i.e. 6.5
  float value_center = 0;   // TODO: half of value_range

  float numer = 0;
  float denom = 0;
  for (int i = 0; i < 8; i++) {
//    Serial2.print(qtr2_vals[i]);
//    Serial2.print('\t');    
    numer += (qtr2_vals[i] - qtr2_biases[i]) * (i+1);
    denom += qtr2_vals[i] - qtr2_biases[i];
  }
//  Serial2.println();
  float line_local = numer / denom;
//  Serial2.print("\tqtr2:\t");
//  Serial2.println(line_local);
  float d_from_center = (line_local - value_center) * (value_range / 7.5);
  float error = d_from_center - 3.5;
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
 
