// Author: Sam Parry u1008557

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

const byte pin_colorL = A4;
const byte pin_color = A5;
const byte pin_colorR = A6;
const int pin_red = 48;
const int pin_green = 50;
const int pin_blue = 52;

const byte pin_hall = A2;

// *Reflectant Sensor Variables*
QTRSensors qtr1;
QTRSensors qtr2;
int16_t qtr1_biases[] = {1205, 968, 968, 917, 1014, 1004, 1110, 1251};
int16_t qtr2_biases[] = {1250, 1076, 1059, 918, 968, 1098, 1011, 1158};
int16_t qtr1_vals[8];
int16_t qtr2_vals[8];

// *Sharp IR Variables*
bool stop_ir = false;
double last_error = 0;
SharpDistSensor sharpL(pin_sharpL, 1);
SharpDistSensor sharpR(pin_sharpR, 1);
SharpDistSensor sharpF(pin_sharpF, 1);

// *Servo variables*
Servo servoW;
Servo servoC;
Servo servoH;

// angle variables define servo range of motion
int servoW_down_angle = 0;
int servoW_up_angle = 0;
int servoC_open_angle = 0;
int servoC_shut_angle = 0;
int servoH_down_angle = 0;
int servoH_up_angle = 0;
 
// *Motor variables*
DualTB9051FTGMotorShieldUnoMega mshield;
double base_speed = 100;
double pid_scaling = base_speed/100;  // linear PID scaling factor
double m1_speed = 250;
double m2_speed = 250;

// *State Variables*
String state = "";
bool going_forward = false;
bool at_magnet = false;
char bounty_color;
String uno_message;

// *Time Variable*
double t;
unsigned long last_time = millis()/1000;

void setup() {

  // *Configure pins*
  pinMode(pin_motor_w1, OUTPUT);
  pinMode(pin_motor_w2, OUTPUT);
  pinMode(pin_red, OUTPUT);
  pinMode(pin_green, OUTPUT);
  pinMode(pin_blue, OUTPUT);
  
  // *Write beginning servo angles*
  servoW.attach(pin_servo_w);
  servoC.attach(pin_servo_c);
  servoH.attach(pin_servo_h);
  servoW.write(servoW_up_angle);
  servoC.write(servoC_open_angle);
  servoH.write(servoH_up_angle);

  // *Initialize sensors*
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

  // *Initialize communication*
  Serial2.begin(9600);
  Serial2.println("Mega Ready");
  //calibrate_qtrs();
}

void loop() {
  // TODO: test that this new serial comm logic works.
  if (Serial2.available()){                       // message sent from Uno
    uno_message = Serial2.readStringUntil('\n');  // read a String sent from Uno
    uno_message = uno_message.substring(0, state.length()-1); // trim trailing white space
    Serial2.read();                               // clear new line character from buffer
    delay(1);

    if (uno_message.length() > 4 && uno_message.substring(0, 4) == "color") {
      bounty_color = uno_message[7];
    }
    else {
      state = uno_message;
      Serial2.print("state set to: ");
      Serial2.println(state);
    }
  }

  // PM 8
  if (state = "pm") {
    approach_hub();
    turn_hub_from_entrance();
    state = "";
  }

  // Hall Effect sensor
  if (state == "hall") {
    read_hall();
    Serial2.print("At magnet: ");
    Serial2.println(at_magnet);
  }

  // Color sensor
  if (state == "c") {
    char color = read_color();
    Serial2.print("Color:\t");
    Serial2.println(color);
    state = "";
  }

  // Sharp IR
  if (state == "irf") {
    going_forward = true;
    follow_wall();
  }
  else if (state == "irb") {
    going_forward = false;
    follow_wall();
  }

  // Line Following
  if (state == "linef") {
    going_forward = true;
    follow_line();
  }
  else if (state == "lineb") {
    going_forward = false;
    follow_line();
  }

  // Kill switch
  if (state == "s") {
    stop_motors();
    state = "";
  }
}

/*********************
 ** State Functions **
 *********************/

// TODO: test
void turn_hub_from_entrance() {
  // drop/rotate wheel arm
  servoW.write(servoW_down_angle);
  digitalWrite(pin_motor_w1, HIGH);
  digitalWrite(pin_motor_w2, LOW);

  // stop/raise arm when magnet sensed
  at_magnet = false;
  while (not at_magnet) {
    read_hall();
  }
  digitalWrite(pin_motor_w1, LOW);
  digitalWrite(pin_motor_w2, LOW);
  servoW.write(servoW_up_angle);

  // enter hub
  going_forward = false;
  at_magnet = false;
  last_time = millis()/1000;
  while (not at_magnet) {
    follow_line();
    read_hall();
  }

  // stop when magnet sensed
  stop_motors();

  // drop/rotate wheel arm
  servoW.write(servoW_down_angle);
  digitalWrite(pin_motor_w1, HIGH);
  digitalWrite(pin_motor_w2, LOW);

  // determine color opposite bounty color
  char target_color;
  switch (bounty_color) {
    case 'r':
      target_color = 'b';
    case 'g':
      target_color = 'k';
    case 'b':
      target_color = 'r';
  }

  // stop rotation when color is found
  char color = 'w';
  while (color != target_color) {
    color = read_color();
  }
  stop_motors();
}

//TODO: test
void approach_hub() {
  // drive past starting line
  last_time = millis()/1000;
  follow_line();
  delay(1000);

  // drive till the black tape is sensed.
  bool at_hub = false;
  while (not at_hub) {
    follow_line();
    at_hub = at_black_tape();
  }
}

/****************************
 ** User-Defined Functions **
 ****************************/

// TODO: Finish writing with working sensor array (tweek threshold & num_reads)
bool at_black_tape() {
  int reds[] = {0, 0, 0};
  int greens[] = {0, 0, 0};
  int blues[] = {0, 0, 0};
  int num_reads = 30;
  int threshold = 0;

  // collect color readings
  for (int i = 0; i < num_reads; i++) {

    // read red
    digitalWrite(pin_red, LOW);
    digitalWrite(pin_green, HIGH);
    digitalWrite(pin_blue, HIGH);
    delay(1);
    reds[0] += analogRead(pin_colorL);
    reds[1] += analogRead(pin_color);
    reds[2] += analogRead(pin_colorR);

    // read green
    digitalWrite(pin_red, HIGH);
    digitalWrite(pin_green, LOW);
    digitalWrite(pin_blue, HIGH);
    delay(1);
    greens[0] += analogRead(pin_colorL);
    greens[1] += analogRead(pin_color);
    greens[2] += analogRead(pin_colorR);

    // read blue
    digitalWrite(pin_red, HIGH);
    digitalWrite(pin_green, HIGH);
    digitalWrite(pin_blue, LOW);
    delay(1);
    blues[0] += analogRead(pin_colorL);
    blues[1] += analogRead(pin_color);
    blues[2] += analogRead(pin_colorR);

    // reset
    digitalWrite(pin_red, HIGH);
    digitalWrite(pin_green, HIGH);
    digitalWrite(pin_blue, HIGH);    
  }

  // average the readings
  for (int i = 0; i < 3; i++) {
    reds[i] = reds[i]/num_reads;
    greens[i] = greens[i]/num_reads;
    blues[i] = blues[i]/num_reads;

    // determine if the sensors are readings black
    if (reds[i] < threshold || greens[i] < threshold || blues[i] < threshold) {
      return false;
    }
  }
  return true;
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
    delay(10);
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

void follow_line() {
  double Kp = 40, Kd = 20;
  double error_p, error_d, error;
  unsigned long current_time = millis()/1000;  // # of seconds since program start
  int delta_time = current_time - last_time;

  // apply pid error scaling factor
  Kp *= pid_scaling;
  Kd *= pid_scaling;

  // read errors
  error_p = line_error();
  error_d = (error_p - last_error)/delta_time;
  error = Kp*error_p + Kd*error_d;

  Serial2.print("error_p:\t");
  Serial2.print(error_p);
  Serial2.print("\terror_d\t");
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

// TODO: more calibration
void follow_wall() {
  double Kd = 0.9, Kp = 0.9;
  int error_p, error_d, error;
  unsigned long current_time = millis()/1000;
  int delta_time = current_time - last_time;
  int optimal_dist = 50;   // 50 mm
  
  // apply pid error scaling factor
  Kp *= pid_scaling;
  Kd *= pid_scaling;

  if (going_forward) {
    error_p = sharpL.getDist() - optimal_dist;
    error_d = (error_p - last_error)/delta_time;
    error = Kp*error_p + Kd*error_d;
    
    m1_speed = base_speed + error;
    m2_speed = base_speed - error;
  }
  else {
    error_p = sharpR.getDist() - optimal_dist;
    error_d = (error_p - last_error)/delta_time;
    error = Kp*error_p + Kd*error_d;
    
    m1_speed = -(base_speed - error);
    m2_speed = -(base_speed + error);
  }
  
  Serial2.print("error_p:\t");
  Serial2.print(error_p);
  Serial2.print("\terror_d\t");
  Serial2.print(error_d);
  Serial2.print("\terror:\t");
  Serial2.print(error);
  Serial2.print("\tdt:\t");
  Serial2.println(delta_time);

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

// TODO: tweak threshold and num_reads
char read_color() {
  char color;
  int num_reads = 30;
  int threshold = 0;
  int red=0, green=0, blue=0;

  // collect color readings
  for (int i = 0; i < num_reads; i++) {

    // read red
    digitalWrite(pin_red, LOW);
    digitalWrite(pin_green, HIGH);
    digitalWrite(pin_blue, HIGH);
    delay(1);
    red += analogRead(pin_color);

    // read green
    digitalWrite(pin_red, HIGH);
    digitalWrite(pin_green, LOW);
    digitalWrite(pin_blue, HIGH);
    delay(1);
    green += analogRead(pin_color);

    // read blue
    digitalWrite(pin_red, HIGH);
    digitalWrite(pin_green, HIGH);
    digitalWrite(pin_blue, LOW);
    delay(1);
    blue += analogRead(pin_color);

    // reset
    digitalWrite(pin_red, HIGH);
    digitalWrite(pin_green, HIGH);
    digitalWrite(pin_blue, HIGH);
  }

  // average the readings
  red = red/num_reads;
  green = green/num_reads;
  blue = blue/num_reads;

  // determine color
  if (red >= threshold && red >= green && red >= (blue-10)) {
    color = 'r';
  }
  else if (green >= threshold && green >= red && green >= (blue-10)) {
    color = 'g';
  }
  else if (blue >= threshold && blue >= red && blue >= (green-10)) {
    color = 'b';
  }
  else {
    color = 'w';
  }
  return color;
}

// TODO: Test with hall effect sensor
void read_hall() {
  int threshold = 1000;
  float reading = analogRead(pin_hall);
  if (reading >= threshold) {
    at_magnet = true;
  }
  else {
    at_magnet = false;
  }
}

void stop_motors() {
  mshield.setM1Speed(0);
  mshield.setM2Speed(0);
  digitalWrite(pin_motor_w1, LOW);
  digitalWrite(pin_motor_w2, LOW);
}

/******************
 ** Delete These **
 ******************/

void test_instrument_motor() {
  instrument_motor_d1();
  delay(2000);
  instrument_motor_d2();
  delay(2000);
  digitalWrite(pin_motor_w1, LOW);
  digitalWrite(pin_motor_w2, LOW);
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
 
void instrument_motor_d1() {
  digitalWrite(pin_motor_w1, HIGH);
  digitalWrite(pin_motor_w2, LOW);
}

void instrument_motor_d2() {
  digitalWrite(pin_motor_w1, LOW);
  digitalWrite(pin_motor_w2, HIGH);
}
