// Author: Sam Parry u1008557

/****************************
 ** #defines and #includes **
 ****************************/
#include <Servo.h>;
#include <SharpDistSensor.h>;
#include <QTRSensors.h>;
#include "DualTB9051FTGMotorShieldUnoMega.h";
#include "Encoder.h";

/***********************
 ** Global Variables ***
 ***********************/
// *Declare pins*
const int pin_motor_w1 = 5;
const int pin_motor_w2 = 7;

const int pin_servo_w = 46;
const int pin_servo_c = 45;
const int pin_servo_h = 44;

const uint8_t pins_qtr1[] = {22, 24, 26, 28, 30, 32, 34, 36};
const uint8_t pins_qtr2[] = {23, 25, 27, 29, 31, 33, 35, 37};

const byte pin_sharpL = A10;
const byte pin_sharpR = A11;
const byte pin_sharpF = A12;

const byte pin_color = A5;
const int pin_red = 48;
const int pin_green = 50;
const int pin_blue = 52;
const byte pin_hall = A2;

const int pin_encoder_m1a = 18;
const int pin_encoder_m1b = 19;
const int pin_encoder_m2a = 20;
const int pin_encoder_m2b = 21;

// *Reflectant Sensor Variables*
QTRSensors qtr1;
QTRSensors qtr2;
int16_t qtr1_biases[] = {654, 552, 516, 488, 504, 504, 552, 603};
int16_t qtr2_biases[] = {600, 520, 549, 322, 506, 582, 554, 653};
int16_t qtr1_vals[8];
int16_t qtr2_vals[8];

// *Sharp IR Variables*
double last_error = 0;
SharpDistSensor sharpL(pin_sharpL, 1);
SharpDistSensor sharpR(pin_sharpR, 1);
SharpDistSensor sharpF(pin_sharpF, 1);

// *Servo variables*
Servo servoW;
Servo servoC;
Servo servoH;

// angle variables define servo range of motion
int servoW_down_angle = 25;
int servoW_up_angle = 0;
int servoC_open_angle = 0;
int servoC_shut_angle = 0;
int servoH_down_angle = 0;
int servoH_up_angle = 0;

// *Motor variables*
DualTB9051FTGMotorShieldUnoMega mshield;
double base_speed = 100;
double pid_scaling = base_speed/100.0;  // linear PID scaling factor
double m1_speed = base_speed;
double m2_speed = base_speed;

// *Encoder variables*
Encoder EncoderM1(pin_encoder_m1a , pin_encoder_m1b);
Encoder EncoderM2(pin_encoder_m2a,  pin_encoder_m2b);
double theta_traveled_m1_old = 0;
double theta_traveled_m2_old = 0;
double radians_traveled = 0;

// *State Variables*
String state = "";
bool going_forward = false;
bool at_magnet = false;
char bounty_color;
char opposite_color;
String uno_message;

// *Time Variable*
double t;
float last_time;
float current_time;
float delta_time;

void setup() {

  // *Configure pins*
  pinMode(pin_motor_w1, OUTPUT);
  pinMode(pin_motor_w2, OUTPUT);
  pinMode(pin_red, OUTPUT);
  pinMode(pin_green, OUTPUT);
  pinMode(pin_blue, OUTPUT);
  pinMode(pin_hall, INPUT);

  color_off();
  
  // *Write beginning servo angles*
  // servoW.attach(pin_servo_w);
  // servoC.attach(pin_servo_c);
  // servoH.attach(pin_servo_h);
  // servoW.write(servoW_up_angle);
  // servoC.write(servoC_open_angle);
  // servoH.write(servoH_up_angle);

  // *Initialize sensors*
  qtr1.setTypeRC();
  qtr2.setTypeRC();
  qtr1.setSensorPins(pins_qtr1, 8);
  qtr2.setSensorPins(pins_qtr2, 8);
  sharpL.setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);
  sharpR.setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);
  // sharpF.setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);

  // *Initialize motor driver*
  mshield.init();
  mshield.enableDrivers();
  mshield.flipM2(true);

  // *Initialize communication*
  Serial2.begin(9600);
  Serial2.println("Mega Ready");

  // *Final commands*
  last_time = millis();
  // calibrate_qtrs();
}

void loop() {

  if (Serial2.available()){
    uno_message = Serial2.readStringUntil('\n');  // read a String sent from Uno
    uno_message = uno_message.substring(0, uno_message.length()-1); // trim trailing white space
    Serial2.read();                               // clear new line character from buffer
    delay(10);

    // recieve bounty color from uno
    if (uno_message.length() > 4 && uno_message.substring(0,5) == "color") {
      bounty_color = uno_message[7];
      identify_opposite_color();
      Serial2.print("Bounty color:\t");
      Serial2.print(bounty_color);
      Serial2.print("\tOpposite color:\t");
      Serial2.println(opposite_color);
    }
    else {  // recieve state commands from uno
      state = uno_message;
      Serial2.print("state set to: ");
      Serial2.println(state);
    }
  }

  //Encoder straight line
  if (state == "line") {

    //Robot Measurements
    double wheel_radius = 35; //mm
    double desired_total_distance = 100;
    double required_radians = desired_total_distance / wheel_radius;
    
    if (radians_traveled <= required_radians) {
      encoder_drive_straight(desired_total_distance);
    }
    else if (radians_traveled >= required_radians) {
      reset_encoder_tracking();
    }
  }
  //Encoder circle
  else if (state == "circle") {

    //Robot Measurements
    double wheel_radius = 35; //mm
    double desired_radius = 400;
    double desired_theta = 2 * PI;
    double required_radians = (desired_radius * desired_theta) / wheel_radius;

    if (radians_traveled <= required_radians) {
      encoder_drive_circle(desired_radius, desired_theta);
    }
    else if (radians_traveled >= required_radians) {
      reset_encoder_tracking();
    }
  }


  // Hall Effect sensor
  if (state == "hall") {
    t = millis()/1000;
    while (millis()/1000 - t <= 5) {
      read_hall();
      delay(100);
      Serial2.print("At magnet:\t");
      Serial2.println(at_magnet);
    }
    state = "";
  }

  // Color sensor
  if (state == "c") {
    t = millis()/1000;
    while (millis()/1000 - t <= 6) {    
      char color = read_color();
      Serial2.print("Color:\t");
      Serial2.println(color);
      delay(1000);
    }
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

  // Kill motors
  if (state == "s") {
    stop_motors();
    state = "";
  }
}

/*********************
 ** State Functions **
 *********************/

void turn_hub_from_entrance() {

  // drop/rotate wheel arm
  servoW.write(servoW_down_angle);
  instrument_motor_cw();

  // stop/raise arm motor when magnet sensed
  at_magnet = false;
  while (not at_magnet) {
    read_hall();
  }
  stop_motors();
  servoW.write(servoW_up_angle);
}

void turn_hub_from_hub() {

  // drop/rotate wheel arm
  servoW.write(servoW_down_angle);
  instrument_motor_cw();

  // stop rotation when color is found
  char color = 'w';
  while (color != opposite_color) {
    color = read_color();
    Serial2.println(color);
  }
  servoW.write(servoW_up_angle);
  stop_motors();
}

/****************************
 ** User-Defined Functions **
 ****************************/

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

void color_off() {
  digitalWrite(pin_red, HIGH);
  digitalWrite(pin_green, HIGH);
  digitalWrite(pin_blue, HIGH);
}

void encoder_drive_straight(double desired_distance) {
  //encoder variables
  double wheel_radius = 35; //mm
  double GearRatio = 100;
  double countsPerRev = 64;
  double RevsShaft2Rad = 2 * PI;
  double required_radians = desired_distance / wheel_radius;
  double Kp = 1.5;

  //Apply PID scaling
  Kp *= pid_scaling;
  
  //Time based variables
  current_time = millis();
  delta_time = current_time / 1000.0 - last_time;

  //read the encoders
  double count_m1 = EncoderM1.read();
  double count_m2 = -EncoderM2.read();

  //solve for radians traveled
  double theta_traveled_m1 = (count_m1 * RevsShaft2Rad) * (1 / (countsPerRev * GearRatio));
  double theta_traveled_m2 = (count_m2 *  RevsShaft2Rad) * (1 / (countsPerRev * GearRatio));

  //solve for velocities
  double velocity_m1 = (theta_traveled_m1 - theta_traveled_m1_old)/delta_time;
  double velocity_m2 = (theta_traveled_m2 - theta_traveled_m2_old)/delta_time;
  double velocity_average = (velocity_m1 + velocity_m2)/2;

  //Solve for the next desired radian count
  double theta_d1 = theta_traveled_m1 + velocity_average * (delta_time);
  double theta_d2 = theta_traveled_m2 + velocity_average * (delta_time);

  //Solve for voltages
  m1_speed = base_speed + Kp * (theta_d1 - theta_traveled_m1);
  m2_speed = base_speed + Kp * (theta_d2 - theta_traveled_m2);
  
  //Set Motor voltages
  mshield.setM1Speed(m1_speed);
  mshield.setM2Speed(m2_speed);
  
  //record global variables
  radians_traveled = (theta_traveled_m1 + theta_traveled_m2) / 2;
  theta_traveled_m1_old = theta_traveled_m1;
  theta_traveled_m2_old = theta_traveled_m2;
  last_time = current_time / 1000.0;
}

void encoder_drive_circle(double desired_radius, double desired_theta) {
  //Robot Measurements
  double wheel_radius = 35; //mm
  double distance_between_wheels = 240; //mm

  //encoder variables
  double GearRatio = 100;
  double countsPerRev = 64;
  double RevsShaft2Rad = 2 * PI;
  double required_radians = RevsShaft2Rad * ((desired_radius * desired_theta)/ (RevsShaft2Rad*wheel_radius));

  //Endoder variables circle
  double velocity_ratio = (desired_radius + distance_between_wheels)/desired_radius;
  double Kp = 50;
  
  //Apply PID scaling
  Kp *= pid_scaling;
  
  //Time based variables
  current_time = millis() /1000.0;
  delta_time = current_time - last_time;
 
  //read the encoders
  double count_m1 = EncoderM1.read();
  double count_m2 = -EncoderM2.read();

  //solve for radians traveled
  double theta_traveled_m1 = (count_m1 * RevsShaft2Rad) * (1 / (countsPerRev * GearRatio));
  double theta_traveled_m2 = (count_m2 *  RevsShaft2Rad) * (1 / (countsPerRev * GearRatio));

  //solve for current velocities
  double velocity_m1 = (theta_traveled_m1 - theta_traveled_m1_old)/delta_time;
  double velocity_m2 = (theta_traveled_m2 - theta_traveled_m2_old)/delta_time;
  double velocity_average = (velocity_m1 + velocity_m2)/2;

  //Solve for the next desired radian count
  double theta_d1 = theta_traveled_m1 + velocity_average * delta_time;
  double theta_d2 = theta_traveled_m2 + velocity_average * delta_time;

  //Solve for voltages
  m1_speed = (base_speed + Kp * (theta_d1 - theta_traveled_m1));
  m2_speed = velocity_ratio * (base_speed + Kp * (theta_d2 - theta_traveled_m2));

  //Set voltages to motors
  mshield.setM1Speed(m1_speed);
  mshield.setM2Speed(m2_speed);

  //record global variables
  radians_traveled = (theta_traveled_m1 + theta_traveled_m2) / 2;
  theta_traveled_m1_old = theta_traveled_m1;
  theta_traveled_m2_old = theta_traveled_m2;
  last_time = current_time;
}

void follow_line() {
  double Kp = 20, Kd = 20;
  float error_p, error_d, error;
  current_time = millis();
  delta_time = current_time - last_time;

  // read errors
  error_p = line_error();
  error_d = (error_p - last_error)/delta_time;
  error = Kp*error_p + Kd*error_d;
  error *= pid_scaling; // apply pid error scaling

  // Serial2.print("error_p:\t");
  // Serial2.print(error_p);
  // Serial2.print("\terror_d\t");
  // Serial2.print(error_d);
  // Serial2.print("\terror:\t");
  // Serial2.print(error);
  // Serial2.print("\tdt:\t");
  // Serial2.println(delta_time);

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

void follow_wall() {
  double Kp = 1, Kd = 1;
  float error_p, error_d, error;
  current_time = millis();
  delta_time = current_time - last_time;
  int optimal_dist = 60;   // mm
  int dist;

  // handles large delta time from first function call
  if (delta_time >= 100) {
    delta_time = 52;
  }

  // The distance readings are all way off for some reason.
  // negative error (turn right) - too far
  // positive error (turn left) - too close
  // turning right (m1 > m2)
  // turning left (m2 > m1)

  if (going_forward) {
    dist = sharpR.getDist();
    error_p = dist - optimal_dist;
    error_d = (error_p - last_error)/delta_time;
    error = Kp*error_p + Kd*error_d;
    error *= pid_scaling;
    
    m1_speed = base_speed - error;
    m2_speed = base_speed + error;
  }
  else {
    dist = sharpL.getDist();
    error_p = dist - optimal_dist;
    error_d = (error_p - last_error)/delta_time;
    error = Kp*error_p + Kd*error_d;
    error *= pid_scaling;
    
    m1_speed = -(base_speed + error);
    m2_speed = -(base_speed - error);
  }
  
  Serial2.print("dist: ");
  Serial2.print(dist);
  Serial2.print("\terror_p: ");
  Serial2.print(error_p);
  Serial2.print("\terror_d ");
  Serial2.print(error_d);
  Serial2.print("\terror: ");
  Serial2.println(error);

  mshield.setM1Speed(m1_speed);
  mshield.setM2Speed(m2_speed);
  last_time = current_time;
}

void identify_opposite_color() {
  // determine color opposite bounty color on hub
  if (bounty_color == 'r') {
    opposite_color = 'b';
  }
  // No hub rotation happens for the green bounty
  else if (bounty_color == 'g') {
    opposite_color = 'g';
  }
  else if (bounty_color == 'b') {
    opposite_color = 'r';
  }
}

void instrument_motor_ccw() {
  digitalWrite(pin_motor_w1, HIGH);
  digitalWrite(pin_motor_w2, LOW);
}

void instrument_motor_cw() {
  digitalWrite(pin_motor_w1, LOW);
  digitalWrite(pin_motor_w2, HIGH);
}

float line_error() {

  // read front or rear values
  float value_center;
  if (going_forward) {
    qtr1.read(qtr1_vals);
    value_center = 4.48;
  }
  else {
    qtr2.read(qtr2_vals);
    value_center = 4.43;
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
  float error = value_center - line_local;
  return error;
}

// TODO: redo with new sensor
char read_color() {
  char color;
  int num_reads = 5;
  int delay_time = 1;
  int threshold = 900;
  int red=0, green=0, blue=0;

  // collect color readings
  for (int i = 0; i < num_reads; i++) {

    // read red
    digitalWrite(pin_red, LOW);
    digitalWrite(pin_green, HIGH);
    digitalWrite(pin_blue, HIGH);
    delay(delay_time);
    red += analogRead(pin_color);

    // read green
    digitalWrite(pin_red, HIGH);
    digitalWrite(pin_green, LOW);
    digitalWrite(pin_blue, HIGH);
    delay(delay_time);
    green += analogRead(pin_color);

    // read blue
    digitalWrite(pin_red, HIGH);
    digitalWrite(pin_green, HIGH);
    digitalWrite(pin_blue, LOW);
    delay(delay_time);
    blue += analogRead(pin_color);

    color_off();
  }

  // average the readings
  red = red/num_reads;
  green = green/num_reads;
  blue = blue/num_reads;

  Serial2.print(red);
  Serial2.print("\t");
  Serial2.print(green);
  Serial2.print("\t");
  Serial2.println(blue);

  // determine color
  if (red >= threshold && green >= threshold) {
    color = 'w';
  }
  else {
    if (red >= green && red >= blue) {
      if (red >= 400) {
        color = 'r';
      }
    }
    else if (green >= red && green >= blue) {
      if (green >= 300) {
        color = 'g';
      }
    }
    else if (blue >= red && blue >= green) {
      if (blue >= 400) {
        color = 'b';
      }
    }
  }
  return color;
}

// TODO: redo threshold when remounted
void read_hall() {
  int threshold = 500;
  int reading = analogRead(pin_hall);
  if (reading <= threshold) {
    at_magnet = true;
  }
  else {
    at_magnet = false;
  }
}

void reset_encoder_tracking(){
      stop_motors();
      radians_traveled = 0;
      theta_traveled_m1_old = 0;
      theta_traveled_m2_old = 0;
      EncoderM1.write(0);
      EncoderM2.write(0);
      Serial2.print("Destination Reached");
}

void stop_motors() {
  mshield.setM1Speed(0);
  mshield.setM2Speed(0);
  digitalWrite(pin_motor_w1, LOW);
  digitalWrite(pin_motor_w2, LOW);
}

/************************
 ** Delete These Later **
 ************************/

void test_instrument_motor() {
  instrument_motor_ccw();
  delay(2000);
  instrument_motor_cw();
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
