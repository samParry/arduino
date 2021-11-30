// Author: Sam Parry u1008557

/****************************
 ** #defines and #includes **
 ****************************/
#define DEBUG 1

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#elif DEBUG == 2
#define debug(x) Serial2.print(x)
#define debugln(x) Serial2.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

#include <Servo.h>;
#include <SharpDistSensor.h>;
#include <QTRSensors.h>;
#include "DualTB9051FTGMotorShieldUnoMega.h";
#include "Encoder.h";

/**********************
 ** Global Variables **
 **********************/
// *Declare pins*
const int pin_motor_w1 = 5;
const int pin_motor_w2 = 7;

const int pin_servo_w = 46;
const int pin_servo_c = 45;
const int pin_servo_h = 44;

const uint8_t pins_qtr1[] = {22, 24, 26, 28, 30, 32, 34, 36};
const uint8_t pins_qtr2[] = {23, 25, 27, 29, 31, 33, 35, 37};

const byte pin_sharpC = A9;
const byte pin_sharpL = A10;
const byte pin_sharpR = A11;
const byte pin_sharpF = A12;

const byte pin_color = A5;
const int pin_red = 48;
const int pin_green = 50;
const int pin_blue = 52;
const byte pin_hall = A2;

const byte pin_freq_filt = A6;
const byte pin_freq_unfilt = A7;

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
SharpDistSensor sharpC(pin_sharpC, 1);
SharpDistSensor sharpL(pin_sharpL, 1);
SharpDistSensor sharpR(pin_sharpR, 1);
SharpDistSensor sharpF(pin_sharpF, 1);

// *Servo variables*
Servo servoW;
Servo servoC;
Servo servoH;

// angle variables define servo range of motion
int servoW_down_angle = 130;
int servoW_up_angle = 135;
int servoC_open_angle = 0;
int servoC_shut_angle = 0;
int servoH_down_angle = 180;
int servoH_mid_angle = 150;
int servoH_up_angle = 120;

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
bool test_state = true;
bool going_forward = false;
bool at_magnet = false;
char bounty_color;
char opposite_color;
String uno_message;

// *Time Variable*
float t;
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
  pinMode(pin_freq_unfilt, INPUT);
  pinMode(pin_freq_filt, INPUT);
  color_off();

  // *Write beginning servo angles*
  // servoW.attach(pin_servo_w);
  // servoC.attach(pin_servo_c);
  // servoH.attach(pin_servo_h);
  // servoW.write(servoW_up_angle);
  // servoC.write(servoC_open_angle);
  // servoH.write(servoH_down_angle);

  // *Initialize sensors*
  // qtr1.setTypeRC();
  // qtr2.setTypeRC();
  // qtr1.setSensorPins(pins_qtr1, 8);
  // qtr2.setSensorPins(pins_qtr2, 8);
  // sharpC.setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);  
  // sharpL.setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);
  // sharpR.setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);
  // sharpF.setModel(SharpDistSensor::GP2Y0A41SK0F_5V_DS);

  // *Initialize motor driver*
  mshield.init();
  mshield.enableDrivers();
  mshield.flipM2(true);

  // *Initialize communication*
  Serial.begin(9600);
  Serial2.begin(9600);
  debugln("Mega Ready");

  // *Final commands*
  // calibrate_qtrs();
  last_time = millis();
}

void loop() {

  read_uno();
  read_mega();

  if (test_state) {
    test_mode();
  }
}

/**********************
 ** Testing Function **
 **********************/

void test_mode() {
  debugln("\n           Test Mode");
  debugln("------------------------------------------");
  debugln("Command    Description    m=menu");
  debugln("------------------------------------------");
  debugln("c          Color sensor");
  debugln("co         Claw open");
  debugln("cs         Claw shut");
  debugln("ec         Drive circle (encoder)");
  debugln("el         Straight line (encoder)");
  debugln("f          Compare frequencies");
  debugln("h          Hall effect Sensor");
  debugln("hd         Hook arm down");
  debugln("hu         Hook arm up");
  debugln("hw         Turn hub wheel");
  debugln("irf        Rangefinder (front-facing)");
  debugln("irl        Rangefinder (left-facing)");
  debugln("irr        Rangefinder (right-facing)");
  debugln("lb         Line following (backward)");
  debugln("lf         Line following (forward)");
  debugln("qtr        Calibrate QTR sensors");
  debugln("qtrbwf      Scan for black/white with QTRs (forward)");
  debugln("qtrbwb      Scan for black/white with QTRs (backward)");
  debugln("tcw        Turn clockwise");
  debugln("tccw       Turn counter-clockwise");
  debugln("wb         Follow wall (backward)");
  debugln("wf         Follow wall (forward)");
  debugln("wu         Raise hub wheel");
  debugln("wd         Lower hub wheel");

  while (test_state) {
    // Read comms at top of loop
    read_uno();
    read_mega();

    // Color sensor
    if (state == "c") {
      char color = read_color();
      delay(1000);
    }

    // Claw
    else if (state == "co") {
      claw_open();
      state = "";
    }
    else if (state == "cs") {
      claw_shut();
      state = "";
    }

    // Encoders
    else if (state == "ec") {

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
    else if (state == "el") {

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

    // Compare frequencies
    else if (state == "f") {
      compare_frequency();
      delay(500);
    }

    // Hall Effect sensor
    else if (state == "h") {
      read_hall();
    }

    // Hook arm
    else if (state == "hd") {
      servoH.write(servoH_down_angle);
      state = "";
    }
    else if (state == "hu") {
      servoH.write(servoW_up_angle);
      state = "";
    }

    // Turn hub wheel
    else if (state == "hw") {
      instrument_motor_cw();
      debugln("Turning CW");
      delay(2000);
      instrument_motor_ccw();
      debugln("Turning CCW");
      delay(2000);
      stop_motors();
      state = "";
    }

    // Measure distances (mm)
    else if (state == "irf") {
      debugln(sharpF.getDist());
      delay(1000);
    }
    else if (state == "irl") {
      debugln(sharpL.getDist());
      delay(1000);
    }
    else if (state == "irr") {
      debugln(sharpR.getDist());
      delay(1000);
    }

    // Line following
    else if (state == "lb") {
      going_forward = false;
      follow_line();
    }
    else if (state == "lf") {
      going_forward = true;
      follow_line();
    }
    else if (state == "qtr") {
      calibrate_qtrs();
      state = "";
    }
    else if (state == "qtrbwf") {
      going_forward = true;
      char color = qtr_black_or_white();
      
      // print readings
      debugln("qtr1 values");
      for (int i = 0; i < 8; i++) {
        debug(qtr1_vals[i]);
        debug('\t');
      }
      debugln(color);
    }
    else if (state == "qtrbwb") {
      going_forward = false;
      char color = qtr_black_or_white();
      
      // print readings
      debugln("qtr2 values");
      for (int i = 0; i < 8; i++) {
        debug(qtr2_vals[i]);
        debug('\t');
      }
      debugln(color);      
    }

    // Turn 90 degrees
    else if (state == "tcw") {
      turn_cw();
      state = "";
    }
    else if (state == "tccw") {
      turn_ccw();
      state = "";
    }

    // Wall Following
    else if (state == "wb") {
      going_forward = false;
      follow_wall();
    }
    else if (state == "wf") {
      going_forward = true;
      follow_wall();
    }

    // Move hub wheel
    else if (state == "wd") {
      servoW.write(servoW_down_angle);
    }
    else if (state == "wu") {
      servoW.write(servoW_up_angle);
    }

    // Stop command
    else if (state == "s") {
      stop_motors();
      state = "";
    }

    // Re-print menu
    else if (state == "m") {
      state = "";
      break;
    }
  }
}

/*******************************
 ** State Functions: Entrance **
 *******************************/

// *Start*
// TODO
void approach_hub() {
}

// TODO
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

// TODO
void enter_hub() {
}

// TODO
void turn_hub_from_hub() {

  // drop/rotate wheel arm
  servoW.write(servoW_down_angle);
  instrument_motor_cw();

  // stop rotation when color is found
  char color = 'w';
  while (color != opposite_color) {
    color = read_color();
    debugln(color);
  }
  servoW.write(servoW_up_angle);
  stop_motors();
}

/*****************************
 ** State Functions: Canyon **
 *****************************/

// TODO: This might be callable once hub is rotated into place
// The canyon doesn't require rotating the hub once on it
void traverse_canyon() {
  // follow line
  // look for signal
  // stop
  stop_motors();
}

// TODO
String face_bounty() {
  // detect frequency
  String turn_dir = compare_frequency();

  // turn to face bounty
  if (turn_dir == "cw") {
    turn_cw();
  }
  else if (turn_dir == "ccw") {
    turn_ccw();
  }

  // grab bounty
  approach_block();
  return turn_dir;
}

// TODO
void back_to_center(String turn_dir) {
  // follow line
  // look for signal
  // stop
  stop_motors();

  // turn opposite direction of initial turn
  if (turn_dir == "cw") {
    turn_ccw();
  }
  else if (turn_dir == "ccw") {
    turn_cw();
  }
}

// TODO
void go_home() {
  // The robot should have a line all the way to the start
  follow_line();
}

/***************************
 ** State Functions: Cave **
 ***************************/

// TODO
void traverse_cave() {
  // move forward a bit to exit hub (TEST THIS)
  going_forward = true;
  t = millis();
  while (millis() - t <= 1500) {
    mshield.setM1Speed(m1_speed);
    mshield.setM2Speed(m2_speed);
  }

  // TODO: It needs to stop wall following when wall is gone
  follow_wall();

  approach_block();
}

// TODO
void traverse_cave_backward() {
  
  // TODO: follow line till line is gone/wall appears
  follow_line();

  // TODO: follow wall till wall is gone/line appears
  follow_line();
}


/*******************************
 ** State Functions: Compound **
 *******************************/

// TODO
void approach_gate() {
  // drop arm and drive to gate
  servoH.write(servoH_down_angle);
  bool at_gate = false;
  while (at_gate == false) {
    follow_line();
    at_gate = sense_gate();
  }
  stop_motors();
}

// TODO
void raise_gate() {
  // raise gate
  servoH.write(servoH_up_angle);
  delay(1000);
  servoH.write(servoH_mid_angle);

  // If gate isn't up, try again
  bool at_gate = sense_gate();
  if (at_gate) {

    // reverse and approach gate again
    going_forward = false;
    t = millis();
    while (millis() - t <= 1500) {
      follow_line();
    }
    going_forward = true;
    approach_gate();
    servoH.write(servoH_up_angle);
    delay(1000);
    servoH.write(servoH_mid_angle);
  }
}

/*****************************
 ** State Functions: Return **
 *****************************/


/****************************
 ** User-Defined Functions **
 ****************************/

// TODO: Make this work with an IR sensor measuring block distance
void approach_block() {
  going_forward = true;
  int dist = sharpC.getDist();
  int threshold = 20;
  while (dist > threshold) {
    follow_line();
    dist = sharpC.getDist();
  }
  stop_motors();
  claw_shut();
  going_forward = false;
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
    delay(5);
  }

  // compute averages
  for (int i = 0; i < 8; i++) {
    qtr1_biases[i] = sums1[i]/num_reads;
    qtr2_biases[i] = sums2[i]/num_reads;
  }

  // print biases
  debugln("qtr1 biases");
  for (int i = 0; i < 8; i++) {
    debug(qtr1_biases[i]);
    debug('\t');
  }
  debugln();
  
  debugln("qtr2 biases");
  for (int i = 0; i < 8; i++) {
    debug(qtr2_biases[i]);
    debug('\t');
  }
  debugln();
}

void claw_open() {
  servoC.write(servoC_open_angle);
}

void claw_shut() {
  servoC.write(servoC_shut_angle);
}

void color_off() {
  digitalWrite(pin_red, HIGH);
  digitalWrite(pin_green, HIGH);
  digitalWrite(pin_blue, HIGH);
}

// TODO
String compare_frequency() {
  // HIGH = Turn left
  // LOW = Turn right
  int filtered = 0;
  int unfiltered = 0;

  int num_reads = 5;
  int delay_time = 1;

  // average multiple readings
  for (int i = 0; i < num_reads; i++) {
    filtered += analogRead(pin_freq_filt);
    unfiltered += analogRead(pin_freq_unfilt);
    delay(delay_time);
  }

  filtered = filtered/num_reads;
  unfiltered = unfiltered/num_reads;
  debug("filtered: "); debug(filtered);
  debug("\tunfiltered: "); debugln(unfiltered);

  // determine if signal is high or low

  // return direction to turn
  return "cw";
}

// TODO: rotate hub a little more to account for offset sensors
void correct_hub() {
  instrument_motor_cw();
  delay(100);   // experiment with timing
  stop_motors();
}

// TODO: Figure out proper thresholds in practice
char determine_color(int r, int g, int b) {

  // determine color
  char color;
  int w_threshold = 700;
  int k_threshold = 250;
  if (r >= w_threshold && g >= w_threshold && b >= w_threshold) {
    color = 'w';
  }
  else if (r <= k_threshold && g <= k_threshold && b <= k_threshold) {
    color = 'k';
  }
  else if (r >= g && r >= b) {
    color = 'r';
  }
  else if (g >= r && g >= b) {
    color = 'g';
  }
  else if (b >= r && b >= g) {
    color = 'b';
  }
  debug(r);
  debug("\t");
  debug(g);
  debug("\t");
  debug(b);
  debug("\tColor:\t");
  debugln(color);
  return color;
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

// TODO: Make it stop if it looses the line
// TODO: Get it to recognize white or black
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

  debug("error_p:\t");
  debug(error_p);
  debug("\terror_d\t");
  debug(error_d);
  debug("\terror:\t");
  debug(error);
  debug("\tdt:\t");
  debugln(delta_time);

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
  
  debug("dist: ");
  debug(dist);
  debug("\terror_p: ");
  debug(error_p);
  debug("\terror_d ");
  debug(error_d);
  debug("\terror: ");
  debugln(error);

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

// TODO: test
char qtr_black_or_white() {
  // returns 'w' for white
  // returns 'b' for black
  // returns 'n' for no
  int white_threshold = 2000;
  int black_threshold = 200;
  bool black = true;
  bool white = true;

  // read qtr
  if (going_forward) {
    qtr1.read(qtr1_vals);
  }
  else {
    qtr2.read(qtr2_vals);
  }

  // check middle 4 sensors
  for (int i = 2; i < 6; i++) {
    if (going_forward) {
      if (qtr1_vals[i] < white_threshold) {
        white = false;
        if (white == false && qtr1_vals[i] > black_threshold) {
          black = false;
          return 'n';
        }
      }
    }
    else {
      if (qtr2_vals[i] < white_threshold) {
        white = false;
        if (white == false && qtr2_vals[i] > black_threshold) {
          black = false;
          return 'n';
        }
      }
    }
  }

  if (white == true) {
    return 'w';
  }
  else if (black == true) {
    return 'b';
  }
  else {
    return 'n';
  }
}

// TODO: Search for white/black
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

char read_color() {
  int num_reads = 5;
  int delay_time = 1;
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

  return determine_color(red, green, blue);
}

void read_uno() {
  if (Serial2.available()){
    uno_message = Serial2.readStringUntil('\n');  // read a String sent from Uno
    uno_message = uno_message.substring(0, uno_message.length()-1); // trim trailing white space
    Serial2.read();                               // clear new line character from buffer
    delay(1);

    // recieve bounty color from uno
    if (uno_message.length() > 4 && uno_message.substring(0,5) == "color") {
      bounty_color = uno_message[7];
      identify_opposite_color();
      debug("Bounty color:\t");
      debug(bounty_color);
      debug("\tOpposite color:\t");
      debugln(opposite_color);
    }
    else if (uno_message.length() > 3 && uno_message.substring(0,4) == "test") {
      test_state = !test_state;
    }
    else {  // receive state commands from uno
      state = uno_message;
      debug("state set to: ");
      debugln(state);
    }
  }
}

void read_mega() {
  if (Serial.available()){
    uno_message = Serial.readStringUntil('\n');
    Serial.read();
    delay(1);

    // recieve bounty color from uno
    if (uno_message.length() > 4 && uno_message.substring(0,5) == "color") {
      bounty_color = uno_message[7];
      identify_opposite_color();
      debug("Bounty color:\t");
      debug(bounty_color);
      debug("\tOpposite color:\t");
      debugln(opposite_color);
    }
    else if (uno_message.length() > 3 && uno_message.substring(0,4) == "test") {
      test_state = !test_state;
    }
    else {  // receive state commands from uno
      state = uno_message;
      debug("state set to: ");
      debugln(state);
    }
  }
}

// TODO: Redo threshold after circuit rework
void read_hall() {
  int threshold = 625;
  int reading = analogRead(pin_hall);
  debugln(reading);
  delay(250);
  if (reading >= threshold) {
    at_magnet = true;
  }
  else {
    at_magnet = false;
  }
}

void reset_encoder_tracking() {
  stop_motors();
  radians_traveled = 0;
  theta_traveled_m1_old = 0;
  theta_traveled_m2_old = 0;
  EncoderM1.write(0);
  EncoderM2.write(0);
  debug("Destination Reached");
}

// TODO: tweak threshold on actual gate
bool sense_gate() {
  int stop_dist = 255; // mm
  int dist = sharpF.getDist();
  debug("Distance: ");
  debug(dist);
  debugln(" mm");
  if (dist <= stop_dist) {
    return true;
  }
  return false;
}

void stop_motors() {
  mshield.setM1Speed(0);
  mshield.setM2Speed(0);
  digitalWrite(pin_motor_w1, LOW);
  digitalWrite(pin_motor_w2, LOW);
}

// TODO
void turn_cw() {
}

// TODO
void turn_ccw() {
}