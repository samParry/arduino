// Author: Sam Parry u1008557

/****************************
 ** #defines and #includes **
 ****************************/
#define DEBUG 2

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
const int pin_motor_w1 = 11;
const int pin_motor_w2 = 13;

const int pin_servo_w = 46;
const int pin_servo_c = 45;
const int pin_servo_h = 44;

const uint8_t pins_qtr1[] = {22, 24, 26, 28, 30, 32, 34, 36};
const uint8_t pins_qtr2[] = {23, 25, 27, 29, 31, 33, 35, 37};

const byte pin_sharpC = A9;
const byte pin_sharpL = A10;
const byte pin_sharpR = A11;
const byte pin_sharpF = A12;

const byte pin_color = A15;
const int pin_red = 48;
const int pin_green = 50;
const int pin_blue = 52;
const byte pin_hall = A8;

const byte pin_freq_filt = A6;
const byte pin_freq_raw = A7;

// blue = PWR
// green = GND
const int pin_encoder_m1a = 18;
const int pin_encoder_m1b = 19;
const int pin_encoder_m2a = 20;
const int pin_encoder_m2b = 21;

// *Reflectant Sensor Variables*
QTRSensors qtr1;
QTRSensors qtr2;
int16_t qtr1_biases[] = {327, 276, 276, 271, 264, 276, 276, 309};
int16_t qtr2_biases[] = {374, 326, 326, 138, 319, 360, 326, 405};
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
int servoW_down_angle = 122;
int servoW_up_angle = 138;
int servoC_open_angle = 165;
int servoC_term_angle = 133;
int servoC_shut_angle = 128;
int servoH_down_angle = 0;
int servoH_term_angle = 7;
int servoH_mid_angle = 30;
int servoH_up_angle = 65;
int servoH_start_angle = 100;

// *Motor variables*
DualTB9051FTGMotorShieldUnoMega mshield;
double base_speed = 200;
double pid_scaling = base_speed/100.0;  // linear PID scaling factor
double m1_speed = base_speed;
double m2_speed = base_speed;

// *Encoder variables*
Encoder EncoderM1(pin_encoder_m1a , pin_encoder_m1b);
Encoder EncoderM2(pin_encoder_m2a,  pin_encoder_m2b);
double theta_traveled_m1_old = 0;
double theta_traveled_m2_old = 0;
double dist_traveled = 0;

// *State Variables*
String state = "";
bool test_state = false;
bool going_forward = false;
char bounty_color = 'r';
char opposite_color = 'b';
String turn_dir = "cw";
String uno_message;

// *Time Variable*
float t;
float last_time;
float current_time;
float delta_time;

void setup() {

  // *Configure pins*
  pinMode(pin_red, OUTPUT);
  pinMode(pin_green, OUTPUT);
  pinMode(pin_blue, OUTPUT);
  pinMode(pin_hall, INPUT);
  pinMode(pin_freq_filt, INPUT);
  pinMode(pin_freq_raw, INPUT);
  pinMode(pin_motor_w1, OUTPUT);
  pinMode(pin_motor_w2, OUTPUT);

  // kill power to declared pins
  digitalWrite(pin_motor_w1, LOW);
  digitalWrite(pin_motor_w2, LOW);
  stop_motors();
  color_off();

  // *Write beginning servo angles*
   servoW.attach(pin_servo_w);
   servoC.attach(pin_servo_c);
   servoH.attach(pin_servo_h);
   servoW.write(servoW_up_angle);
   servoC.write(servoC_open_angle);
   servoH.write(servoH_start_angle);

  // *Initialize sensors*
  qtr1.setTypeRC();
  qtr2.setTypeRC();
  qtr1.setSensorPins(pins_qtr1, 8);
  qtr2.setSensorPins(pins_qtr2, 8);
  sharpC.setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);
  sharpL.setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);
  sharpR.setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);
  sharpF.setModel(SharpDistSensor::GP2Y0A41SK0F_5V_DS);

  // *Initialize motor driver*
  mshield.init();
  mshield.enableDrivers();
  mshield.flipM2(true);

  // *Initialize communication
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

  // *** START ***
  else if (state == "start") {
    start();
  }
  else if (state == "ahub") {
    approach_hub();
  }
  else if (state == "turn_hub1") {
    turn_hub_from_entrance();
  }
  else if (state == "enter_hub") {
    enter_hub();
  }
  else if (state == "turn_hub2") {
    turn_hub_from_hub();
  }


  // *** CANYON ***
  else if (state == "enter_canyon") {
    enter_canyon();
  }
  else if (state == "face_block") {
    face_bounty();
  }
  else if (state == "terminate") {
    terminate();
  }
  else if (state == "block_canyon") {
    get_block_canyon();
  }
  else if (state == "to_center") {
    back_to_center();
    t = millis(); // needed for go_home_canyon
  }
  else if (state == "go_home_canyon") {
    go_home_canyon();
  }


  // *** CAVE ***
  else if (state == "enter_cave") {
    enter_cave();
  }
  else if (state == "trav_cave1") {
    traverse_cave();
  }
  else if (state == "mudhorn") {
    mudhorn();
  }
  else if (state == "block_cave") {
    get_block_cave();
  }
  else if (state == "back_to_cave") {
    back_to_cave_walls();
  }
  else if (state == "trav_cave2") {
    traverse_cave_backward();
  }
  else if (state == "cave_enter_hub") {
    cave_enter_hub();
  }


  // *** COMPOUND ***
  else if (state == "agate") {
    approach_gate();
  }
  else if (state == "gate") {
    raise_gate();
  }
  else if (state == "block_compound") {
    get_block_compound();
  }
  else if (state == "comp_to_hub") {
    compound_to_hub();
  }
  else if (state == "comp_ent_hub") {
    compound_enter_hub();
  }


  // *** RETURN ***
  else if (state == "hub_back") {
    turn_hub_back();
    t = millis(); // set time for go_home()
  }
  else if (state == "go_home") {
    go_home();
  }
  else if (state == "s") {
    stop_motors();
    state = "";
  }
}

/**********************
 ** Testing Function **
 **********************/

void test_mode() {
  stop_motors();
  debugln("\n           Test Mode");
  debugln("---------------------------------------------------");
  debugln("Command    Description    m=menu, gf=toggle forward");
  debugln("---------------------------------------------------");
  debugln("at         At LED?");
  debugln("c          Color sensor");
  debugln("chomp      Chomp at the bit");
  debugln("co         Claw open");
  debugln("cm         Claw mid angle");
  debugln("cs         Claw shut");
  debugln("el         Straight line (encoder)");
  debugln("f          Compare frequencies");
  debugln("h          Hall effect Sensor");
  debugln("hd         Hook arm down");
  debugln("hm         Hook arm mid");
  debugln("hu         Hook arm up");
  debugln("hs         Hook arm starting angle");
  debugln("ht         Hook arm terminate angle");
  debugln("irc        Rangefinder (block-facing)");
  debugln("irf        Rangefinder (front-facing)");
  debugln("irl        Rangefinder (left-facing)");
  debugln("irr        Rangefinder (right-facing)");
  debugln("lb         Line following (backward)");
  debugln("lf         Line following (forward)");
  debugln("ocw        Correct offset CW");
  debugln("occw       Correct offset CCW");
  debugln("oh         Correct offset Hall");
  debugln("qtr        Calibrate QTR sensors");
  debugln("qtrbwf     Scan for black/white with QTRs (forward)");
  debugln("qtrbwb     Scan for black/white with QTRs (backward)");
  debugln("qtrp1      Print forward facing QTRs");
  debugln("qtrp2      Print rear facing QTRs");
  debugln("tcw        Turn clockwise");
  debugln("tccw       Turn counter-clockwise");
  debugln("tcw45      Turn clockwise 45 deg");
  debugln("tccw45     Turn counter-clockwise 45 deg");
  debugln("tcw55      Turn clockwise 55 deg");
  debugln("tccw55     Turn counter-clockwise 55 deg");
  debugln("wb         Follow wall (backward)");
  debugln("wf         Follow wall (forward)");
  debugln("wcw        Turn wheel clockwise");
  debugln("wccw       Turn wheel counter-clockwise");
  debugln("wu         Raise hub wheel");
  debugln("wd         Lower hub wheel");

  while (test_state) {
    // Read comms at top of loop
    read_uno();
    read_mega();

    // At LED
    if (state == "at") {
      bool at = at_led();
      delay(250);
    }

    // Color sensor
    else if (state == "c") {
      char color = read_color();
      delay(200);
    }

    // Chomp at the bit
    else if (state == "chomp") {
      chomp_at_the_bit();
      state = "";
    }

    // Claw
    else if (state == "co") {
      claw_open();
      state = "";
    }
    else if (state == "cm") {
      servoC.write(servoC_term_angle);
      state = "";
    }
    else if (state == "cs") {
      claw_shut();
      state = "";
    }

    // Encoders
    else if (state == "el") {
      double drive_dist = 100; // mm
      base_speed = 400;
      if (dist_traveled < drive_dist) {
        drive_straight(drive_dist);
        debugln(dist_traveled);
      }
      else {
        stop_motors();
        state = "";
      }
    }

    // Compare frequencies
    else if (state == "f") {
      compare_frequency();
    }

    // toggle forward direction
    else if (state == "gf") {
      if (going_forward) {
        going_forward = false;
        debugln("Going backward (wheel first)");
      }
      else {
        going_forward = true;
        debugln("going forward (claw first)");
      }
      state = "";
    }

    // Hall Effect sensor
    else if (state == "h") {
      bool at_magnet = read_hall();
      delay(100);
      if (at_magnet) {
        debugln("Magnet detected!");
        claw_shut();
        delay(100);
        claw_open();
        delay(100);
      }
    }

    // Hook arm
    else if (state == "hd") {
      servoH.write(servoH_down_angle);
      debugln(servoH.read());
      state = "";
    }
    else if (state == "hm") {
      servoH.write(servoH_mid_angle);
      debugln(servoH.read());
      state = "";
    }
    else if (state == "hu") {
      servoH.write(servoH_up_angle);
      debugln(servoH.read());
      state = "";
    }
    else if (state == "hs") {
      servoH.write(servoH_start_angle);
      debugln(servoH.read());
      state = "";
    }
    else if (state == "ht") {
      servoH.write(servoH_term_angle);
      debugln(servoH.read());
      state = "";
    }    

    // Measure distances (mm)
    else if (state == "irc") {
      debugln(sharpC.getDist());
      delay(1000);
    }
    else if (state == "irf") {
      debugln(sharpF.getDist());
      delay(500);
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

    // Correct Hub Offset
    else if (state == "ocw") {
      debugln("Correcting offset cw");
      correct_hub_offset("cw");
      state = "";
    }
    else if (state == "occw") {
      debugln("Correcting offset ccw");
      correct_hub_offset("ccw");
      state = "";
    }
    else if (state == "oh") {
      debugln("Correcting offset by hall sensor");
      correct_hub_offset("h");
      state = "";
    }

    // QTRs
    else if (state == "qtr") {
      calibrate_qtrs();
      state = "";
    }
    else if (state == "qtrbwf") {
      going_forward = true;
      char color = qtr_black_or_white(true);
      
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
      char color = qtr_black_or_white(true);
      
      // print readings
      debugln("qtr2 values");
      for (int i = 0; i < 8; i++) {
        debug(qtr2_vals[i]);
        debug('\t');
      }
      debugln(color);      
    }
    else if (state == "qtrp1") {
      qtr1.read(qtr1_vals);
      print_qtr(1);
      delay(250);
    }
    else if (state == "qtrp2") {
      qtr1.read(qtr2_vals);
      print_qtr(2);
      delay(250);
    }

    // Turning
    else if (state == "tcw") {
      turn_cw();
      state = "";
    }
    else if (state == "tccw") {
      turn_ccw();
      state = "";
    }
    else if (state == "tcw45") {
      turn_cw_45();
      state = "";
    }
    else if (state == "tccw45") {
      turn_ccw_45();
      state = "";
    }
    else if (state == "tcw55") {
      turn_cw_55();
      state = "";
    }
    else if (state == "tccw55") {
      turn_ccw_55();
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
    else if (state == "wcw") {
      instrument_motor_cw();
    }
    else if (state == "wccw") {
      instrument_motor_ccw();
    }
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

void print_qtr(int num) {
  if (num == 1) {
    debugln("qtr1 values");
    for (int i = 0; i < 8; i++) {
      debug(qtr1_vals[i]);
      debug('\t');
    }
    debug(qtr_black_or_white(1));
    debugln();
  }
  else if (num == 2) {
    debugln("qtr2 values");
    for (int i = 0; i < 8; i++) {
      debug(qtr2_vals[i]);
      debug('\t');
    }
    debug(qtr_black_or_white(2));
    debugln();
  }
}

/****************************
 ** State Functions: Start **
 ****************************/

void start() {
  // get over the line with a timed motor burn
  debugln("Starting");
  base_speed = 400;
  going_forward = false;
  t = millis();
  while (millis() - t <= 2000) {
    follow_line();
  }
  state = "ahub"; debugln("Approaching Hub");
}

void approach_hub() {
  // follow line until black tape
  char color = qtr_black_or_white(true);
  if (color != 'w') {
    follow_line();
  }
  else {
    stop_motors();

    // scoot back a little
    delay(50); // trying to prevent wheely
    base_speed = 200;
    t = millis();
    while (millis() - t < 300) {
      mshield.setM1Speed(100);
      mshield.setM2Speed(100);
    }
    mshield.setM1Speed(0);
    mshield.setM2Speed(0);
    base_speed = 400;
    state = "turn_hub1"; debugln("Turning Hub");
  }
}

void turn_hub_from_entrance() {

  // turn hub till magnet sensed
  servoW.write(servoW_down_angle);
  instrument_motor_cw();
  while (not read_hall()) {
  }
  stop_motors();

  // Turn the hub to account for offset sensor
  correct_hub_offset("h");
  if (bounty_color == 'g') {
    state = "enter_canyon"; debugln("Entering Canyon");
  }
  else {
    state = "enter_hub"; debugln("Entering Hub");
  }
}

void enter_hub() {
  going_forward = false;
  reset_encoder_tracking();
  base_speed = 400;
  // int drive_dist = 457; // hub is 18" (457mm) in diameter
  // int drive_dist = 225; // dist for base_speed = 200
  int drive_dist = 223; // dist for base_speed = 400
  while (dist_traveled < drive_dist) {
    drive_straight(drive_dist);
  }
  stop_motors();
  state = "turn_hub2"; debugln("Turning Hub");
}

void turn_hub_from_hub() {

  // drop/rotate wheel arm
  servoW.write(servoW_down_angle);
  if (bounty_color == 'r') {
    instrument_motor_cw();
  }
  else if (bounty_color == 'b') {
    instrument_motor_ccw();
  }

  // stop rotation when color is found
  char color = read_color();
  if (color == opposite_color) {

    // double check the reading
    if (read_color() == opposite_color) {

      // come to a full stop to prevent overrotation from momentum
      stop_motors();
      delay(250);
      
      // update state
      if (bounty_color == 'r') {
        correct_hub_offset("cw");
        state = "enter_cave"; debugln("Entering Cave");
      }
      else if (bounty_color == 'b') {
        correct_hub_offset("compound");
        state = "agate"; debugln("Approaching Gate");
      }      
    }
  }
}

/*****************************
 ** State Functions: Canyon **
 *****************************/

void enter_canyon() {

  // get through and hub/enter canyon
  base_speed = 400;
  going_forward = false;
  bool stop = at_led();
  if (!stop) {
    follow_line();
  }
  else {
    stop_motors();
    base_speed = 200;
    state = "face_block"; debugln("Turning to face bounty");
  }
}

void face_bounty() {

  // inch forward a bit
  reset_encoder_tracking();
  while (dist_traveled < 40) {
    drive_straight(40);
  }
  stop_motors();

  // detect frequency
  turn_dir = compare_frequency();

  // turn to face bounty (opposite of true bounty turn)
  if (turn_dir == "cw") {
    turn_cw();
  }
  else if (turn_dir == "ccw") {
    turn_ccw();
  }
  state = "terminate"; debugln("Terminating Bounty");
}

void terminate() {
  get_block();

  // release block to terminate
  t = millis();
  while (millis() - t < 300) {
    mshield.setM1Speed(-base_speed);
    mshield.setM2Speed(-base_speed);
  }
  mshield.setM1Speed(0);
  mshield.setM2Speed(0);
  claw_open();
  delay(50);

  // back up to clear block
  t = millis();
  while (millis() - t < 750) {
    mshield.setM1Speed(-base_speed);
    mshield.setM2Speed(-base_speed);
  }
  mshield.setM1Speed(0);
  mshield.setM2Speed(0);  
  state = "block_canyon"; debugln("Collecting Bounty");
}

void get_block_canyon() {
  // turn around and get bounty
  turn_cw();
  turn_cw();
  get_block();
  state = "to_center"; debugln("Returning to Center");
}

void back_to_center() {
  // back up to center point
  going_forward = false;
  t = millis();
  while (millis() - t < 1250) {
    mshield.setM1Speed(-base_speed);
    mshield.setM2Speed(-base_speed);
  }
  mshield.setM1Speed(0);
  mshield.setM2Speed(0);

  // turn opposite direction of original turn
  // faces the canyon wheel-first
  if (turn_dir == "cw") {
    turn_ccw();
  }
  else if (turn_dir == "ccw") {
    turn_cw();
  }
  state = "go_home_canyon"; debugln("Returning Home");
}

void go_home_canyon() {
  // The robot should have a line all the way to the start
  base_speed = 400;
  t = millis();
  while (millis() - t < 1160) {
    follow_line();
  }
  stop_motors();
}

/***************************
 ** State Functions: Cave **
 ***************************/

void enter_cave() {
  base_speed = 400;
  going_forward = true;
  char color = 'n';
  while (color != 'w') {
    follow_line();
    color = qtr_black_or_white(false);
  }
  state = "trav_cave1"; debugln("Traversing Cave");
}

void traverse_cave() {
  char color = qtr_black_or_white(true);
  if (color != 'b') {
    follow_wall();
  }
  else {
    stop_motors();
    state = "mudhorn"; debugln("Death to the Mudhorn");
  }
}

void mudhorn() {
  going_forward = true;
  base_speed = 400;
  servoH.write(servoH_term_angle);
  delay(50);

  // correct weird off center starting point
  t = millis();
  while (millis() - t < 100) {
    mshield.setM1Speed(300);
    mshield.setM2Speed(-400);
  }
  mshield.setM1Speed(0);
  mshield.setM2Speed(0);

  // // drive clear of cave walls
  t = millis();
  while (millis() - t < 700) {
    follow_line();
  }
  stop_motors();

  // death to the mudhorn (poke poke)
  turn_cw();
  turn_ccw();
  turn_ccw();
  turn_cw();

  // int speed = 200;
  // int turn_time = 800;
  // t = millis();
  // while (millis() - t < turn_time) { // cw
  //   mshield.setM1Speed(-speed);
  //   mshield.setM2Speed(speed);
  // }
  // t = millis();
  // while (millis() - t < turn_time) { // ccw
  //   mshield.setM1Speed(speed);
  //   mshield.setM2Speed(-speed);
  // }
  // t = millis();
  // while (millis() - t < turn_time) { // ccw
  //   mshield.setM1Speed(speed);
  //   mshield.setM2Speed(-speed);
  // }
  // t = millis();
  // while (millis() - t < turn_time) { // cw
  //   mshield.setM1Speed(-speed);
  //   mshield.setM2Speed(speed);
  // }
  // stop_motors();
  state = "block_cave"; debugln("Collecting Bounty");
}

// Mechanisms can't reach the bounty
void get_block_cave() {

  // back up to ensure straight approach
  going_forward = false;
  t = millis();
  while (millis() - t < 500) {
    follow_line();
  }
  stop_motors();

  chomp_at_the_bit();

  // terminate bounty (crash into cave)
  // t = millis();
  // while (millis() - t < 2500) {
  //   mshield.setM1Speed(300);
  //   mshield.setM2Speed(300);
  // }
  // mshield.setM1Speed(0);
  // mshield.setM2Speed(0);

  going_forward = true;
  t = millis();
  while (millis() - t < 1000) {
    follow_line();
  }
  stop_motors();

  state = "back_to_cave"; debugln("Returning to Walls");
}

void back_to_cave_walls() {
  going_forward = false;
  char color = qtr_black_or_white(true);
  if (color != 'w') {
    follow_line();
  }
  else {
    // inch back a bit more then switch to wall following
    time_burst(400, 'b');
    stop_motors();
    state = "trav_cave2"; debugln("Returning to Hub");
  }
}

// On Dec 8th 2021, this function worked first try. Hail. Hail.
void traverse_cave_backward() {
  going_forward = false;
  base_speed = 400;
  int no_wall_val = 120;
  int dist = sharpL.getDist();
  if (dist < no_wall_val) {
    follow_wall();
  }
  else {
    stop_motors();
    state = "cave_enter_hub"; debugln("Re-entering Hub");
  }
}

void cave_enter_hub() {
  base_speed = 400;
  reset_encoder_tracking();
  int drive_dist = 237;
  while (dist_traveled < drive_dist) {
    drive_straight(drive_dist);
    follow_line();
  }
  stop_motors();
  state = "hub_back"; debugln("Re-orienting Hub");
}

/*******************************
 ** State Functions: Compound **
 *******************************/

void approach_gate() {
  going_forward = true;
  // drop arm and drive to gate
  servoH.write(servoH_down_angle);
  delay(100);
  follow_line();
  if (sense_gate(210)) {
    stop_motors();
    state = "gate"; debugln("Lifting Gate");
  }
}

void raise_gate() {
  // raise gate
  servoH.write(servoH_up_angle);
  delay(900);
  servoH.write(servoH_mid_angle);
  delay(500);
  state = "block_compound"; debugln("Collecting Bounty");
}

void get_block_compound() {
  get_block();
  state = "comp_to_hub"; debugln("Returning to Hub");
}

void compound_to_hub() {
  going_forward = false;
  follow_line();
  char color = qtr_black_or_white(true);
  debugln(color);
  if (color == 'w') {
    stop_motors();
    state = "comp_ent_hub"; debugln("Back at Hub");
  }
}

void compound_enter_hub() {
  reset_encoder_tracking();
  double drive_dist = 225; // hub is 18" (457mm) in diameter
  while (dist_traveled < drive_dist) {
    drive_straight(drive_dist);
  }
  stop_motors();
  state = "hub_back"; debugln("Back on Hub");
}

/*****************************
 ** State Functions: Return **
 *****************************/

void turn_hub_back() {
  // drop/rotate wheel arm
  servoW.write(servoW_down_angle);
  if (bounty_color == 'r') {
    instrument_motor_cw();
  }
  else if (bounty_color == 'b') {
    instrument_motor_ccw();
  }

  // stop rotation when black tape is found
  char color = read_color();
  if (color == 'k') {
    correct_hub_offset("cw");
    servoW.write(servoW_up_angle);
    state = "go_home"; debugln("Returning Home");
  }
}

void go_home() {
  // The robot should have a line all the way to the start
  if (millis() - t <= 3000) {
    follow_line();
  }
  else {
    if (qtr_black_or_white(2) == 'n') {
      follow_line();
    }
    else {
      reset_encoder_tracking();
      while (dist_traveled < 300) {
        drive_straight(300);
      }
      stop_motors();
      servoH.write(servoH_up_angle);
      state = ""; debugln("Bounty Confirmed");
    }
  }
}

/********************
 ** Core Functions **
 ********************/

bool at_led() {
  int raw = analogRead(pin_freq_raw);
  int filt = analogRead(pin_freq_filt);
  int diff = abs(raw - filt);
  if (diff > 20) {
    return true;
  }
  else {
    return false;
  }
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

void chomp_at_the_bit() {
  claw_shut();
  servoH.write(servoH_term_angle);
  delay(200);
  claw_open();
  servoH.write(servoH_mid_angle);
  delay(200);
  claw_shut();
  servoH.write(servoH_term_angle);
  delay(200);
  claw_open();
  servoH.write(servoH_mid_angle);

  time_burst(175, 'b');
  time_burst(175, 'f');

  claw_shut();
  servoH.write(servoH_term_angle);
  delay(200);
  claw_open();
  servoH.write(servoH_mid_angle);
  delay(200);  
  claw_shut();
  servoH.write(servoH_term_angle);
  delay(200);
  claw_open();
  servoH.write(servoH_mid_angle);

  time_burst(175, 'b');
  time_burst(175, 'f');

  delay(200);  
  claw_shut();
  servoH.write(servoH_term_angle);
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

String compare_frequency() {
  int num_reads = 100;
  int amp_diff = 0;

  int filt = 0;
  int filt_max = 0;
  int filt_min = 0;
  int filt_amp = 0;

  int raw = 0;
  int raw_max = 0;
  int raw_min = 0;
  int raw_amp = 0;

  for (int i = 0; i < num_reads; i++) {
    raw = analogRead(pin_freq_raw);
    filt = analogRead(pin_freq_filt);

    if (i > 10) {
      filt_max = (filt > filt_max) ? filt : filt_max;
      filt_min = (filt < filt_min) ? filt : filt_min;
      raw_max = (raw > raw_max) ? raw : raw_max;
      raw_min = (raw < raw_min) ? raw : raw_min;
    }
  }
  raw_amp = raw_max - raw_min;
  filt_amp = filt_max - filt_min;
  amp_diff = abs(raw_amp - filt_amp);
  
  // debug("Amp diff: "); debugln(amp_diff);

  // HIGH = Turn left (experimental: 60 -> 200)
  // LOW = Turn right (experimental: 01 -> 80)
  if (amp_diff < 50) {
    return "ccw";
  }
  else {
    return "cw";
  }
}

void correct_hub_offset(String dir) {
  servoW.write(servoW_down_angle);
  t = millis();
  if (dir == "cw") {
    while(millis() - t < 250) {
      instrument_motor_cw(); 
    }
  }
  else if (dir == "ccw") {
    while(millis() - t < 250) {
      instrument_motor_ccw(); 
    }
  }
  else if (dir == "compound") {
    while(millis() - t < 500) {
      instrument_motor_cw(); 
    }    
  }
  // offset for hall effect is always ccw and 
  // requires a different length then offset from 
  // color sensors
  else if (dir == "h") {
    while(millis() - t < 200) {
      instrument_motor_cw();
    }
  }
  stop_motors();
  delay(250); // lifting up right away causes hub to keep spinning
  servoW.write(servoW_up_angle);
}

char determine_color(int r, int g, int b) {

  // determine color
  char color;
  int w_threshold = 700;
  int k_threshold = 260;
  if (r >= w_threshold && g >= w_threshold && b >= w_threshold) {
    color = 'w';
  }
  else if (r <= k_threshold && g <= k_threshold && b <= k_threshold) {
    color = 'k';
  }

  // ensures colors are above black threshold
  else if (r >= 400 && r >= g && r >= b) {
    color = 'r';
  }
  else if (g >= k_threshold && g >= r && g >= b) {
    color = 'g';
  }
  else if (b >= 400 && b >= r && b >= g) {
    color = 'b';
  }

  // if (test_state == true) {
  if (true == true) {
    debug(r);
    debug("\t");
    debug(g);
    debug("\t");
    debug(b);
    debug("\tColor:\t");
    debugln(color);
  }
  return color;
}

void drive_straight(double drive_dist) {
  
  // encoder variables
  double wheel_radius = 35; //mm
  double GearRatio = 100;
  double countsPerRev = 64;
  double RevsShaft2Rad = 2 * PI;
  double required_radians = drive_dist / wheel_radius;
  double Kp = 1.5;
  
  // time based variables
  current_time = millis();
  delta_time = current_time / 1000.0 - last_time;

  // read encoders
  double count_m1;
  double count_m2;
  if (going_forward) {
    count_m1 = EncoderM1.read();
    count_m2 = -EncoderM2.read();
  }
  else {
    count_m1 = -EncoderM1.read();
    count_m2 = EncoderM2.read();
  }

  // solve for radians traveled
  double theta_traveled_m1 = abs((count_m1 * RevsShaft2Rad) * (1 / (countsPerRev * GearRatio)));
  double theta_traveled_m2 = abs((count_m2 *  RevsShaft2Rad) * (1 / (countsPerRev * GearRatio)));

  // solve for velocities
  double velocity_m1 = (theta_traveled_m1 - theta_traveled_m1_old)/delta_time;
  double velocity_m2 = (theta_traveled_m2 - theta_traveled_m2_old)/delta_time;
  double velocity_average = (velocity_m1 + velocity_m2)/2;
  // debug("vel ave: "); debug(velocity_average); debug('\t');

  // solve for the next desired radian count
  double theta_d1 = theta_traveled_m1 + velocity_average * (delta_time);
  double theta_d2 = theta_traveled_m2 + velocity_average * (delta_time);

  //Solve for voltages
  m1_speed = base_speed + Kp * (theta_d1 - theta_traveled_m1);
  m2_speed = base_speed + Kp * (theta_d2 - theta_traveled_m2);
  debug(m1_speed); debug('\t'); debugln(m2_speed);

  // set Motor voltages
  if (going_forward) {
    mshield.setM1Speed(m1_speed);
    mshield.setM2Speed(m2_speed);
  }
  else {
    mshield.setM1Speed(-m1_speed);
    mshield.setM2Speed(-m2_speed);
  }
  
  // record global variables
  dist_traveled = wheel_radius * (theta_traveled_m1 + theta_traveled_m2) / 2;
  theta_traveled_m1_old = theta_traveled_m1;
  theta_traveled_m2_old = theta_traveled_m2;
  last_time = current_time / 1000.0;
  // debug("dist trav: "); debug(dist_traveled); debugln(" mm");
}

void follow_line() {
  double Kp = 60, Kd = 60;
  float error_p, error_d, error;
  current_time = millis();
  delta_time = current_time - last_time;

  // read errors
  error_p = line_error();
  error_d = (error_p - last_error)/delta_time;
  error = Kp*error_p + Kd*error_d;
  error *= pid_scaling; // apply pid error scaling

  // debug("error_p:\t");
  // debug(error_p);
  // debug("\terror_d\t");
  // debug(error_d);
  // debug("\terror:\t");
  // debug(error);
  // debug("\tdt:\t");
  // debugln(delta_time);

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
  double Kp = 10, Kd = 10;
  float error_p, error_d, error;
  current_time = millis();
  delta_time = current_time - last_time;
  int optimal_dist = 50; // mm
  int dist;

  // handles large delta time from first function call
  if (delta_time >= 100) {
    delta_time = 52;
  }

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
  
  // debug("dist: ");
  // debugln(dist);
  // delay(1);
  // debug("\terror_p: ");
  // debug(error_p);
  // debug("\terror_d ");
  // debug(error_d);
  // debug("\terror: ");
  // debugln(error);

  mshield.setM1Speed(m1_speed);
  mshield.setM2Speed(m2_speed);
  last_time = current_time;
}

void get_block() {
  going_forward = true;
  claw_open();
  while (sharpC.getDist() > 20) {
    follow_line();
  }
  stop_motors();
  
  // // move forward 2cm
  // t = millis();
  // while (millis() - t < 100) {
  //   mshield.setM1Speed(base_speed);
  //   mshield.setM2Speed(base_speed);
  // }
  // stop_motors();
  
  claw_shut();
  delay(250); // wait for claw to shut
  going_forward = false;
  debugln("Bounty Secured");
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

char qtr_black_or_white(bool read) {
  // returns 'w' for white
  // returns 'b' for black
  // returns 'n' for neither

  int white_threshold = 500;
  int black_threshold = 2000;
  bool black = true;
  bool white = true;

  // read qtr
  if (read) {
    if (going_forward) {
      qtr1.read(qtr1_vals);
    }
    else {
      qtr2.read(qtr2_vals);
    }
  }

  // check middle 4 sensors
  for (int i = 2; i < 6; i++) {
    if (going_forward) {
      if (qtr1_vals[i] > white_threshold) {
        white = false;
        if (white == false && qtr1_vals[i] < black_threshold) {
          black = false;
          return 'n';
        }
      }
    }
    else {
      if (qtr2_vals[i] > white_threshold) {
        white = false;
        if (white == false && qtr2_vals[i] < black_threshold) {
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
  red = red/num_reads - 200;
  green = green/num_reads;
  blue = blue/num_reads - 200;

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
      state = "start";
    }
    else if (uno_message.length() > 3 && uno_message.substring(0,4) == "test") {
      test_state = !test_state;
      debug("Test Mode: "); debugln(test_state);
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
      debug("Test Mode: "); debugln(test_state);
    }
    else {  // receive state commands from uno
      state = uno_message;
      debug("state set to: ");
      debugln(state);
    }
  }
}

bool read_hall() {
  int baseline = 555; // approximate
  int threshold = 400;
  int reading = analogRead(pin_hall);
  int diff = abs(reading - baseline); // abs diff
  // debug("reading: "); debug(reading); debug("\tdiff: "); debugln(diff);
  if (diff > threshold) {
    // debugln(reading); delay(1);
    return true;
  }
  else {
    return false;
  }
}

void reset_encoder_tracking() {
  dist_traveled = 0;
  theta_traveled_m1_old = 0;
  theta_traveled_m2_old = 0;
  EncoderM1.write(0);
  EncoderM2.write(0);
}

bool sense_gate(int stop_dist) {
  int dist = sharpF.getDist();
  // debug("Distance: ");
  // debugln(dist);
  // debugln(" mm");
  if (dist <= stop_dist) {
    // double check reading to avoid false positives from noise
    if (sharpF.getDist() <= stop_dist) {
      return true; 
    }
  }
  return false;
}

void stop_motors() {
  mshield.setM1Speed(0);
  mshield.setM2Speed(0);
  digitalWrite(pin_motor_w1, LOW);
  digitalWrite(pin_motor_w2, LOW);
  if (EncoderM1.read() != 0) {
    debugln("Encoders Reset");
    reset_encoder_tracking();
  }
}

void time_burst(int time, char dir) {
  t = millis();
  while (millis() - t < time) {
    if (dir == 'f') {
      mshield.setM1Speed(400);
      mshield.setM2Speed(400);
    }
    else if (dir == 'b') {
      mshield.setM1Speed(-400);
      mshield.setM2Speed(-400);      
    }
  }
  mshield.setM1Speed(0);
  mshield.setM2Speed(0);  
}

void turn_cw() {
  // int speed = 200;
  // int turn_time = 1100;
  int speed = 400;
  int turn_time = 550;
  t = millis();
  while (millis() - t < turn_time) {
    mshield.setM1Speed(-speed);
    mshield.setM2Speed(speed);
  }
  stop_motors();
}

void turn_cw_45() {
  int speed = 400;
  int turn_time = 275;
  t = millis();
  while (millis() - t < turn_time) {
    mshield.setM1Speed(-speed);
    mshield.setM2Speed(speed);
  }
  stop_motors();  
}

void turn_cw_55() {
  int speed = 400;
  int turn_time = 335;
  t = millis();
  while (millis() - t < turn_time) {
    mshield.setM1Speed(-speed);
    mshield.setM2Speed(speed);
  }
  stop_motors();  
}

void turn_ccw() {
  // int speed = 200;
  // int turn_time = 1100;
  int speed = 400;
  int turn_time = 575;
  t = millis();
  while (millis() - t < turn_time) {
    mshield.setM1Speed(speed);
    mshield.setM2Speed(-speed);
  }
  stop_motors();
}

void turn_ccw_45() {
  int speed = 400;
  int turn_time = 285; 
  t = millis();
  while (millis() - t < turn_time) {
    mshield.setM1Speed(speed);
    mshield.setM2Speed(-speed);
  }
  stop_motors();
}

void turn_ccw_55() {
  int speed = 400;
  int turn_time = 335;
  t = millis();
  while (millis() - t < turn_time) {
    mshield.setM1Speed(speed);
    mshield.setM2Speed(-speed);
  }
  stop_motors();  
}
