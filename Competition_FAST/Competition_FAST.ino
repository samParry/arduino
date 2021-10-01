/****************************************************************
  Author Name:Kenzie Hoggan () & Sam Parry ()
  Date: 11/14/19
  Sketch Name: Demo4
  Sketch Description: Demonstrates the full functionality of Earl in all of his glory

  Pin Usage:    Pin type/number     Hardware
              ----------------    ----------------
              digital pin 13      IR LED
              digital pin 12      Right Limit Switch
              digital pin 11      Left Limit Switch
              digital pin 10      reloader Servo
              digital pin 9       launcher Servo
              digital pin 4       motor direction
              digital pin 5       motor power
              analog pin A5       IR sensor
              digital pin 6       Solenoid power
              digital pin 7       Solenoid direction
*******************************************************************/

/****************************
 ** #defines and #includes **
 ****************************/
#include <Servo.h>
/***********************
 ** Global Variables ***
 ***********************/
// *** Declare & Initialize Pins ***
const int IRLEDPin = 13;
const int rightSwitchPin = 12;
const int leftSwitchPin = 11;
const int reloaderServoPin = 10;
const int launcherServoPin = 9;
const int motorDirectionPin = 4;
const int motorPowerPin = 5 ;
const int irSensorPin = A5;
const int solPowerPin = 6;
const int solDirecPin = 7;

// *** Create Servo Objects ***
Servo reloaderServo;    // creates a servo object for the reloader
Servo launcherServo;    // creates a servo object for the launcher

// *** Declare & Initialize Program Variables ***
boolean outputToggle = 0; // put a 1 to print out intermediate results for de-bugging ...

// *DC motor variables*
const int motorPower = 255;   // motor power
boolean motorOn = 0;          // toggles motor on/off
boolean motorLeft = 0;    // if true, direction is left, if false, direction is right
int reverseTime = 40;     // experimentlally determined for the BrakeMotor() function - allows Earl to stop 'on a dime'

// *Solenoid variables*
int solPower = 255;         // solenoid power
int solActivTime = 500;     // solenoid activation time [milliseconds]

// *Encoder variables*
boolean black = 1;
int irSensorReading = 0;
int lowThreshold = 150;   // value that irSensorReading must be UNDER when over a WHITE stripe
int highThreshold = 600;  // value that irSensorReading must be OVER when over a BLACK stripe
boolean stripeBlack;
unsigned long stripeTime;
unsigned long currentTime = 0;
int counts = 24;            // the default value set to counts
int desiredPosition = 24;   // the default value of desiredPosition
const int reloaderPosition = 100;      // count number under the reloader
const int homePosition = -45;          // count number at home position

// *Reloader variables*
int reloaderHoldAngle = 40;     // angle that holds the balls in the reloader
int reloaderDispenseAngle = 10; // angle that dispenses balls from the reloader

// *Targeting variables*
int launcherServoAngle = 90;  // the default angle of the launcher
int servoSmallIncrement = 1;
int servoLargeIncrement = 5;
byte target = 0;             // variable to keep track of which target you are on
//// byte driveTo[6] = {10, 15, 20, 25, 30, 35};       // encoder positions of targets in cm TESTING
//// byte driveTo[6] = {10, 10, 10, 10, 10, 10};       // encoder positions for the far left row of target holes
//// byte driveTo[6] = {24, 24, 24, 24, 24, 24};       // encoder positions for the middle row of target holes
//// byte driveTo[6] = {39, 39, 39, 39, 39, 39};       // encoder positions for the far right row of target holes
//// encoder positions for first, second and third row of targets used to test demo 2
//   byte driveTo[22] = {8,8,8,8, 24, 24, 24, 24, 39, 39, 39, 8,8,8,8 , 24, 24, 24, 24, 39, 39, 39};
//
//// int writeToServo[6] = {30, 50, 70, 90, 110, 130};     // servo angles in degrees TESTING
//// int writeToServo[6] = {123, 116, 106, 95, 123, 116};  // Servo angles required to hit the middle row of targets
//// int writeToServo[6] = {122, 112, 102, 122, 112, 102}; // Servo angles required to hit the far right row of targets
//// encoder positions for first, second and third row of targets used to test demo 2
//  int writeToServo[22] = {127, 118, 109, 99, 123, 115, 106, 95, 122, 112, 102,127, 118, 109, 99, 124, 115, 106, 95,122, 112, 102};

// *Launcher variables*
int leftSwitchReading = 0;  // the condition of the left contact switch
int rightSwitchReading = 0; // the condition of the right contact switch

// *Demo 3 variables*
byte val = 69;        // initializing a variable doesn't matter what it's set to at first ...
byte driveTo[6];      // initializing a vector to hold encoder positions
float xTarget[6];     // a vector to hold target distances
int writeToServo[6];  // a vector to hold servo angles
/********************
 ** Setup Function **
 ********************/
void setup(void) {
  // PUT YOUR SETUP CODE HERE, TO RUN ONCE:


  // *** Configure Pins & Attach Servos ***
  pinMode(IRLEDPin, OUTPUT);               //IR LED pin
  pinMode(rightSwitchPin, INPUT_PULLUP);   // right limit switch
  pinMode(leftSwitchPin, INPUT_PULLUP);    // left limit switch
  reloaderServo.attach(reloaderServoPin);  // reloader Servo
  launcherServo.attach(launcherServoPin);  // launcher Servo
  pinMode(motorDirectionPin, OUTPUT);      // Motor Direction
  pinMode(motorPowerPin, OUTPUT);  // Motor Power
  pinMode(solPowerPin, OUTPUT);    // Solenoid Power
  pinMode(solDirecPin, OUTPUT);    // Solenoid Direction

  // *** Initialize Serial Communication ***
  Serial.begin(9600);
  // Serial.begin(57600);
  Serial.write('a');

  // *** Take Initial Readings ***
  // *** Move Hardware to Desired Initial Positions - PART 1 ***
  reloaderServo.write(reloaderHoldAngle);   // moves reloader to intial hold position

  // *** Get Data from MATLAB ***
  int k;
  for (k = 0; k < 6; ++k) {
    while (Serial.available() < 3) {  // waiting for 3 inputs from MATLAB that's where the < 3 comes from
      // do nothing
    }
    byte encoderPos = Serial.read();   // reads the encoder position from MATLAB
    byte xTarget_HB = Serial.read();   // reads the 'high-byte' xTarget value from MATLAB
    byte xTarget_LB = Serial.read();   // reads the 'low-byte' xTarget value from MATLAB
    float xTarget_mm = xTarget_HB * 256.0 + xTarget_LB; // computes the xTarget in mm from the high and low byte components of xTarget
    float xTarget_m = xTarget_mm / 1000.0;  // computes xTarget in meters from xTarget in mm
    driveTo[k] = encoderPos; // writes the value for encoder position into the global driveTo vector
    xTarget[k] = xTarget_m;  // wirtes the value for target distance into the global xTarget vector

    Serial.print("For target ");
    Serial.print(k);
    Serial.print(" drive to ");
    Serial.print(driveTo[k]);
    Serial.print(" aim for ");
    Serial.print(xTarget[k], 3);
    Serial.println(" m.");
  }
  // *** Move Hardware to Desired Initial Positions - PART 2 ***
  leftSwitchReading = digitalRead(leftSwitchPin);
  if (leftSwitchReading == 1) { // if Earl is home, resets counts and prints a message
    counts = 5;
    Serial.println("Earl is home, counts reset to 5");
  }
  else {                        // if Earl is not home, sends Earl home and prints a message
    Serial.println("Earl will be sent home");
    MoveLauncher(homePosition);
  }
  // *** Take Initial Readings ***
  stripeBlack = GetEncoderBoolean();        // initialize stripeBlack
  stripeTime = millis();                    // initialize stripeTime

  // *** Declare and Initialize Variables Needed for Targeting Computations ***

  float d[] = {0.041, 0.190, 0.067};       // initializing linkage parameters
  float  kappa = 3.4738;                   // intial velocity
  float lambda = -0.0003003;
  // float  H[] = {0.1311, 0.0960, 0.088, 0.0464};      // intializing variables
  float H[] = {0.1311, .128, .0960, .0304};
  float alpha = 23.3516;                             // intilaizing best fit params
  float beta = 0.1157;
  float thetaLO = 13.4988;

  // *** Start the Competition Timer ***
  Serial.println("Time is starting");
  digitalWrite(IRLEDPin, HIGH);   // turns on IR LED to start the timer
  delay(1000);
  digitalWrite(IRLEDPin, LOW);    // turns off the IR LED after 1 second

  // *** Perform Targeting Computations ***
  TargetServoAngles(d, kappa, lambda, H, alpha, beta, thetaLO, xTarget); // computes the servo angle required to hit a specified target
  int i;
  ReorderTargets();
  FinalOrder();
  for (i = 0; i < 6; ++i) {
    Serial.print("To hit a target at ");    // sends computed values back over to MATLAB
    //   Serial.print(xTarget[i]);
    Serial.print(driveTo[i]);
    Serial.print(" m, command the servo to ");
    Serial.print(writeToServo[i]);
    Serial.println(" deg");
  }
}// end setup() function

/*******************
 ** Loop Function **
 *******************/
void loop(void) {
  //PUT YOUR MAIN CODE HERE, TO RUN REPEATEDLY

  if (target < 5) {           // shoots ands reloads the first 5 targets in driveTo Vector
    MoveLauncher(driveTo[target]);              // moves launcher to position in driveTo vector
    launcherServo.write(writeToServo[target]);  // move servo angle to value in writeToServo
    delay(1000);
    
    FireSolenoid();          // fires the solenoid
    Serial.print("Earl has fired at target # ");
    Serial.println(target + 1);
    ReloadEarl();            // reloads Earl
    target = target + 1;     // moves on the the next target value in the driveTo vector
  }
  else {
    MoveLauncher(driveTo[target]);              // moves Earl to final target position
    launcherServo.write(writeToServo[target]);  // move servo angle to value in writeToServo
    delay(1000);
    FireSolenoid();               // fires the solenoid
    Serial.print("Earl has fired at target # ");
    Serial.println(target + 1);
    MoveLauncher(homePosition);   // moves the launcher to the home position
    // leftSwitchReading = digitalRead(leftSwitchPin);
    // if (leftSwitchReading == 1) {
    Serial.println("Earl is home");
    digitalWrite(IRLEDPin, HIGH); // turns on the IR LED to stop the timer
    delay(1000);
    digitalWrite(IRLEDPin, LOW);  // turns off the IR LED after 1 second
    Serial.println("The timer has been stopped");
    //  }

    Serial.println("Earl is the greatest robot");   // just stating the obvious
    Serial.println(""); // empty message to break out of MATLAB loop
    while (true) {      // creates an infinite loop to block all other code
    }
  }


} // end loop() function

/****************************
 ** User-Defined Functions **
 ****************************/
// create custom headings as necessary to clearly organize your sketch
// e.g., Button functions, DC Motor functions, Servo functions, etc.

boolean GetEncoderBoolean(void) { // assigns a 1 if the IR sensor is over a BLACK stripe
  // assigns a 0 if the IR sensor is over a WHITE stripe
  irSensorReading = analogRead(irSensorPin);
  if ( irSensorReading < lowThreshold && black == 1) {
    return 0;     // returns 0 if sensor reading is under low threshold
  }
  else if (irSensorReading > highThreshold && black == 0) {
    return 1;     // returns 1 if sensor reading is above high threshold
  }
  else {
    return black;   // retuns previous value of black (either a 1 or a 0)
  }
}

void TurnMotorOn(void) {
  digitalWrite(motorDirectionPin, motorLeft);   // sets motor direction to variable motorLeft
  analogWrite(motorPowerPin, motorPower);       // turns motor on
  motorOn = 1;
}
void BrakeMotor( int reverseTime) {
  analogWrite(motorPowerPin, 0);            // turns motor off
  motorOn = 0;
  delay(10);
  int reverseDirection = !motorLeft ;                 // reverse motor direction
  digitalWrite(motorDirectionPin, reverseDirection);
  analogWrite(motorPowerPin, motorPower);             // turns motor on
  motorOn = 1;
  delay(reverseTime);
  analogWrite(motorPowerPin, 0);            // turns motor off
  motorOn = 0;
}
void CountStripes(void) {   // count stripes
  black = GetEncoderBoolean();
  currentTime = millis();
  unsigned long elapsedTime = currentTime - stripeTime;
  if (black != stripeBlack) {
    if (motorLeft) {
      counts = counts - 1;  // decrease counts by 1
    }
    else {
      counts = counts + 1;  // increases counts by 1
    }
    stripeBlack = black;
    stripeTime = currentTime;
    if (outputToggle) {
      Serial.print("The value of counts is ");
      Serial.print(counts);
      Serial.print(" The elapsed time is ");
      Serial.println(elapsedTime);
    }
  }
}

void MoveLauncher(int desPos) {   // moves Earl to input desPos
  if (desPos == counts) {
    Serial.print("Earls is already at ");
    Serial.println(desPos);
    return;
  }
  else if (desPos > counts) {
    motorLeft = 0;   // sets direction to the right if desPos is to the right of current value of counts
  }
  else {
    motorLeft = 1;   // sets direction to the left if desPos is to the left of current value of counts
  }

  TurnMotorOn();
  while (desPos != counts) {
    leftSwitchReading = digitalRead(leftSwitchPin);     // checks if the left switch is tripped
    rightSwitchReading = digitalRead(rightSwitchPin);   // checks if the right switch is tripped

    if (!motorLeft && rightSwitchReading == 1) {   // stops motor if right switch is pushed AND Earl is moving right
      break;
    }
    if (motorLeft && leftSwitchReading == 1) {     // stops motor if left switch is pushed AND Earl is moving left
      break;
    }
    CountStripes();     // finds and prints the count # that Earl stopped on
  }

  BrakeMotor(reverseTime);
  if (leftSwitchReading == 1) {     // if Earl is home, sets counts to 5 and prints a message
    counts = 5;
    Serial.println("Earl is home, counts reset to 5");
  }
  if (rightSwitchReading == 1) {    // if Earl is at the reloader, sets counts to 43 and prints a message
    counts = 43;
    Serial.println("Earl is at reloader, counts reset to 43");
  }
}

void FireSolenoid(void) {   // fires solenoid
  digitalWrite(solDirecPin, HIGH);
  analogWrite(solPowerPin, solPower);       // fires solenoid
  delay(solActivTime);                      // keeps solenoid on
  analogWrite(solPowerPin, 0);              // turns solenoid off
}

void ReloadEarl(void) {    // reloads the Earl
  launcherServo.write(0);  // lowers Earl to fit under the reloader
  MoveLauncher(reloaderPosition); // moves Earl to the reloader
  delay(100);                      // gives Earl some time to settle at the reloader
  reloaderServo.write(reloaderDispenseAngle); // moves reloader to dispense a ball
  Serial.println("Earl has been reloaded");
  delay(250);
  reloaderServo.write(reloaderHoldAngle);     // resets reloader servo to the hold angle
}

float Deg2Rad(float angleDeg) { // computes the angle in radians given the angle in degrees

  float angleRad = 0;  // the angle in radians
  angleRad = angleDeg * (PI / 180.0);    // computes the angle in radians given the angle in degrees
  return angleRad;
}

//  Rad2Deg funtion: to convert angles from radians to degrees

float Rad2Deg(float angleRad) {
  float angleDeg = angleRad * (180.0 / PI);   // computes the angle in degrees given the angle in radians
  return angleDeg;
}

float Quadratic(float a, float b, float c, int plusOrMinus) {  // solves quadratic formula for a polynomial
  float root = (-b + (plusOrMinus) * sqrt(b * b - 4 * a * c)) / (2 * a); // w/ coefficients a,b,c and variable plusOrMinus
  return root;
}

float LandingDistance(float d[], float kappa, float lambda, float thetaL) {
  float xLand = 0;
  float g = 9.81;
  float thetaLRadians = thetaL * PI / 180.0;                              // convert thetaL from degrees into radians
  float x0 = d[1] * cos(thetaLRadians) - (d[2] * sin(thetaLRadians));     // compute the intial x coordinate
  float y0 = d[0] + d[1] * sin(thetaLRadians) + (d[2] * cos(thetaLRadians)); // compute the intial y coordinate
  float v0 = kappa + lambda * thetaL;
  float v0x = v0 * cos(thetaLRadians);                                    // compute intial x component of velocity
  float v0y = v0 * sin(thetaLRadians);                                      // compute intial y component of velocity
  float tLand = Quadratic(-0.5 * g, v0y, y0, -1); // find landing time of projectile
  xLand = x0 + v0x * tLand;                  // landing distance of projectile
  return xLand;
}


float RangeAngle(float d[], float kappa, float lambda) {                   // Finds the launch angle that gives a maximum range
  float thetaL = 35;
  float rangeAngle = thetaL;
  float xLand = LandingDistance(d, kappa, lambda, thetaL);
  float range = xLand;
  while (thetaL <= 85) {
    thetaL = thetaL + 0.05;                             // increases thetaL y .05 degrees each pass through the loop
    xLand = LandingDistance(d, kappa, lambda, thetaL);             // computes an updated value for xland

    if (xLand > range) {                              // holds on the the maximum landing distances and returns the corresponding launch angle
      range = xLand;
      rangeAngle = thetaL;
    }
  }
  return  rangeAngle;
}

// ThetaServo function //
float ThetaServo(float H[], float thetaL, float alpha, float beta, float thetaLO) {
  float thetaLRad = Deg2Rad(thetaL);
  float alphaRad = Deg2Rad(alpha);
  float thetaLORad = Deg2Rad(thetaLO);
  float K[3];
  K[0] = H[0] / H[1]; // compute K values
  K[1] = H[0] / H[3];
  K[2] = (pow(H[0], 2) + pow(H[1], 2) - pow(H[2], 2) + pow(H[3], 2) ) / (2 * H[1] * H[3]);
  float theta2 = thetaLRad - thetaLORad;  // compute theta2
  float a = cos(theta2) - K[0] - K[1] * cos(theta2) + K[2]; // compute a, b, and  ccoefficients
  float b = -2 * sin(theta2);
  float c = K[0] - (K[1] + 1) * cos(theta2) + K[2];
  float theta4 = 2 * atan(Quadratic(a, b, c, -1));    // compute theta4
  float thetaSRad = (theta4 + alphaRad) / (1 - beta); // compute thetaS
  float thetaS = Rad2Deg(thetaSRad);                   // convert and return thetaS in degrees
  return thetaS;
}



// create a function LaunchAngle to compute the "steep" launch angle to hit a target at ta specified distance

float LaunchAngle(float d[], float kappa, float lambda, float xTarget) {
  float angleLaunch = RangeAngle(d, kappa, lambda);
  float xLand = LandingDistance(d, kappa, lambda, angleLaunch);
  while (xLand > xTarget) {
    angleLaunch = angleLaunch + 0.05;         // Increases launch angle by 0.05 degrees
    xLand = LandingDistance(d, kappa, lambda, angleLaunch);  // computes an updated value of xLand
  }
  return angleLaunch;
}

// Target Servo Angles function
void TargetServoAngles(float d[], float kappa, float lambda, float H[], float alpha, float beta, float thetaLO, float xTargetVec[]) {
  float localThetaL[6];
  float localThetaS[6];
  int k;

    Serial.print("Computing target 0");
    localThetaL[0] = LaunchAngle(d, kappa, lambda, xTargetVec[0]);
    localThetaS[0] = ThetaServo(H, localThetaL[0], alpha, beta, thetaLO);
    writeToServo[0] = round(localThetaS[0]);
    
        Serial.print("Computing target 1");
    localThetaL[1] = LaunchAngle(d, kappa, lambda, xTargetVec[1]);
    localThetaS[1] = ThetaServo(H, localThetaL[1], alpha, beta, thetaLO);
    writeToServo[1] = round(localThetaS[1]);
    
        Serial.print("Computing target 2");
    localThetaL[2] = LaunchAngle(d, kappa, lambda, xTargetVec[2]);
    localThetaS[2] = ThetaServo(H, localThetaL[2], alpha, beta, thetaLO);
    writeToServo[2] = round(localThetaS[2]);
    
        Serial.print("Computing target 3");
    localThetaL[3] = LaunchAngle(d, kappa, lambda, xTargetVec[3]);
    localThetaS[3] = ThetaServo(H, localThetaL[3], alpha, beta, thetaLO);
    writeToServo[3] = round(localThetaS[3]);
    
        Serial.print("Computing target 4");
    localThetaL[4] = LaunchAngle(d, kappa, lambda, xTargetVec[4]);
    localThetaS[4] = ThetaServo(H, localThetaL[4], alpha, beta, thetaLO);
    writeToServo[4] = round(localThetaS[4]);
    
        Serial.print("Computing target 5");
    localThetaL[5] = LaunchAngle(d, kappa, lambda, xTargetVec[5]);
    localThetaS[5] = ThetaServo(H, localThetaL[5], alpha, beta, thetaLO);
    writeToServo[5] = round(localThetaS[5]);
}

void ReorderTargets(void) {
for (int i = 0; i < 6; ++i) {
    byte temp;
    byte temp2;
    byte temp3;

    for (int k = 0; k < 5; ++k) {
      if (driveTo[k] > driveTo[k + 1]) {
        temp = driveTo[k];
        driveTo[k] = driveTo[k + 1];
        driveTo[k + 1] = temp;

        temp2 = writeToServo[k];
        writeToServo[k] = writeToServo[k + 1];
        writeToServo[k + 1] = temp2;

        temp3 = xTarget[k];
        xTarget[k] = xTarget[k + 1];
        xTarget[k + 1] = temp3;

      }
    }
  }
}



void FinalOrder(void) {
  byte encoder0 = driveTo[0];
  byte encoder1 = driveTo[1];
  byte encoder2 = driveTo[2];
  byte encoder3 = driveTo[3];
  byte encoder4 = driveTo[4];
  byte encoder5 = driveTo[5];

  driveTo[0] = encoder1;
  driveTo[1] = encoder2;
  driveTo[2] = encoder3;
  driveTo[3] = encoder4;
  driveTo[4] = encoder5;
  driveTo[5] = encoder0;

  int servo0 = writeToServo[0];
  int servo1 = writeToServo[1];
  int servo2 = writeToServo[2];
  int servo3 = writeToServo[3];
  int servo4 = writeToServo[4];
  int servo5 = writeToServo[5];

  writeToServo[0] = servo1;
  writeToServo[1] = servo2;
  writeToServo[2] = servo3;
  writeToServo[3] = servo4;
  writeToServo[4] = servo5;
  writeToServo[5] = servo0;

  float target0 = xTarget[0];
  float target1 = xTarget[1];
  float target2 = xTarget[2];
  float target3 = xTarget[3];
  float target4 = xTarget[4];
  float target5 = xTarget[5];

  xTarget[0] = target1;
  xTarget[1] = target2;
  xTarget[2] = target3;
  xTarget[3] = target4;
  xTarget[4] = target5;
  xTarget[5] = target0;
}
