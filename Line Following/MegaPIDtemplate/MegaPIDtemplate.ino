// MegaPIDtemplate.ino
// Prof. Stephen Mascaro
// 10/11/21

//  This sketch uses serial comms on an Arduino Mega to wirelessly receive control parameters and transmit data back to an Arduino UNO

double Kp=1,Kd=0,Ki=0; //PID gains
double base_speed=100;
double t, t0, error=0, m1c=0, m2c=0;


void setup() {
  Serial2.begin(9600); //Serial Comms with Uno via Xbee
  
  String inString = "";
  Serial2.println("Waiting for PID values");
  while(Serial2.available()==0);
  inString = Serial2.readStringUntil(' ');
  Kp = inString.toFloat();
  inString = Serial2.readStringUntil(' ');
  Kd = inString.toFloat();
  inString = Serial2.readStringUntil(' ');
  Ki = inString.toFloat();
  inString = Serial2.readStringUntil(' ');
  base_speed = inString.toFloat();
  
  t0 = micros()/1000000.;
  Serial2.print("Kp = ");
  Serial2.println(Kp);
  Serial2.print("Kd = ");
  Serial2.println(Kd);
  Serial2.print("Ki = ");
  Serial2.println(Ki);
  Serial2.print("Base Speed = ");
  Serial2.println(base_speed);
}

void loop() {
 
  t = micros()/1000000.-t0;
   // put your PID control code here....
   // read sensors....
   // compute error ....
   // implement PID control law:
   m1c = base_speed + Kp*error;   // motor 1 command
   m2c = base_speed - Kp*error;   // motor 2 command

   //transmit data back to UNO (add as many variables as you like
  if (t<3.0) {
    Serial2.print("time: ");
    Serial2.print(t,5);
    Serial2.print("\terror: ");
    Serial2.print(error,5);
    Serial2.print("\tm1c: ");
    Serial2.print(m1c,5);
    Serial2.print("\tm2c: ");
    Serial2.println(m2c,5);
    delay(50); //need to let UNO keep up or buffer will overflow

  }
}
