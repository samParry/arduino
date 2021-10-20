// UnoRelayData.ino
// Professor Stephen Mascaro
// 10/11/2021

// This sketch uses serial comms on an Arduino Uno to wirelessly relay data back and forth between MATLAB and an Arduino MEGA

#include <SoftwareSerial.h>

// declare serial channel for X-bee communication with MEGA
SoftwareSerial mySerial(2, 3); // RX, TX

String inString = "";
String fromMega;

void setup()  
{
  // Open serial communications with MATLAB and wait for port to open:
  Serial.begin(9600);  // This must match the Baud Rate in MATLAB

  // Print a message to the computer through the USB
  Serial.print("Ready to Relay Commands to Mega \n");  //this is to notify MATLAB that Arduino is ready

  // Open serial communications with the MEGA
  mySerial.begin(9600);  //Xbee (this must match the Baud Rate on your MEGA)

 
  while(Serial.available()==0);  
  inString = Serial.readStringUntil('\n');  // receive string of control parameters from MATLAB
  mySerial.print(inString);   //  relay string of control parameters to MEGA

}

void loop() // run over and over
{
  if(mySerial.available()>2){
    inString = mySerial.readStringUntil('\n');  // receive string of data from MEGA
    Serial.println(inString);
  }
}
