#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 3); // RX, TX

void setup()  
{
  // Open serial communications with computer and wait for port to open:
  Serial.begin(9600);

  // Print a message to the computer through the USB
  Serial.println("Hello Computer!");

  // Open serial communications with the other Arduino board
  mySerial.begin(9600);
}

void loop() // run over and over
{
  if(Serial.available()){
    mySerial.write(Serial.read());
    Serial.read();
    delay(50);
    Serial.println("Message Sent");
  }
}
