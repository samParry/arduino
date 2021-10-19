#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 3); // RX, TX
String message;
String message_from_mega;

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
    message = Serial.readStringUntil('\n');
    mySerial.println(message);
    Serial.read();  // clear new line from buffer
    delay(10);
    Serial.println("Message Sent");
  }
}
