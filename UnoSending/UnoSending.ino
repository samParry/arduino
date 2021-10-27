#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 3); // RX, TX
String message;
String from_mega;
char color;

const byte pin_color = A0;
const int pin_red = 10;
const int pin_green = 9;
const int pin_blue = 8;

void setup() {

  // *Configure pins*
  pinMode(pin_red, OUTPUT);
  pinMode(pin_green, OUTPUT);
  pinMode(pin_blue, OUTPUT);
  digitalWrite(pin_red, HIGH);
  digitalWrite(pin_green, HIGH);
  digitalWrite(pin_blue, HIGH);

  // Open serial communications with computer and wait for port to open:
  Serial.begin(9600);

  // Print a message to the computer through the USB
  Serial.println("Hello Computer!");

  // Open serial communications with the other Arduino board
  mySerial.begin(9600);
}

void loop() {
  if(Serial.available()){
    message = Serial.readStringUntil('\n');
    // mySerial.println(message);
    // determine if message is a color or state
    if (message.substring(0,5) == "color") {
      send_color();
    }
    else {
      Serial.println(message);
      mySerial.println(message);
    }
    // clear buffer and wait to catch up
    Serial.read();  // clear new line from buffer
    delay(10); 
  }

  // print any messages sent from Mega
  if (mySerial.available()) {
    from_mega = mySerial.readStringUntil('\n');  // receive string of data from MEGA
    Serial.println(from_mega);
  }
}

/****************************
 ** User-Defined Functions **
 ****************************/

void send_color() {
  String color;
  int num_reads = 10;
  int red=0, green=0, blue=0;

  // collect color readings
  for (int i = 0; i < num_reads; i++) {

    // read red
    digitalWrite(pin_red, LOW);
    digitalWrite(pin_green, HIGH);
    digitalWrite(pin_blue, HIGH);
    delay(5);
    red += analogRead(pin_color);

    // read green
    digitalWrite(pin_red, HIGH);
    digitalWrite(pin_green, LOW);
    digitalWrite(pin_blue, HIGH);
    delay(5);
    green += analogRead(pin_color);

    // read blue
    digitalWrite(pin_red, HIGH);
    digitalWrite(pin_green, HIGH);
    digitalWrite(pin_blue, LOW);
    delay(5);
    blue += analogRead(pin_color);

    // reset
    digitalWrite(pin_red, HIGH);
    digitalWrite(pin_green, HIGH);
    digitalWrite(pin_blue, HIGH);
  }

  // average the readings
  red = red/num_reads;
  green = green/num_reads;
  blue = blue/num_reads - 300;

  Serial.println(red);
  Serial.println(green);
  Serial.println(blue);

  // determine color
  if (red >= green && red >= blue) {
    color = "red";
  }
  else if (green >= red && green >= blue) {
    color = "green";
  }
  else if (blue >= red && blue >= green) {
    color = "blue";
  }
  
  // print to uno
  Serial.print("color: ");
  Serial.println(color);

  // print to mega
  mySerial.print("color: ");
  mySerial.println(color);
}
