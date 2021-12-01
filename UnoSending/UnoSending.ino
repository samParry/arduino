
#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3); // RX, TX
String message;
String from_mega;

char color;
const byte pin_color = A0;
const int pin_red = 12;
const int pin_green = 10;
const int pin_blue = 8;

void setup() {

  // *Configure pins*
  pinMode(pin_red, OUTPUT);
  pinMode(pin_green, OUTPUT);
  pinMode(pin_blue, OUTPUT);
  color_off();

  Serial.begin(9600);
  mySerial.begin(9600);
  Serial.println("Hello Computer!");
}

void loop() {
  if(Serial.available()){
    message = Serial.readStringUntil('\n');

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
    delay(1); 
  }

  // print any messages sent from Mega
  if (mySerial.available()) {
    from_mega = mySerial.readStringUntil('\n');
    Serial.println(from_mega);
  }
}

/****************************
 ** User-Defined Functions **
 ****************************/

void color_off() {
  digitalWrite(pin_red, HIGH);
  digitalWrite(pin_green, HIGH);
  digitalWrite(pin_blue, HIGH);
}

void send_color() {
  String color;
  int num_reads = 5;
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
    color_off();
  }

  // average the readings
  red = red/num_reads;
  green = green/num_reads;
  blue = blue/num_reads;

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

  // print to mega
  mySerial.print("color: ");
  mySerial.println(color);
}
