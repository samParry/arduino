/* Active color sensor. Based on INL-5APT30 photo sensor, and a common anode Red Green Blue LED.


*/

int analogPin = A0;         // orange cable on the prebuilt color sensor
const int bluePin = 5;      // blue cable on the prebuilt color sensor
const int redPin = 7;       // yellow cable on the prebuilt color sensor
const int greenPin = 6;     // green cable on the prebuilt color sensor

int vals[3];                // array to store three color reading
int delaytime = 1;          // delay between readings to allow phototransistor to come to steady state.
                            // Keep this delay as low as possible while still allowing your sensor to work properly

char serialByte;

void setup() {
  // Start serial
  Serial.begin(250000);
  while (!Serial) {
    ; // wait for a serial port connection before continuing.
  }

  pinMode(bluePin, OUTPUT); digitalWrite(bluePin, HIGH);
  pinMode(redPin, OUTPUT); digitalWrite(redPin, HIGH);
  pinMode(greenPin, OUTPUT); digitalWrite(greenPin, HIGH);
}

void loop() {

  // if we get a valid byte, read analog ins:
  if (Serial.available() > 0) {
    // get incoming byte:
    serialByte = Serial.read();
  }

  if (serialByte == 'g') {
    Serial.println("");
    Serial.println("Blue, Red, Green");
    serialByte = '0';
    for (int i = 0; i <= 29; i++) {

      // set LED color to blue
      digitalWrite(bluePin, LOW);
      digitalWrite(redPin, HIGH);
      digitalWrite(greenPin, HIGH);

      delay(delaytime);// wait for photo tranistor to come to steady state

      vals[0] = analogRead(analogPin); // read phototransistor and store value
      Serial.print(vals[0]); Serial.print(", "); // print stored value.

      // set LED color to Red
      digitalWrite(bluePin, HIGH);
      digitalWrite(redPin, LOW);
      digitalWrite(greenPin, HIGH);
      
      delay(delaytime);// wait for photo tranistor to come to steady state

      vals[1] = analogRead(analogPin); // read phototransistor and store value
      Serial.print(vals[1]); Serial.print(", "); // print stored value.

      // set LED color to green
      digitalWrite(bluePin, HIGH);
      digitalWrite(redPin, HIGH);
      digitalWrite(greenPin, LOW);
      
      delay(delaytime);// wait for photo tranistor to come to steady state

      vals[2] = analogRead(analogPin); // read phototransistor and store value
      Serial.println(vals[2]); // print stored value.

    }
    // turn off LED and wait for another cycle
    digitalWrite(bluePin, HIGH);
    digitalWrite(redPin, HIGH);
    digitalWrite(greenPin, HIGH);
  }
}
