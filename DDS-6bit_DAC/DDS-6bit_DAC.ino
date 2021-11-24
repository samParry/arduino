/*MEEN 3230 Direct Digital Synthesis Digital to Analog Converter
 This code performs Direct Digital Synthesis and a R2R resistor network to produce an analog signal 
 
*/ 
// Signal Mode 'S' for Sine wave, 'T' for Triangle Mode, and 'Sq'
char SignalMode = 'S';


int sinTable6Bit[]={32,35,38,41,44,47,50,52,
                    54,57,58,60,61,62,63,63,
                    63,63,63,62,61,59,57,55,
                    53,51,48,45,42,39,36,33,
                    30,27,24,21,18,15,12,10, 
                    8, 6, 4, 2, 1, 0, 0, 0, 
                    0, 0, 1, 2, 3, 5, 6, 9,
                    11,13,16,19,22,25,28,31};

int triangleTable6Bit[]={32,34,36,38,40,42,44,46,
                         48,50,52,54,56,58,60,62,
                         63,62,60,58,56,54,52,50,
                         48,46,44,42,40,38,36,34,
                         32,30,28,26,24,22,20,18,
                         16,14,12,10,8,6,4,2,0,
                         2,4,6,8,10,12,14,16,
                         18,20,22,24,26,28,30};

int squareWaveTable6Bit[]={63,63,63,63,63,63,63,63,
                           63,63,63,63,63,63,63,63,
                           63,63,63,63,63,63,63,63,
                           63,63,63,63,63,63,63,63,
                           0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,0};

int potVal=0;
int scaledPotVal = 0;
byte byteScaled = scaledPotVal;
byte shiftedByte = byteScaled << 2;
int i = 0;

int rampVal = 0;
int mode = 0;

int speedMode = 2;

float freq = 0;
float freqScalar = 16000000.0/(64.*64.);
float oldFreqVal = 0;

void setup() {
  
  Serial.begin(250000);
  pinMode(12,INPUT);digitalWrite(12, LOW);
  pinMode(13,INPUT);digitalWrite(13, LOW);
  
  // Data Direction Register D (pins 0-7): all outputs, but keep pins 0 and 1 how they are (Serial connection)
  DDRD = B11111100 | DDRD;
   
  //set timer0 interrupt 
  TCCR0A = 0;// set entire TCCR0A register to 0
  TCCR0B = 0;// same for TCCR0B
  TCNT0  = 0;//initialize counter value to 0
  // set compare match register for 20Hz increments
  OCR0A = 51;// = (16*10^6) / (8*40000)-1
  // turn on CTC mode
  TCCR0A |= B010; 
  // Set CS01 bit for 64 prescaler
  TCCR0B |= B011;  
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
}

ISR(TIMER0_COMPA_vect){
  
  i=i+1;
  
  if(i>63){
      i=1;
      }
  if (SignalMode == 'S'){
  PORTD =  sinTable6Bit[i] << 2;
  }
  else if(SignalMode == 'T'){
  PORTD = triangleTable6Bit[i] << 2;
  }
  else if (SignalMode == 'B'){
  PORTD = squareWaveTable6Bit[i] << 2;
  }
}

void loop() {
      if(speedMode==0){
        scaledPotVal = map(analogRead(A1),0,1023,2,64);
      }
      else if(speedMode ==1){
        scaledPotVal = map(analogRead(A1),0,1023,2,255);
      }
      else{
        scaledPotVal = analogRead(A1);
      }
      
      OCR0A = scaledPotVal;
      freq = freqScalar/(scaledPotVal + 1.);
      
      if( freq!=oldFreqVal){
        // Serial.print(scaledPotVal); Serial.print("  "); Serial.println(freq);
        Serial.print("Freq: "); Serial.println(freq);
      }
      
      oldFreqVal = freq;
}
