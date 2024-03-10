// **ONE MOTOR CODE - LAST UPDATED 1/31/2023** //

#include <Servo.h>
#define SERVO_PIN1 9//11
//#define SERVO_PIN2 10
#define MOSFET_PIN1 8
//#define MOSFET_PIN2 13
#define MYO_PIN A0
#define POLL_TIME 5000 // 5 seconds
#define DELAY_BLINK 500

// stop turning after some time

// pin for voltage, one for ground, enable A, in1, in2
  // enable: velocity, in1 = speed/direction, in2 same
// put an opamp in - just an hbridge
// enable pin - send it pulses of the same magnitude and the frequency determines the power?speed? decide amplitude
// arduino has 0-255 build, 0 -1023

#define PULSEWIDTH 50 //in ms
#define TRAVEL_DELAY 650 //in ms
#define SYSTEM_DELAY 1000 //in ms

#define LED_PIN_R 2 // Red
#define LED_PIN_G 3 // Green
#define LED_PIN_B 4 // Blue
#define LED_IN A1 // Battery voltage signal - LED input

// **LED Thresholds - indicates battery level; must be above each to show respective color** //
#define RED_THRESH 675
#define YELLOW_THRESH 757
#define GREEN_THRESH 859 //unnecessary but here for uniformity

#define SMOOTH_READS 8
#define THRESH_MULTIPLIER 0.75
#define RELAX_THRESH_MULTIPLIER 0.9
#define fsrAnalogPin = A0;

int fsrReading = 0;
#define IRSensor = 11; 
#define dcmotor = 9;
int closedHand = false;
int sensorStatus = 0;
int LED = 12;

// input to hbridge - ground, power
// 0 clockwise, 1 counter clockwise
// track state to ignore IR sensor when the hand is closed
// input pins are black, blue pins are power
// enable 1/A, input 1, input 2
// input 3, input 4, enable B
// input one and input 2 are 0 and 1s
// 1, 0 is forward
// 0,1 is backward
// 0,0 or 1,1 is brake
// enable pin is speed - need to initialize all of these (enableA, pin 1, pin 2)

int pos = 140; // Put starting degree value here
int deg = 0; // Put end degree value here
unsigned threshold = 400;
int volt_reg = 0; // input to display battery level

// **Initializing colors** //
inline void writeColors(uint8_t red, uint8_t green, uint8_t blue) {
  digitalWrite(LED_PIN_R, red);
  digitalWrite(LED_PIN_G, green);
  digitalWrite(LED_PIN_B, blue);
}

// **Flashes blue 3 times to indicate start of calibration sequence** //
void startupBlink() {
  for (uint8_t i = 0; i < 3; ++i) {
    Serial.print("Blink");
    //Serial.println(i+1);
    delay(DELAY_BLINK);
    // Flash blue
    writeColors(LOW, LOW, HIGH);
    delay(DELAY_BLINK);
    // Flash off
    digitalWrite(LED_PIN_R, LOW);
    digitalWrite(LED_PIN_G, LOW);
    digitalWrite(LED_PIN_B, LOW);
  }
}


unsigned smoothRead () {
  unsigned sum = 0;
  for (int i = 0; i < SMOOTH_READS; i++) {
    sum += analogRead(fsrAnalogPin);
  }
  return (sum / SMOOTH_READS);
}

void calibrate() {
  startupBlink();
  unsigned long end_time = millis() + POLL_TIME;

  // Maximum flex signal
  unsigned flex_max = 0; 

  while (millis() < end_time) {
    unsigned flex_signal = analogRead(fsrAnalogPin);
    Serial.println(flex_signal);
    if (flex_signal > flex_max) {
      flex_max = flex_signal;
    }
    Serial.println("1st step");
  }

  if(flex_max < threshold){
    //writeColors(LOW, HIGH, HIGH); //flash yellow to signal that calibration didn't work
    flex_max = threshold;
    Serial.println("2nd step");
  } //end of calibration
  else{
    //flash green to signal successful calibration
    //writeColors(LOW, HIGH, LOW);
    Serial.println("3rd step");
  }
  
  // Use 75% of peak signal amplitude as threshold for state change //
  threshold = flex_max * THRESH_MULTIPLIER;
  Serial.println("Threshold");
  Serial.println(threshold);
}



void toggleMotor(int reading) {
  writeColors(HIGH, LOW, HIGH); //purple for flexing
  digitalWrite(MOSFET_PIN1,HIGH);

  int scaled_value = map(reading, 0, 1023, 0 , 255);
  analogWrite(enableA, scaled_value); // Send scaled value to enableA 

  if (!closedHand) { // if hand is open
    // close hand -- this rotates forward, not sure if that opens or closes
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    closedHand = true;
    delay(20); // specifies how long the motor rotates for
    digitalWrite(in2, HIGH); // if in1=in2, motor stop rotating. could also do (in1, LOW)
  }
  else { // hand is closed
    // close hand -- this rotates forward, not sure if that opens or closes
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    closedHand = false;
    delay(20);
    digitalWrite(in2, LOW); // if in1=in2, motor stop rotating. could also do (in1, HIGH)
  }

  if (deg <= pos) {
    //For 0 degrees:
    //open hand
    for (pos = 140; pos >= deg; pos -= 1) { // goes from 180 degrees to 0 degrees
      myservo1.write(pos);              // tell servo to go to position in variable ‘pos’
      //myservo2.write(pos);
      Serial.println("opening");
      delay(15);                  // waits 15ms for the servo to reach the position
    }
    pos = 0;
    deg = 140;
    
  }
  else {
    //close hand
    for (pos = 0; pos <= deg; pos++) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      myservo1.write(pos);              // tell servo to go to position in variable ‘pos’
      //myservo2.write(pos);
      Serial.println("closing");
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    pos = 140;
    deg = 0;
  }
  digitalWrite(MOSFET_PIN1,LOW);
  while (smoothRead() > threshold * RELAX_THRESH_MULTIPLIER) { //worked with analogRead(MYO_PIN) > threshold()
    Serial.println("Waiting to reach relax threshold");
  }
}

void batteryCheck() {
  volt_reg = analogRead(LED_IN);
  Serial.println("battery check");
  // If below RED_THRESH, the battery is extremely low; if below YELLOW_THRESH, should change soon; if below GREEN_THRESH, you're fine
  if (volt_reg < RED_THRESH) {
    writeColors(HIGH, LOW, LOW);
  }
  else if (volt_reg < YELLOW_THRESH) {
    writeColors(HIGH, HIGH, LOW);
  }
  else { //(volt_reg <= GREEN_THRESH)
    writeColors(LOW, HIGH, LOW);
  } // end of battery LED code  
}


//void setup() { loop and setup for IR sensors, need to swap in place of servo initialization/reading and myoware
//  Serial.begin(9600);
//    pinMode(IRSensor, INPUT); 
//    pinMode(LED, OUTPUT);
//}
//
//void loop() {
//  fsrReading = analogRead(fsrAnalogPin);
//  Serial.print("Pressure Sensor Reads: ");
//  Serial.println(fsrReading);
//  sensorStatus = digitalRead(IRSensor);
//  Serial.print("IR Sensor Reads: ");
//  Serial.println(sensorStatus);
//
//  if (sensorStatus == 0 && fsrReading > 400){
//    digitalWrite(LED, HIGH);
//  }
//  else{
//    digitalWrite(LED, LOW);
//  }
//}


void setup() {
  //Setup IO
  myservo1.attach(SERVO_PIN1);
  //myservo2.attach(SERVO_PIN2);
  pinMode(MOSFET_PIN1, OUTPUT);
  pinMode(LED_PIN_R, OUTPUT);
  pinMode(LED_PIN_G, OUTPUT);
  pinMode(LED_PIN_B, OUTPUT);
  pinMode(LED_IN, INPUT); // analog pin reading in battery voltage

  volt_reg = analogRead(LED_IN);
  //pinMode(MOSFET_PIN2, OUTPUT);
  //Set Initial State
  digitalWrite(MOSFET_PIN1,HIGH);
  pinMode(fsrAnalogPin, INPUT);
  //digitalWrite(MOSFET_PIN2,HIGH);
  Serial.begin(9600);
  delay(50);

  calibrate();
}

void loop() {
   batteryCheck();
   int reading = smoothRead();
   if(reading > threshold && IRSensor == 1){
    toggleMotor(reading);
    while (smoothRead() > threshold) {
      //waits till the signal dies down before continuining to anything
    }
   }
   toggleMotor(); //open once they stop pressing the button
   delay(PULSEWIDTH);
}
