#include <Servo.h>
#define SERVO_PIN 11
#define MOSFET_PIN 13
#define MYO_PIN A2

#define OPEN 0
#define CLOSE 180
#define PULSEWIDTH 50 //in ms
#define TRAVEL_DELAY 650 //in ms
#define SYSTEM_DELAY 1000 //in ms
#define DEFAULT_THRESH 100 //(0,1024)
// Multiplier for threshold so that the threshold is attainable. e.g. threshold = min(flex_max * 0.8,  moving_max)
#define THRESH_MULTIPLIER 0.65
#define CALIBRATION_TIME 5000 //in ms

Servo servo;
int pos = OPEN;
unsigned threshold = DEFAULT_THRESH;

void setup() {
  //Setup MOSFET, servo motor, and MyoWare sensor
  servo.attach(SERVO_PIN);
  pinMode(MOSFET_PIN, OUTPUT);
  pinMode(MYO_PIN, INPUT);

  //Set Initial State
  digitalWrite(MOSFET_PIN,HIGH);
  servo.write(OPEN);
  delay(TRAVEL_DELAY);
  digitalWrite(MOSFET_PIN,LOW);
  
  Serial.begin(9600);
}

void loop() {
  //Wait for Muscle Signal
  if ( analogRead(MYO_PIN) < threshold ) {
//    Serial.println(analogRead(MYO_PIN));
    return;
  }

  static unsigned long calibration_end_time = millis() + CALIBRATION_TIME;
  unsigned long action_end_time = millis() + PULSEWIDTH;
  unsigned flex_signal = analogRead(MYO_PIN);
//  Serial.println(flex_signal);
  unsigned flex_max = threshold;

  //Wait for PULSEWIDTH time to see if signal was due to noise
  while (flex_signal > threshold && millis() < action_end_time) {
    // If in calibration phase and the signal is significantly greater than the threshold, update it
    if (millis() < calibration_end_time && flex_signal * THRESH_MULTIPLIER > threshold) {
      flex_max = flex_signal * THRESH_MULTIPLIER;
    }
    flex_signal = analogRead(MYO_PIN);
//    Serial.println(flex_signal);
  }

  threshold = flex_max;
  
  // delay(PULSEWIDTH);
  
  //Toggle Servo Logic
  if (analogRead(MYO_PIN) > threshold){
//  Serial.println(analogRead(MYO_PIN));
    //OPEN to CLOSE
    if (pos == OPEN){
      digitalWrite(MOSFET_PIN,HIGH);
      pos = CLOSE;
      servo.write(CLOSE);
      delay(TRAVEL_DELAY);
    }
    //CLOSE to OPEN
    else if (pos == CLOSE){
      pos = OPEN;
      servo.write(OPEN);
      delay(TRAVEL_DELAY);
      digitalWrite(MOSFET_PIN,LOW);
    }
  
    //Wait for arm to relax again
    unsigned long relax_end_time = millis() + PULSEWIDTH;
    while ( analogRead(MYO_PIN) > threshold && millis() < relax_end_time) {
//      Serial.println(analogRead(MYO_PIN));
    }
    
    // Prevent accidental secondary trigger
    delay(PULSEWIDTH);
  }
}
