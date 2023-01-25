#include <Servo.h>

// **ONE MOTOR CODE - LAST UPDATED 9/4/2022** //

// **Pin Definitions** //
#define SERVO_PIN 9
#define MOSFET_PIN 8
#define MYO_PIN A0 // Pin for the myoware sensor
// Pins for the LED components
#define LED_PIN_R 2 // Red
#define LED_PIN_G 3 // Green
#define LED_PIN_B 4 // Blue
#define LED_IN A1 // Battery voltage signal - LED input

// **Servo Positions** //
#define OPEN 40
#define CLOSE 140

// **Time Delays** //
#define PULSEWIDTH 50 //in ms
#define TRAVEL_DELAY 650 //in ms
#define SYSTEM_DELAY 1000 //in ms

#define DEFAULT_THRESH 300 //(0,1024)

// Parameters to fine tune
// Number of reads to take in order to smooth out noise; it is recommended to take a power of 2 for computational efficiency
#define SMOOTH_READS 8
// Multiplier for threshold so that the threshold is attainable. e.g. threshold = min(flex_max * 0.65,  moving_max)
#define THRESH_MULTIPLIER 0.75
// What the threshold for being allowed to flex is again as a proportion of the regular threshold
#define RELAX_THRESH_MULTIPLIER 0.9

// **LED Thresholds - indicates battery level; must be above each to show respective color** //
#define RED_THRESH 675
#define YELLOW_THRESH 757
#define GREEN_THRESH 859 //unnecessary but here for uniformity

// Number of samples to take for determining the maximum signal for moving one's arm around and for flexing
#define POLL_TIME 5000 // 5 seconds
// Delay between the blinking light, in milliseconds
#define DELAY_BLINK 500
// Simulated delay for the servo motors to activate
#define DELAY_SERVO 250

Servo servo;
int pos = OPEN;
unsigned threshold;
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
    //Serial.print("Blink");
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

// **Smoothing out noise from raw myoware signal - output is smoothed signal value** //
unsigned smoothRead () {
  unsigned sum = 0;
  for (int i = 0; i < SMOOTH_READS; i++) {
    sum += analogRead(MYO_PIN);
  }
  return (sum / SMOOTH_READS);
}

// **Initiate calibration sequence to calculate flexing threshold** //
void calibrateSimple() {

  startupBlink();
  // After blue flashes 3 time, hold blue while calibrating
  writeColors(LOW, LOW, HIGH);
  
  // Record signals for POLL_TIME in ms
  unsigned long end_time = millis() + POLL_TIME;

  // Maximum flex signal
  unsigned flex_max = 0; // SHOULD WE CHANGE THIS???
  // Get the largest peak value from flexing for the duration of POLL_TIME
  while (millis() < end_time) {
    unsigned flex_signal = analogRead(MYO_PIN);
    if (flex_signal > flex_max) {
      flex_max = flex_signal;
    }
  }

  if(flex_max < DEFAULT_THRESH){
    writeColors(LOW, HIGH, HIGH); //flash yellow to signal that calibration didn't work
    flex_max = DEFAULT_THRESH;
  } //end of calibration
  else{
    //flash green to signal successful calibration
    writeColors(LOW, HIGH, LOW);
  }
  
  // Use 75% of peak signal amplitude as threshold for state change //
  threshold = flex_max * THRESH_MULTIPLIER;
  //Serial.println("Threshold: " + uint8_t(threshold));
}

// **Changing servo motor states between open and close** //
void toggleMotor() {
  if (pos == OPEN){
    digitalWrite(MOSFET_PIN,HIGH);
    pos = CLOSE;
    servo.write(CLOSE);
    delay(TRAVEL_DELAY);
  }
  //CLOSE to OPEN
  else {  //(pos == CLOSE)
    pos = OPEN;
    servo.write(OPEN);
    delay(TRAVEL_DELAY);
    digitalWrite(MOSFET_PIN,LOW);
  }
}

// **Setting up inputs and outputs and starting calibration sequence to determine threshold value** //
void setup() {
  Serial.begin(9600);

  // Set up servo
  servo.attach(SERVO_PIN);

  // Set MOSFET and LED as outputs of the Arduino
  pinMode(MOSFET_PIN, OUTPUT);
  pinMode(MYO_PIN, INPUT); // pin reading in myoware sensor signal
  pinMode(LED_PIN_R, OUTPUT);
  pinMode(LED_PIN_G, OUTPUT);
  pinMode(LED_PIN_B, OUTPUT);
  pinMode(LED_IN, INPUT); // analog pin reading in battery voltage
  
  volt_reg = analogRead(LED_IN);
  //Serial.println(volt_reg);

  // Set initial state of MOSFET
  digitalWrite(MOSFET_PIN, HIGH);
  
  // Open servo
  servo.write(OPEN);
  delay(TRAVEL_DELAY);
  
  // Set up MOSFET
  digitalWrite(MOSFET_PIN,LOW);

  // Starting calibration sequence
  digitalWrite(LED_PIN_R, LOW);
  digitalWrite(LED_PIN_G, LOW);
  digitalWrite(LED_PIN_B, LOW);
  calibrateSimple();
  
  //Serial.println("Past setup");
  //writeColors(HIGH, HIGH, LOW);
}

// **Main loop** //
void loop() {
  // Battery signal - LOOK HERE!!!
  volt_reg = analogRead(LED_IN);
  
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
  
  //writeColors(HIGH, HIGH, LOW);
  /*while (smoothRead() < threshold) {
    Serial.println("waiting... ");
  }*/
  //Serial.println(analogRead(MYO_PIN));
  
  // Toggle Servo Logic - when myoware signal is greater than threshold, change servo state (either open to close or close to open, depending on current state)
  if (analogRead(MYO_PIN) > threshold){
    // Flash purple to indicate flex/above threshold
    writeColors(HIGH,LOW,HIGH);
//  Serial.println(analogRead(MYO_PIN));
    // OPEN TO CLOSE
    if (pos == OPEN){
      digitalWrite(MOSFET_PIN,HIGH);
      pos = CLOSE;
      servo.write(CLOSE);
      delay(TRAVEL_DELAY);
    }
    // CLOSE to OPEN
    else if (pos == CLOSE){
      pos = OPEN;
      servo.write(OPEN);
      delay(TRAVEL_DELAY);
      digitalWrite(MOSFET_PIN,LOW);
    }
    
    // Wait for arm to relax again
    while (smoothRead() > threshold * RELAX_THRESH_MULTIPLIER) { //worked with analogRead(MYO_PIN) > threshold()
      Serial.println(analogRead(MYO_PIN) + "Waiting to reach " + threshold);
    }
    
    // Prevent accidental secondary trigger
    delay(PULSEWIDTH);
  }
}
