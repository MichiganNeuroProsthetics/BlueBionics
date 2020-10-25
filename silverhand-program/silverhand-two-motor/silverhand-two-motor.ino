#include <Servo.h>
#define TI_SERVO_PIN 5 // Pin for servo
#define MRP_SERVO_PIN 11 // Pin for servo
#define MOSFET_PIN_TI 9 // 1st MOFSET pin; thumb, index finger
#define MOSFET_PIN_MRP 10 // 2nd MOFSET pin; middle finger, ring finger, pinky
#define MYO_PIN A2 //Pin for the sensor
#define MODE_POT_PIN A7 // Potentiometer for mode control
#define OPEN 0 // 0 degree rotation for open
#define CLOSE 180 // 180 degree rotation for close
#define PULSEWIDTH 50 //in ms
#define TRAVELDELAY 650 //in ms
#define SYSTEMDELAY 750 // delay for servo motors; want to try 1200
#define DEFAULT_THRESH 300 // must be above 300 to change position
#define DEFAULT_RELAXTHRESH 275 // must be below 275 to be able to read another flex
#define POT_MAX 504 // the maximum value for the potentiometer

// Parameters to fine tune
// Number of reads to take in order to smooth out noise
#define SMOOTH_READS 16
// Multiplier for threshold so that the threshold is attainable. e.g. threshold = min(flex_max * 0.65,  moving_max)
#define THRESH_MULTIPLIER 0.65

//Pins for the 4 LED components
#define LED_PIN_R 2 // Red
#define LED_PIN_G 4 // Green
#define LED_PIN_B 3 // Blue
#define LED_IN A3   // LED input

//The thresholds for the various LED colors - must be above each to show each respective color
#define RED_THRESH 675
#define YELLOW_THRESH 757
#define GREEN_THRESH 859 //unnecessary but here for uniformity

// Don't worry about this. It just makes it more readable when turning on/off calibration mode
enum states {
  CALIBRATION,
  NORMAL_USE
};

// Number of samples to take for determining the maximum signal for moving one's arm around and for flexing
#define POLL_TIME 4000 // 4 seconds
// Delay between the blinking light, in milliseconds
#define DELAY_BLINK 500
// Simulated delay for the servo motors to activate
#define DELAY_SERVO 250
// Set whether we're using calibration mode or normal use, where the threshold is predefined
//static bool state = CALIBRATION;
static enum states state = CALIBRATION;
// The threshold to be set; above it, the motor will activate 
unsigned threshold;

enum Mode {
  full,
  ti,
  mrp,
  grab,
  any
};

Mode mode;

/* Modes
 * A: Go between open hand and fist or between the secondary positions; change both motor positions
 * B: Pinching
 * C: Change mrp position
 * D: Grab cycle - open hand -> pointing -> fist -> open hand -> ...
 * E: Go between open hand and fist, exclusively
*/

// #define the location of the new stuff, and probably change the old stuff

Servo ti_servo; // controls thumb and index.
Servo mrp_servo; // controls middle, ring, pinky fingers.
int ti_pos = OPEN;// position of ti_servo.
int mrp_pos = OPEN;//position of mrp_servo.
short flex_count = 0;
int volt_reg = 0; //input to display battery level

// I just extracted this to write the colors of the LED
inline void writeColors(uint8_t red, uint8_t green, uint8_t blue) {
  digitalWrite(LED_PIN_R, red);
  digitalWrite(LED_PIN_G, green);
  digitalWrite(LED_PIN_B, blue);
}

// Blink LED 3 times, leaving it on
void startupBlink() {
  for (uint8_t i = 0; i < 3; ++i) {
    //Serial.print("Blink");
    //Serial.println(i+1);
    delay(DELAY_BLINK);
    // Flash blue
    writeColors(LOW, LOW, HIGH);
    delay(DELAY_BLINK);
    // Flash off
    writeColors(LOW, LOW, LOW);
  }
}

unsigned smoothRead () {
  unsigned sum = 0;
  for (int i = 0; i < SMOOTH_READS; i++) {
    sum += analogRead(MYO_PIN);
  }
  return (sum / SMOOTH_READS);
}

// Calibrate optimal threshold
void calibrateSimple() {
  //Serial.println("Beginning Calibration");
  
  // Blink 3 times to indicate that the user must begin flexing after the blinks
  startupBlink();

  // Remain blue while reading
  writeColors(LOW, LOW, HIGH);

  // Record signals for POLL_TIME in ms
  unsigned long end_time = millis() + POLL_TIME;

  // Serial.println("measuring for flex");

  // Maximum flex signal
  unsigned flex_max = 0;
  // Get the largest peak value from flexing for the duration of POLL_TIME
  while (millis() < end_time) {
    unsigned flex_signal = analogRead(MYO_PIN);
    if (flex_signal > flex_max) {
      flex_max = flex_signal;
    }
    // Serial.println(flex_signal);
  }

  // Set threshold to a fraction of its maximum reading
  threshold = flex_max * THRESH_MULTIPLIER;

  // Turn off LED, indicating that the user need not flex anymore; not necessary, but here for uniformity
  // writeColors(LOW, LOW, LOW);
  
  // Serial.println("done measuring");
  // Serial.print("threshold:");
  // Serial.println(threshold);
  // return threshold;
  // Serial.println("Calibratation Ended");
}

void setup() {
  //Set up servo
  ti_servo.attach(TI_SERVO_PIN);
  mrp_servo.attach(MRP_SERVO_PIN);
  
  // Set MOSFET and LED as outputs of the Arduino
  pinMode(MOSFET_PIN_TI, OUTPUT);
  pinMode(MOSFET_PIN_MRP, OUTPUT);
  pinMode(LED_PIN_R, OUTPUT);
  pinMode(LED_PIN_G, OUTPUT);
  pinMode(LED_PIN_B, OUTPUT);
  pinMode(LED_IN, INPUT);
  pinMode(MODE_POT_PIN, INPUT);
  
  //Set Initial State of MOSFETS
  digitalWrite(MOSFET_PIN_TI,HIGH);
  digitalWrite(MOSFET_PIN_MRP,HIGH);
  
  //Open up servos
  ti_servo.write(OPEN);
  mrp_servo.write(OPEN);
  delay(TRAVELDELAY);
  
  //Set up MOSFET
  digitalWrite(MOSFET_PIN_TI,LOW);
  digitalWrite(MOSFET_PIN_MRP, LOW);
  Serial.begin(9600);
  
  //Default the LED to off
  digitalWrite(LED_PIN_R, LOW);
  digitalWrite(LED_PIN_G, LOW);
  digitalWrite(LED_PIN_B, LOW);

  // If state is set here to calibration, it will undergo normal calibration; if not, it will have a static threshold
  if (state == CALIBRATION) {
    calibrateSimple();
  } else {
    threshold = DEFAULT_THRESH;
  }
}

// Invert current servo position
void servoLogic(int &pos, Servo &servo, int mos){
  // NOTE: It would be sleeker to use pos = pos % 360, but it takes more cycles
  // If pos is open, close it and write the servos to do such
  if (pos == OPEN){
    digitalWrite(mos,HIGH);    
    pos = CLOSE;
    servo.write(CLOSE);
    delay(TRAVELDELAY);
    //DEBUG
    //Serial.println("Closed motor connected to mosfet pin at: ");
    //Serial.println(mos);
  }
  // If pos is closed, open it and write the servos to do such
  else {
    pos = OPEN;
    servo.write(OPEN);
    delay(TRAVELDELAY);
    //DEBUG
    //Serial.println("Opened motor connected to mosfet pin at: ");
    //Serial.println(mos);
    digitalWrite(mos,LOW);
  }
}

void updateMotors () {
  // Mode will be updated via voice control
  
  /*if (mode == full) {
    servoLogic(ti_pos, ti_servo, MOSFET_PIN_TI); //this line is duplicated in TI
    servoLogic(mrp_pos, mrp_servo, MOSFET_PIN_MRP); // this line is duplicated in MRP
  }*/
   // if changing thumb and index finger position
  if (mode == ti || mode == full){
    servoLogic(ti_pos, ti_servo, MOSFET_PIN_TI);
  }
  // if changing middle finger, ring finger, and pinky position
  if (mode ==  mrp || mode == full) {
    servoLogic(mrp_pos, mrp_servo, MOSFET_PIN_MRP);
  }

  // TODO: Combine the 3 if statements below akin to how you will do for the 3 above
  // if grabbing
  else if (mode == grab) {
    if (ti_pos == OPEN && mrp_pos == OPEN){
      servoLogic(mrp_pos, mrp_servo, MOSFET_PIN_MRP);
    }
    else if (ti_pos != mrp_pos){
      servoLogic(ti_pos, ti_servo, MOSFET_PIN_TI);
    }
    else if (ti_pos == CLOSE && mrp_pos == CLOSE){
      servoLogic(ti_pos, ti_servo, MOSFET_PIN_TI);
      servoLogic(mrp_pos, mrp_servo, MOSFET_PIN_MRP);
    }
  }
  //if opening and closing hand, exclusively
  else if (mode == any && ti_pos == mrp_pos) {
    servoLogic(ti_pos, ti_servo, MOSFET_PIN_TI);
    servoLogic(mrp_pos, mrp_servo, MOSFET_PIN_MRP);
  } else if (mode == any) { // just in case
    // By default, just change the position of the thumb and index finger
    servoLogic(ti_pos, ti_servo, MOSFET_PIN_TI);
  }
  
  delay(SYSTEMDELAY-PULSEWIDTH-TRAVELDELAY);
}

// Potentiometer-based control of mode selection
void updateMode () {
  int pot_val = analogRead(MODE_POT_PIN);
  mode = (Mode) map(pot_val % POT_MAX, 0, POT_MAX, 0, 3);
}

void loop() {
  // Battery signal
  volt_reg = analogRead(LED_IN);
  
  //DEBUG
  //Serial.print("Volt reg: ");
  //Serial.println(volt_reg);
  
  // If below RED_THRESH, the battery is extremely low; if below YELLOW_THRESH, should change soon; if below GREEN_THRESH, you're fine
  if (volt_reg < RED_THRESH) {
    writeColors(HIGH, LOW, LOW);
  }
  else if (volt_reg < YELLOW_THRESH) {
    writeColors(HIGH, HIGH, LOW);
  }
  else { //(volt_reg <= GREEN_THRESH)
    writeColors(LOW, HIGH, LOW);
  }
  
  //Wait for muscle signal
  while (smoothRead() < threshold) {
    //DEBUG
    //Serial.println(analogRead(MYO_PIN));
    // return; // check how long writing to LEDs is
  }
  
  // Make purple while in use/above threshold
  writeColors(HIGH, LOW, HIGH);

  //Toggle Servo Logic if flexed for at least PULSEWIDTH  
  //Keep Signal to Actuation Delay around 1 second.

  // Check if mode has changed;
  updateMode();
  
  // If the smooth read is above the threshold, it's a flex
  if (smoothRead() > threshold){
    // TODO: Add any necessary parameters
    updateMotors();
  }
  
  //Wait for PULSEWIDTH time
  delay(PULSEWIDTH);

  // Wait until below threshold if not already
  while (smoothRead() > threshold) {}
}
