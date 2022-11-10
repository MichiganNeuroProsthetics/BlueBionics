#include <Servo.h>
#define TI_SERVO_PIN 11 // Pin for servo
#define MRP_SERVO_PIN 10 // Pin for servo
#define MOSFET_PIN_TI 12 // 1st MOFSET pin; thumb, index finger
#define MOSFET_PIN_MRP 13 // 2nd MOFSET pin; middle finger, ring finger, pinky
#define MYO_PIN A2 //Pin for the sensor
#define MODE_POT_PIN A5 // Potentiometer for mode control
//Pins for the 4 LED components
#define LED_PIN_R 2 // Red
#define LED_PIN_G 3 // Green
#define LED_PIN_B 4 // Blue
#define LED_IN A1   // Battery voltage signal

#define OPEN 0 // 0 degree rotation for open
#define CLOSE 180 // 180 degree rotation for close
#define PULSEWIDTH 50 //in ms
#define TRAVELDELAY 650 //in ms
#define SYSTEMDELAY 750 // delay for servo motors; want to try 1200
#define DEFAULT_THRESH 300 // must be above 300 to change position
#define DEFAULT_RELAXTHRESH 275 // must be below 275 to be able to read another flex
#define POT_MAX 522 // the maximum value for the potentiometer

// Parameters to fine tune
// Number of reads to take in order to smooth out noise; it is recommended to take a power of 2 for computational efficiency
#define SMOOTH_READS 8
// Multiplier for threshold so that the threshold is attainable. e.g. threshold = min(flex_max * 0.65,  moving_max)
#define THRESH_MULTIPLIER 0.65
// What the threshold for being allowed to flex is again as a proportion of the regular threshold
#define RELAX_THRESH_MULTIPLIER 0.9

//The thresholds for the various LED colors - must be above each to show each respective color
#define STOP_THRESH 615
#define RED_THRESH 675
#define YELLOW_THRESH 757
#define GREEN_THRESH 859 //unnecessary but here for uniformity

// Don't worry about this. It just makes it more readable when turning on/off calibration mode
enum states {
  CALIBRATION,
  NORMAL_USE
};

// Number of samples to take for determining the maximum signal for moving one's arm around and for flexing
#define POLL_TIME 3000 // 3 seconds
// Delay between the blinking light, in milliseconds
#define DELAY_BLINK 500
// Simulated delay for the servo motors to activate
#define DELAY_SERVO 250

// Set whether we're using calibration mode or normal use, where the threshold is predefined
static states state = CALIBRATION;

enum Mode {
  full,
  ti,
  mrp,
  grab,
  any
};

/* Modes
 * Full: Go between open hand and fist or between the secondary positions; change both motor positions
 * TI: Pinching (update thumb and index finger)
 * MRP: Change middle finger, ring finger, and pinky position
 * Grab: Grab cycle - open hand -> pointing -> fist -> open hand -> ...
 * Any: Go between open hand and fist, exclusively
*/

// #define the location of the new stuff, and probably change the old stuff

Mode mode;
Servo ti_servo; // controls thumb and index.
Servo mrp_servo; // controls middle, ring, pinky fingers.
int ti_pos = OPEN;// position of ti_servo.
int mrp_pos = OPEN;//position of mrp_servo.
int volt_reg = 0; //input to display battery level
// The threshold to be set; above it, the motor will activate 
unsigned threshold;

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
  }
  
  if(flex_max < DEFAULT_THRESH){
    writeColors(LOW, HIGH, HIGH); //flash yellow to signal that calibration didn't work
    flex_max = DEFAULT_THRESH;
  } //end of calibration
  else{
    //flash green to signal successful calibration
    writeColors(LOW, HIGH, LOW);
  }
  // Set threshold to a fraction of its maximum reading
  threshold = flex_max * THRESH_MULTIPLIER;

}

// EMG Attempt: Calibrate optimal EMG threshold
// *** Define globals to set ***

// ***                       ***
void calibrateLessSimple() {
  // Implement calibration setup
}

bool userFlexing(unsigned emg_val) {
  // Implement to take emg_value and return boolean for if flexing
  return false;
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
    calibrateLessSimple();
  } else {
    threshold = DEFAULT_THRESH;
  }
}

// Invert current servo position; in future iterations, may opt for a function that instead simply dictates desired positions
void invertServoPos(int &pos, Servo &servo, int mos){
  // NOTE: It would be sleeker to use pos = (pos + 180) % 360, but it takes more cycles
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

// Change position of motors based on which mode is currently selected
void updateMotors () {
  // Mode will be updated via voice control
  switch(mode) {
    case ti: //silver - mode 2
      Serial.println("ti");
      invertServoPos(ti_pos, ti_servo, MOSFET_PIN_TI);
      break;
    case mrp: //red - mode 3
      Serial.println("mrp");
      invertServoPos(mrp_pos, mrp_servo, MOSFET_PIN_MRP);
      break;
    case full: //gold: go between open hand and closed exclusively- mode 1
      Serial.println("full");
      invertServoPos(ti_pos, ti_servo, MOSFET_PIN_TI);
      if (ti_pos != mrp_pos) {
        invertServoPos(mrp_pos, mrp_servo, MOSFET_PIN_MRP);
      } //add == version to gold/full
      break;
    case grab://3 step grab, close mrp, close ti, open both (blue - mode 4)
     //Serial.println("grab");
      if (ti_pos == OPEN && mrp_pos == OPEN){
        invertServoPos(mrp_pos, mrp_servo, MOSFET_PIN_MRP);
      }
      else if (ti_pos != mrp_pos){
        invertServoPos(ti_pos, ti_servo, MOSFET_PIN_TI);
      }
      else if (ti_pos == CLOSE && mrp_pos == CLOSE){
        invertServoPos(ti_pos, ti_servo, MOSFET_PIN_TI);
        invertServoPos(mrp_pos, mrp_servo, MOSFET_PIN_MRP);
      }
      break;
    case any: //go between opposite positions exclusively - purple (mode 5)
      //Serial.println("any");
      invertServoPos(ti_pos, ti_servo, MOSFET_PIN_TI);
      if (ti_pos == mrp_pos) {
        invertServoPos(mrp_pos, mrp_servo, MOSFET_PIN_MRP);
      } //add == version to gold/full
      break;
  }
  
  delay(SYSTEMDELAY-PULSEWIDTH-TRAVELDELAY);
}

// Potentiometer-based control of mode selection
void updateMode () {
  int pot_val = analogRead(MODE_POT_PIN);
  //Serial.println(pot_val);
  mode = (Mode) map(pot_val, 0, POT_MAX, 0, 5);
  mode = (mode > 4) ? 4 : mode;
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
  if(volt_reg >= STOP_THRESH){
    while (useFlexing(smoothRead())) {
      //DEBUG
      //Serial.println(analogRead(MYO_PIN));
      // return; // check how long writing to LEDs is
    }
    
    // Make purple while in use/above threshold
    writeColors(HIGH, LOW, HIGH);
  
    // Check if mode has changed;
    updateMode();
    
    // If the smooth read is above the threshold, it's a flex
    updateMotors();
    
    //Wait for PULSEWIDTH time
    delay(PULSEWIDTH);
  
    // Wait until below relax threshold if not already
    while (userFlexing(smoothRead())) {}
  }
}
