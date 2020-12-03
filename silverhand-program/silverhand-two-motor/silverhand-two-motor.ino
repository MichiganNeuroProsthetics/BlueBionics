#include <Servo.h>
// Pins
#define TI_SERVO_PIN 5 // Pin for servo
#define MRP_SERVO_PIN 11 // Pin for servo
#define MOSFET_PIN_TI 9 // 1st MOFSET pin; thumb, index finger
#define MOSFET_PIN_MRP 10 // 2nd MOFSET pin; middle finger, ring finger, pinky
#define MYO_PIN A2 //Pin for the MyoWare sensor
#define QRE_PIN A4 // Pin for reflective sensor QRE1113GR
#define MODE_POT_PIN A7 // Potentiometer for mode control

// Settings
#define SPEED_MAX 6 // maximum speed per increment of time for the servo motors
#define SPEED_MIN 0 // minimum speed per increment of time
#define SPEED_REVERSE -3 // how quickly the motors will regress to the open state per increment of time
#define TH_SPEED_REVERSE 0 //0-100. If strength <= 0, reverse the TI motor to return to the open position.
#define TH_SPEED_ZERO 30 //0-100. If 0 < strength <= 30, hold TI motor's position
#define POSITION_MAX 180 //0-180; test
#define POSITION_MIN 0 // 0-180
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
#define SMOOTH_READS 8
// Multiplier for threshold so that the threshold is attainable. e.g. myo_thresh = min(flex_max * 0.65,  moving_max)
#define THRESH_MULTIPLIER 0.65
// What the threshold for being allowed to flex is again as a proportion of the regular threshold
#define RELAX_THRESH_MULTIPLIER 0.6

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
#define POLL_TIME 3000 // 3 seconds
// Delay between the blinking light, in milliseconds
#define DELAY_BLINK 500
// Simulated delay for the servo motors to activate
#define DELAY_SERVO 250
// Set whether we're using calibration mode or normal use, where the threshold is predefined
//static bool state = CALIBRATION;
static enum states state = CALIBRATION;
// The threshold to be set; above it, the motor will activate 
unsigned myo_thresh;

enum Mode {
  full,
  ti,
  mrp,
  grab,
  any
};

Mode mode;

/* Modes
 * Full: Go between open hand and fist or between the secondary positions; change both motor positions
 * TI: Pinching (update thumb and index finger)
 * MRP: Change middle finger, ring finger, and pinky position
 * Grab: Grab cycle - open hand -> pointing -> fist -> open hand -> ...
 * Any: Go between open hand and fist, exclusively
*/


Servo ti_servo; // controls thumb and index.
Servo mrp_servo; // controls middle, ring, pinky fingers.
int ti_pos = OPEN;// position of ti_servo.
int mrp_pos = OPEN;//position of mrp_servo.
uint16_t flex_count = 0;
int volt_reg = 0; //input to display battery level
int reflection_max = 1023;
int reflection_min = 1023;
bool myo_recently_active = false; // whether or not recently updated MRP as to not yield a double flex event

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

// either MYO_PIN or QRE_PIN
unsigned smoothAnalogRead (uint8_t sensor_pin) {
  unsigned sval = 0;
  for (uint8_t i = 0; i < SMOOTH_READS; i++) {
    sval += analogRead(sensor_pin);
  }
  return sval/SMOOTH_READS;
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
  unsigned myo_max = 0;
  // Get the largest peak value from flexing for the duration of POLL_TIME (while light is solid blue)
  while (millis() < end_time) {
    unsigned myo_signal = analogRead(MYO_PIN);
    unsigned reflection_signal = analogRead(QRE_PIN);
    if (myo_signal > myo_max) myo_max = myo_signal;
    if (reflection_signal < reflection_min) reflection_min = reflection_signal;
  }

  // Set threshold to a fraction of its maximum reading
  myo_thresh = myo_max * THRESH_MULTIPLIER;


  // Turn off LED, indicating that the user need not flex anymore; not necessary, but here for uniformity
  writeColors(LOW, LOW, LOW);
  delay(DELAY_BLINK);

  // Blink 3 times to indicate that the user must begin "curling" their arm, alternating between a 180 and 45 degree angle between their forearm and upper arm
  startupBlink();

  // Remain blue while reading
  writeColors(LOW, LOW, HIGH);

  // Record signals for POLL_TIME in ms
  end_time = millis() + POLL_TIME;

  // Serial.println("measuring for flex");

  // Get the largest peak value from flexing for the duration of POLL_TIME (while light is solid blue)
  while (millis() < end_time) {
    unsigned reflection_signal = analogRead(QRE_PIN);
    if (reflection_signal < reflection_max) reflection_max = reflection_signal;
  }
  
  // Serial.println("done measuring");
  // Serial.print("threshold:");
  // Serial.println(myo_thresh);
  // return myo_thresh;
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
  pinMode(MYO_PIN, INPUT);
  pinMode(QRE_PIN, INPUT);
  
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
    myo_thresh = DEFAULT_THRESH;
  }
}

// Invert current servo position
void toggleMRP(){
  // NOTE: It would be sleeker to use pos = pos % 360, but it takes more cycles
  // If pos is open, close it and write the servos to do such
  if (mrp_pos == OPEN){
    digitalWrite(MOSFET_PIN_MRP,HIGH);    
    mrp_pos = CLOSE;
    mrp_servo.write(CLOSE);
    delay(TRAVELDELAY);
    //DEBUG
    //Serial.println("Closed motor connected to mosfet pin at: ");
    //Serial.println(MOSFET_PIN_MRP);
  }
  // If pos is closed, open it and write the servos to do such
  else {
    mrp_pos = OPEN;
    mrp_servo.write(OPEN);
    delay(TRAVELDELAY);
    //DEBUG
    //Serial.println("Opened motor connected to mosfet pin at: ");
    //Serial.println(MOSFET_PIN_MRP);
    digitalWrite(MOSFET_PIN_MRP,LOW);
  }
}

int16_t findTIPos(int16_t sensor_value){
    /**
     * This functions converts the sensor value to a change in index finger position, within certain limits,
     * thus preventing the finger from snapping to a new position too quickly
     */
    int16_t m_speed = 0;
    
    // Convert sensor_value to percentage, based on reflection_min and reflection_max
    int strength = map(sensor_value, reflection_min, reflection_max, 100, 0);
    // If strength <= 0, speed = -3, bring the hand back open
    if(strength<=TH_SPEED_REVERSE) m_speed=SPEED_REVERSE;
    // If 0 < strength < 30, speed = 0, keep position as is
    else if(strength<TH_SPEED_ZERO) m_speed=SPEED_MIN;
    // If 30 < strength <= 100, speed = map strength to be in the domain of 0 to 6, close the hand at a proportional motor speed
    // Previously, it mapped from 40 to 100. Although this makes the mapping intuitive (m_speed = (strength - 40)/10), it would make 31-39 reverse while 1-30 are zero
    // Consider making the mapping from 30 to 90
    else m_speed=map(strength,30,100,SPEED_MIN,SPEED_MAX);
    
    // Target position is current position + motor speed
    return constrain(ti_pos + m_speed, POSITION_MIN, POSITION_MAX);
}

void updateTI(unsigned new_ti_pos) {
  if (ti_pos == POSITION_MIN) {
    // Open servo's connection in use
    digitalWrite(MOSFET_PIN_TI,HIGH);
    ti_servo.write(new_ti_pos);
    delay(TRAVELDELAY);
  } else if (new_ti_pos == POSITION_MIN) {
    ti_servo.write(new_ti_pos);
    // Wait until done writing to cut off servo connection, just in case
    delay(TRAVELDELAY);
    // Cut off servo's connection while not in use
    digitalWrite(MOSFET_PIN_TI,LOW);
  } else {
    ti_servo.write(new_ti_pos);
  }
}


//delay(SYSTEMDELAY-PULSEWIDTH-TRAVELDELAY);

// Potentiometer-based control of mode selection
void updateMode () {
  int pot_val = analogRead(MODE_POT_PIN);
  mode = (Mode) map(pot_val % POT_MAX, 0, POT_MAX, 0, 4);
}

void loop() {
  unsigned reflection_value;
  // Battery signal
  volt_reg = analogRead(LED_IN);
  
  //DEBUG
  //Serial.print("Volt reg: ");
  //Serial.println(volt_reg);
  
  // If below RED_THRESH, the battery is extremely low; if below YELLOW_THRESH, should change soon; if below GREEN_THRESH, you're fine
  // If recently flexed, the LED will shine purple
  if (volt_reg < RED_THRESH) {
    uint8_t cycles_below_min = 0;
    for(uint8_t i = 0; i < 8; i++) {
      cycles_below_min += (analogRead(LED_IN) < RED_THRESH);
    }
    // If not just noise and actually have low battery, turn off access to servos
    if (cycles_below_min == 7) {
      // Turn off access to motors
      digitalWrite(MOSFET_PIN_TI,LOW);
      digitalWrite(MOSFET_PIN_MRP,LOW);
      while (1) {
        delay(DELAY_BLINK);
        writeColors(HIGH, LOW, LOW);
        delay(DELAY_BLINK);
        writeColors(LOW, LOW, LOW);
      }
    }
  }
  else if (volt_reg < YELLOW_THRESH) {
    writeColors(HIGH, HIGH, LOW);
  }
  else if (myo_recently_active) {
    if (smoothAnalogRead(MYO_PIN) > myo_thresh * RELAX_THRESH_MULTIPLIER) {
      writeColors(HIGH, LOW, HIGH);
    } else {
      writeColors(LOW, HIGH, LOW);
      myo_recently_active = false;
    }
  }
  else { //(volt_reg <= GREEN_THRESH)
    writeColors(LOW, HIGH, LOW);
  }
  
  
  reflection_value = smoothAnalogRead(QRE_PIN);
  reflection_value = constrain(reflection_value, reflection_min, reflection_max);

  unsigned new_ti_pos = findTIPos(reflection_value);
  // Only write to the motor if new value
  if (new_ti_pos != ti_pos) {
    updateTI(new_ti_pos);
    ti_pos = new_ti_pos;
  }

  // To avoid activating twice on the same signal, make sure it hasn't recently been active before registering another flex
  if (!myo_recently_active && smoothAnalogRead(MYO_PIN) > myo_thresh) {
    // Make purple while in use/above threshold
    writeColors(HIGH, LOW, HIGH);
    myo_recently_active = true;
    // If the smooth read is above the threshold, toggle the middle finger, ring finger, and pinky positions
    toggleMRP();
  }

  delay(PULSEWIDTH);
}
