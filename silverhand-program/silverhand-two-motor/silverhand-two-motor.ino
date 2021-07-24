#include <Servo.h>
#include <SoftwareSerial.h>

//Pins
#define LED_PIN_R 2 //Red
#define LED_PIN_G 3 //Green
#define LED_PIN_B 4 //Blue

#define BATT_IN A1 //Battery level
#define THERMAL_PIN A7 //Prevent overheat
#define FLEX_PIN A2 //Pin for the sensor

#define TI_SERVO_PIN 5 //Pin for servo
#define MRP_SERVO_PIN 11 //Pin for servo
#define TI_MOSFET_PIN 9 //1st MOFSET pin; thumb, index finger
#define MRP_MOSFET_PIN 10 //2nd MOFSET pin; middle finger, ring finger, pinky

#define VOICE_REC_TX_PIN 7 //connects to TX of voice rec module
#define VOICE_REC_RX_PIN 6 //connects to RX of voice rec module

//Predefined values
//The thresholds for the various LED colors - must be above each to show each respective color
#define RED_THRESH 675
#define YELLOW_THRESH 757
#define GREEN_THRESH 859

#define DELAY_BLINK 500 //in ms //Delay between the blinking light, in milliseconds

#define ANALOG_MIN 0 //minimum reading of 8-bit analog pins
#define ANALOG_MAX 1023 //maximum reading of 8-bit analog pins
#define DEFAULT_THRESH 300 //must be above 300 to change position
#define DEFAULT_RELAXTHRESH 275 //must be below 275 to be able to read another flex

#define OPEN 0 //0 degree rotation for open
#define CLOSED 180 //180 degree rotation for close

//Parameters to fine tune
//Number of samples to take for determining the maximum signal for moving one's arm around and for flexing
#define POLL_TIME 3000 //in ms
//Number of reads to take in order to smooth out noise
#define SMOOTH_READS 8
//Multiplier for threshold so that the threshold is attainable. e.g. myo_thresh = min(flex_max * 0.65,  moving_max)
#define THRESH_MULTIPLIER 0.6
//What the threshold for being allowed to flex is again as a proportion of the regular threshold
#define RELAX_THRESH_MULTIPLIER 0.5
//minimum time (ms) needed for servo to move 180 degrees. Also the stated operation speed in the spec of 60deg/0.1s @ 7.4V
#define MIN_TRAVEL_TIME 300
//longest time (ms) to move 180 degrees. Can be changed but I think 2s is as slow as we need
#define MAX_TRAVEL_TIME 2000

//Don't worry about this. It just makes it more readable when turning on/off calibration mode
enum states {
  CALIBRATION,
  NORMAL_USE
};
//Set whether we're using calibration mode or normal use, where the threshold is predefined
static enum states state = CALIBRATION;

enum Mode {
  full,
  ti,
  mrp,
  grab,
  any
};

Mode mode;

/* Modes
   Full: Go between open hand and fist; change both motor positions;
   TI: Pinching (update thumb and index finger)
   MRP: Change middle finger, ring finger, and pinky position
   Grab: Grab cycle - open hand -> pointing -> fist -> open hand -> ...
   Any: *YET TO BE IMPLEMENTED*
*/

class VoiceRec
{
  private:
    SoftwareSerial voiceRecSerial;
    void clearBuffer();
    void writeToVoiceRec(byte hexKey);
  public:
    VoiceRec(uint8_t rxPin, uint8_t txPin) : voiceRecSerial{ rxPin, txPin }
    {
    }
    void begin(int baudRate = 9600);
    int available();
    byte read();
    byte waitForResponse();
    String query();
    void record(int groupNum, bool debug = false);
    void enterListenMode(int groupNum);
};

Servo ti_servo; //controls thumb and index.
Servo mrp_servo; //controls middle, ring, pinky fingers.
VoiceRec voiceRec(VOICE_REC_TX_PIN, VOICE_REC_RX_PIN); //voice-control module housed on pins 0 and 1

//things to keep track of
unsigned batt_level = 0; //input to display battery level
unsigned trigger_thresh = 0; //The trigger threshold to be set; above it, the motors will activate
unsigned relax_thresh = 0; //The relax threshold to be set; must drop below it before activating motors again
bool recently_active = false; //whether or not recently updated servos as to not yield a double flex event
unsigned ti_pos = OPEN; //position of ti_servo.
unsigned mrp_pos = OPEN; //position of mrp_servo.


//BEGIN LED UI FUNCTIONS

//I just extracted this to write the colors of the LED
inline void writeColors(uint8_t red, uint8_t green, uint8_t blue) {
  digitalWrite(LED_PIN_R, red);
  digitalWrite(LED_PIN_G, green);
  digitalWrite(LED_PIN_B, blue);
}

//Blink blue LED 3 times, leaving it on
void startupBlink() {
  for (uint8_t i = 0; i < 3; ++i) {
    //Flash blue
    writeColors(LOW, LOW, HIGH); //blue
    delay(DELAY_BLINK);
    //Flash off
    writeColors(LOW, LOW, LOW);
    delay(DELAY_BLINK);
  }
}

//END LED UI FUNCTIONS
//BEGIN BATTERY REGULATION FUNCTIONS

//Regulate entire system based on battery level
void checkBattery(unsigned batt_level) {
  //If below RED_THRESH, the battery is extremely low; if below YELLOW_THRESH, should change soon; if below GREEN_THRESH, you're fine; If above, blink green
  //If recently flexed, the LED will shine purple
  if (batt_level < RED_THRESH) {
    uint8_t cycles_below_min = 0;
    for (uint8_t i = 0; i < 8; i++) {
      cycles_below_min += (analogRead(BATT_IN) < RED_THRESH);
    }
    //If not just noise and actually have low battery, turn off access to servos
    if (cycles_below_min == 7) {
      //Turn off access to motors
      digitalWrite(TI_MOSFET_PIN, LOW);
      digitalWrite(MRP_MOSFET_PIN, LOW);
      while (true) { //Blink red
        delay(DELAY_BLINK);
        writeColors(HIGH, LOW, LOW);
        delay(DELAY_BLINK);
        writeColors(LOW, LOW, LOW);
      }
    }
  }
  else if (batt_level < YELLOW_THRESH) {
    writeColors(HIGH, HIGH, LOW);
  }
  else if (batt_level <= GREEN_THRESH) {
    writeColors(LOW, HIGH, LOW);
  }
  else {
    uint8_t cycles_above_max = 0;
    for (uint8_t i = 0; i < 8; i++) {
      cycles_above_max += (analogRead(BATT_IN) < RED_THRESH);
    }
    //If not just noise and actually supercharged, turn off access to servos
    if (cycles_above_max == 7) {
      //Turn off access to motors
      digitalWrite(TI_MOSFET_PIN, LOW);
      digitalWrite(MRP_MOSFET_PIN, LOW);
      while (true) { //Blink green
        delay(DELAY_BLINK);
        writeColors(LOW, HIGH, LOW);
        delay(DELAY_BLINK);
        writeColors(LOW, LOW, LOW);
      }
    }
  }
}

//END BATTERY REGULATION FUNCTIONS
//BEGIN SENSING/CALIBRATION FUNCTIONS

//averages SMOOTH_READS number of samples to reduce the effects of noise
unsigned smoothAnalogRead(unsigned sensor_pin) {
  unsigned sval = 0;
  for (uint8_t i = 0; i < SMOOTH_READS; ++i) {
    sval += 1023 - analogRead(sensor_pin);
  }
  return sval / SMOOTH_READS;
}

//Calibrate optimal threshold for flex
void calibrateSimple() {
  //Temporary tracker variables
  unsigned calibrate_signal;
  //Blink 3 times to indicate that the user must begin flexing after the blinks
  startupBlink();

  //Remain blue while reading
  writeColors(LOW, LOW, HIGH);

  //Record signals for POLL_TIME in ms
  unsigned long end_time = millis() + POLL_TIME;

  //Maximum flex signal
  unsigned signal_max = 0;
  //Get the largest peak value from flexing for the duration of POLL_TIME (while light is solid blue)
  while (millis() < end_time) {
    calibrate_signal = 1023 - analogRead(FLEX_PIN);
    if (calibrate_signal > signal_max) signal_max = calibrate_signal;
  }

  //Set thresholds to a fraction of its maximum reading
  trigger_thresh = signal_max * THRESH_MULTIPLIER;
  relax_thresh = signal_max * RELAX_THRESH_MULTIPLIER;
}

//END SENSING/CALIBRATION FUNCTIONS
//BEGIN MOVEMENT CONTROL FUNCTIONS

void openHand() {
  digitalWrite(TI_MOSFET_PIN, HIGH);
  digitalWrite(MRP_MOSFET_PIN, HIGH);
  ti_servo.write(OPEN);
  mrp_servo.write(OPEN);
  delay(MIN_TRAVEL_TIME);
  digitalWrite(TI_MOSFET_PIN, LOW);
  digitalWrite(MRP_MOSFET_PIN, LOW);
}

//Opens/closes fingers over the specified duration
void timedServoWrite(Servo &servo, unsigned servo_mosfet_pin, unsigned &servo_pos, unsigned duration) {
  unsigned long increment_duration = duration / 60; //compute delay needed per increment
  increment_duration = max(increment_duration, 5); //prevent exceeding servo's max speed of 3deg/5ms @7.4V

  digitalWrite(servo_mosfet_pin, HIGH); //turn on servo
  if (servo_pos == OPEN) { //curl in fingers if extended
    for (int i = 1;  i <= 60; ++i) {
      servo.write(i * 3);
      delay(increment_duration);
    }
    servo_pos = CLOSED;
  } else if (servo_pos == CLOSED) { //extend fingers if curled
    for (int i = 59; i >= 0; --i) {
      servo.write(i * 3);
      delay(increment_duration);
    }
    servo_pos = OPEN;
  }
  digitalWrite(servo_mosfet_pin, LOW); //turn off servo to save power
}

//Same as timedServoWrite but with two servos. Couldn't make an array of servo references
void timedTwoServoWrite(Servo &servo1, unsigned servo1_mosfet_pin, unsigned &servo1_pos, Servo &servo2, unsigned servo2_mosfet_pin, unsigned &servo2_pos, unsigned duration) {
  unsigned long increment_duration = duration / 60; //compute delay needed per increment
  increment_duration = max(increment_duration, 5); //prevent exceeding servo's max speed of 3deg/5ms @7.4V

  digitalWrite(servo1_mosfet_pin, HIGH); //turn on servos
  digitalWrite(servo2_mosfet_pin, HIGH);
  if (servo1_pos == OPEN && servo2_pos == OPEN) { //curl in fingers if extended
    for (int i = 1;  i <= 60; ++i) {
      servo1.write(i * 3);
      servo2.write(i * 3);
      delay(increment_duration);
    }
    servo1_pos = CLOSED;
    servo2_pos = CLOSED;
  } else if (servo1_pos == CLOSED && servo2_pos == CLOSED) { //extend fingers if curled
    for (int i = 59; i >= 0; --i) {
      servo1.write(i * 3);
      servo2.write(i * 3);
      delay(increment_duration);
    }
    servo1_pos = OPEN;
    servo2_pos = OPEN;
  } else if (servo1_pos != servo2_pos) { //invert both TI and MRP if asynchronous
    if (servo1_pos == OPEN) {
      for (int i = 1; i <= 60; ++i) {
        servo1.write(i * 3);
        servo2.write((60 - i) * 3);
        delay(increment_duration);
      }
      servo1_pos = CLOSED;
      servo2_pos = OPEN;
    } else {
      for (int i = 59; i >= 0; --i) {
        servo1.write(i * 3);
        servo2.write((60 - i) * 3);
        delay(increment_duration);
      }
      servo1_pos = OPEN;
      servo2_pos = CLOSED;
    }
  }
  digitalWrite(servo1_mosfet_pin, LOW); //turn off servos to save power
  digitalWrite(servo2_mosfet_pin, LOW);
}

//Maps flex strength to servo movement duration in a reverse manner
//in other words: harder the flex, faster the movement
//IDEA: may be useful to use the max recorded signal during calibration instead ANALOG_MAX
unsigned mapFlexToDuration(unsigned flex_signal) {
  flex_signal = constrain(flex_signal, trigger_thresh, ANALOG_MAX);
  return map(flex_signal, trigger_thresh, ANALOG_MAX, MAX_TRAVEL_TIME, MIN_TRAVEL_TIME);
}

//Trigger servo motors based on current mode
void moveFingers(unsigned flex_signal) {
  switch (mode) {
    case full: //go between open and closed hand
      timedTwoServoWrite(mrp_servo, MRP_MOSFET_PIN, mrp_pos, ti_servo, TI_MOSFET_PIN, ti_pos, mapFlexToDuration(flex_signal));
      break;
    case ti:
      timedServoWrite(ti_servo, TI_MOSFET_PIN, ti_pos, mapFlexToDuration(flex_signal));
      break;
    case mrp:
      timedServoWrite(mrp_servo, MRP_MOSFET_PIN, mrp_pos, mapFlexToDuration(flex_signal));
      break;
    case grab:
      if (ti_pos == OPEN && mrp_pos == OPEN) {
        timedServoWrite(mrp_servo, MRP_MOSFET_PIN, mrp_pos, mapFlexToDuration(flex_signal));
      } else if (ti_pos == OPEN && mrp_pos == CLOSED) {
        timedServoWrite(ti_servo, TI_MOSFET_PIN, ti_pos, mapFlexToDuration(flex_signal));
      } else if (ti_pos == CLOSED && mrp_pos == CLOSED) {
        timedTwoServoWrite(mrp_servo, MRP_MOSFET_PIN, mrp_pos, ti_servo, TI_MOSFET_PIN, ti_pos, mapFlexToDuration(flex_signal));
      }
      break;
    case any:
      break;
  }
}

//END MOVEMENT CONTROL FUNCTIONS
//BEGIN VOICE-BASED MODE SELECTION FUNCTIONS

//change mode based on response from voice rec module
void updateMode(byte voiceCommand) {
  mode = (Mode) (voiceCommand - 0x11);
  //initializing actions
  switch (mode) {
    case full:
      openHand();
      break;
    case grab:
      openHand();
      break;
  }
}

//check if there is a need to switch modes
void checkVoiceUpdates() {
  if (voiceRec.available()) {
    byte response = voiceRec.read();
    if (response >= 0x11 && response <= 0x14) {
      updateMode(response);
      for (uint8_t i = 0; i < response - 0x10; ++i) { //blink yellow to indicate current mode. 1 blink = mode 1, 2 blinks = mode 2 etc.
        writeColors(LOW, LOW, LOW);
        delay(DELAY_BLINK);
        writeColors(HIGH, HIGH, LOW);
        delay(DELAY_BLINK);
      }
      writeColors(LOW, LOW, LOW);
      delay(DELAY_BLINK);
      writeColors(LOW, HIGH, LOW);
    }
  }
}

//END VOICE-BASED MODE SELECTION FUNCTIONS
//BEGIN VOICE-REC MODULE COMMUNICATION FUNCTIONS

void VoiceRec::begin(int baudRate = 9600) {
  voiceRecSerial.begin(baudRate);
}

int VoiceRec::available() {
  return voiceRecSerial.available();
}

void VoiceRec::clearBuffer() { // Used to clear out unnecesary responses
  while (voiceRecSerial.available()) {
    voiceRecSerial.read();
  }
}

void VoiceRec::writeToVoiceRec(byte hexKey) { //signals voice-rec module
  voiceRecSerial.write(0xAA);
  voiceRecSerial.write(hexKey);
  delay(250); // delay needed to not overload voice-rec module. Maybe it can be shortened
}

byte VoiceRec::read() { //reads signal from voice-rec module
  return voiceRecSerial.read();
}

byte VoiceRec::waitForResponse() { //reads signal from voice-rec module while pausing everything else
  while (true) {
    if (voiceRecSerial.available()) {
      return voiceRecSerial.read();
    }
  }
}

String VoiceRec::query() { //query and decode recording status
  writeToVoiceRec((byte)0x00); //set voice-rec module to waiting mode
  writeToVoiceRec(0x37); //set voice-rec module to compact (hex) mode
  clearBuffer();
  writeToVoiceRec(0x24); //query voice-rec

  byte response = voiceRecSerial.read();
  switch (response) {
    case (byte)0x00:
      return "No group is recorded";
      break;
    case 0x01:
      return "Group 1 is recorded";
      break;
    case 0x02:
      return "Group 2 is recorded";
      break;
    case 0x04: //not a typo. Idk why it's not in order
      return "Group 3 is recorded";
      break;
    case 0x03:
      return "Groups 1 and 2 are recorded";
      break;
    case 0x05:
      return "Groups 1 and 3 are recorded";
      break;
    case 0x06:
      return "Groups 2 and 3 are recorded";
      break;
    case 0x07:
      return "All 3 groups are recorded";
      break;
    default:
      return "Unknown message: " + String(response);
  }
}

void VoiceRec::record(int groupNum, bool debug = false) { // self-explanatory
  Serial.println("Now recording group " + String(groupNum) + " commands. Please wait for the \"START\" message");

  writeToVoiceRec((byte)0x00); //set voice-rec module to waiting mode
  writeToVoiceRec(0x37); //set voice-rec module to compact (hex) mode
  writeToVoiceRec(groupNum); //delete existing command set
  clearBuffer();
  writeToVoiceRec(groupNum + 0x10); //set voice-rec module to record mode for specific group

  //record until all 5 instructions complete
  int count = 1;
  bool finished = false;
  while (!finished) {
    byte response = waitForResponse();
    if (debug) {
      Serial.println("Debug: " + String(response));
    }
    switch (response) {
      case 0xcc:
        //here for completeness. Don't need to do anything
        break;
      case 0xe0:
        Serial.println("ERROR");
        break;
      case 0x40:
        Serial.println("START: " + String(count) + "/5");
        break;
      case 0x41:
        Serial.println("No voice");
        break;
      case 0x42:
        Serial.println("Again when it says \"START\"");
        break;
      case 0x43:
        Serial.println("Too loud");
        break;
      case 0x44:
        Serial.println("Different");
        break;
      case 0x45:
        Serial.println("Finish one");
        ++count;
        break;
      case 0x46:
        Serial.println("Group 1 finished");
        finished = true;
        writeToVoiceRec(0x21);
        break;
      case 0x47:
        Serial.println("Group 2 finished");
        finished = true;
        writeToVoiceRec(0x22);
        break;
      case 0x48:
        Serial.println("Group 3 finished");
        finished = true;
        writeToVoiceRec(0x23);
        break;
    }
    delay(250);
  }
  return;
}

void VoiceRec::enterListenMode(int groupNum) { //set voice-rec module to listen for specified group of commands
  writeToVoiceRec((byte)0x00); //set voice-rec module to waiting mode
  writeToVoiceRec(0x37); //set voice-rec module to compact (hex) mode
  clearBuffer();
  writeToVoiceRec(groupNum + 0x20); //set voice-rec module to listen mode for specific group

  return;
}

//END VOICE-REC MODULE COMMUNICATION FUNCTIONS

//put your setup code here, to run once
void setup() {
  //Set up servo
  ti_servo.attach(TI_SERVO_PIN);
  mrp_servo.attach(MRP_SERVO_PIN);

  //Set up voice rec module
  voiceRec.begin(9600);
  voiceRec.enterListenMode(1); //Default to command set 1

  //Set MOSFET and LED as outputs of the Arduino
  pinMode(LED_PIN_R, OUTPUT);
  pinMode(LED_PIN_G, OUTPUT);
  pinMode(LED_PIN_B, OUTPUT);
  pinMode(BATT_IN, INPUT);
  pinMode(FLEX_PIN, INPUT);
  pinMode(TI_MOSFET_PIN, OUTPUT);
  pinMode(MRP_MOSFET_PIN, OUTPUT);

  //Initialize hand position
  openHand();

  //Default the LED to off
  digitalWrite(LED_PIN_R, LOW);
  digitalWrite(LED_PIN_G, LOW);
  digitalWrite(LED_PIN_B, LOW);

  //If state is set here to calibration, it will undergo normal calibration; if not, it will have a static threshold
  if (state == CALIBRATION) {
    calibrateSimple();
  } else {
    trigger_thresh = DEFAULT_THRESH;
    relax_thresh = DEFAULT_RELAXTHRESH;
  }
}

//put your main code here, to run repeatedly
void loop() {
  checkBattery(analogRead(BATT_IN)); //check battery
  checkVoiceUpdates(); //check for new voice commands to update mode
  unsigned flex_signal = constrain(smoothAnalogRead(FLEX_PIN), ANALOG_MIN, ANALOG_MAX); //read in flex signal
  //To avoid activating twice on the same signal, make sure it hasn't recently been active before registering another flex
  if (!recently_active && flex_signal > trigger_thresh) {
    delay(128); //Wait for flex to reach inteded position, then reread flex signal
    flex_signal = constrain(smoothAnalogRead(FLEX_PIN), ANALOG_MIN, ANALOG_MAX);
    //Make purple while in use/above threshold
    writeColors(HIGH, LOW, HIGH);
    //If the smooth read is above the threshold, trigger servos
    moveFingers(flex_signal);
    recently_active = true;
  } else if (recently_active && flex_signal < relax_thresh) {
    writeColors(LOW, HIGH, LOW);
    recently_active = false;
  }
  delay(1); //For reading stability. Limits refresh rate to 1000 Hz which is plenty.
}
