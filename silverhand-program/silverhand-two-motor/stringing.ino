// This code will be used for stringing the servos of a two motor system
#include <Servo.h>
#define SERVO_PIN1 11//11
#define SERVO_PIN2 10
#define MOSFET_PIN1 12
#define MOSFET_PIN2 13

Servo myservo1;
Servo myservo2;
int pos = 0;
int deg = 180; // Change this value to change the servos' position; input 0, 179, 180


void setup() {
  //Setup IO
  myservo1.attach(SERVO_PIN1);
  myservo2.attach(SERVO_PIN2);
  pinMode(MOSFET_PIN1, OUTPUT);
  pinMode(MOSFET_PIN2, OUTPUT);
  //Set Initial State
  digitalWrite(MOSFET_PIN1,HIGH);
  digitalWrite(MOSFET_PIN2,HIGH);
  Serial.begin(9600);
  delay(50);



  if (deg == 0) {
    //For 0 degrees:

    for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
      myservo1.write(pos);              // tell servo to go to position in variable ‘pos’
      myservo2.write(pos);
      Serial.println(pos);
      delay(15);                    // waits 15ms for the servo to reach the position
    }
  }
  else {
    //For 180 Degrees
    for (pos = 0; pos <= 180; pos++) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      myservo1.write(pos);              // tell servo to go to position in variable ‘pos’
      myservo2.write(pos);
      Serial.println(pos);
      delay(15);                       // waits 15ms for the servo to reach the position
    }
  }
}

void loop() {
  myservo1.write(deg); 
  myservo2.write(deg);
  delay(150);
  Serial.println("in loop");
}
