int fsrAnalogPin = 0;
int fsrReading = 0;
int IRSensor = 9; 
int sensorStatus = 0;
int LED = 12;

void setup() {
  Serial.begin(9600);
    pinMode(IRSensor, INPUT); 
    pinMode(LED, OUTPUT);
}

void loop() {
  fsrReading = analogRead(fsrAnalogPin);
  Serial.print("Pressure Sensor Reads: ");
  Serial.println(fsrReading);
  sensorStatus = digitalRead(IRSensor);
  Serial.print("IR Sensor Reads: ");
  Serial.println(sensorStatus);

  if (sensorStatus == 0 && fsrReading > 400){
    digitalWrite(LED, HIGH);
  }
  else{
    digitalWrite(LED, LOW);
  }
}
