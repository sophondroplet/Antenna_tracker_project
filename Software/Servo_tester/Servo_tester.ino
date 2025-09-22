#include<ESP32Servo.h>
Servo servo;
int servoPos = 90;
const int LED_PIN = 13;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  servo.attach(12);
  servo.write(servoPos);
  
  Serial.begin(9600);
  Serial.setTimeout(100);
  Serial.println(servoPos);
}

//Elevator (>1542 angle down, 1524<=stop=<1542)
//Azumith (>1437 clockwise, 1426<=stop<=1437)

void loop() {
  if (Serial.available() > 0) {
  // read until you see '\n' (you must press Enter in the Serial Monitor)
    String s = Serial.readStringUntil('\n');
    int setpoint_microsecs = s.toInt();
    // setpoint_microsecs = constrain(setpoint_microsecs, 1000, 2000);

    servo.writeMicroseconds(setpoint_microsecs);
    Serial.println(setpoint_microsecs);
  }
}



