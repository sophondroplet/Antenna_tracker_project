#include <TinyGPS++.h>
#define GPS_BAUD_RATE 115200


TinyGPSPlus gps;

int LED_PIN = 2;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  Serial1.begin(GPS_BAUD_RATE, SERIAL_8N1, 16, 17);
}

void loop() {

  while (Serial1.available() > 0)
  gps.encode(Serial1.read());
  // digitalWrite(LED_PIN, 1);
  // delay(100);
  // digitalWrite(LED_PIN, 0);
  // delay(100);
  if (gps.location.isUpdated()) {
    Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
    Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
    Serial.print("ALT=");  Serial.println(gps.altitude.meters());
}
}
