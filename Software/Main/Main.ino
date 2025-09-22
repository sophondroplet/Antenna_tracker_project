#include <Wire.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <ESP32Servo.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HardwareSerial.h>

/*
  WARNING NOTE:

  THIS CODE WAS WRITTEN FOR USE WITH THE M10Q-250 GPS MODULE, NOT THE BN-880. CHECK WHICH IS ATTACHED BEFORE UPLOADING.
*/

#define AZIMUTH_SERVO_PIN 12
#define ELEVATION_SERVO_PIN 13 // NOTE: THIS MIGHT NEED TO BE SWAPPED WITH AZIMUTH (we forgot which one was which)
#define UDP_TX_PACKET_MAX_SIZE 64 // this might need to be 255?
#define SDA_PIN 23
#define SCL_PIN 22
#define POT_PIN 39
#define I2C_ADDR 0x0D
#define GPS_BAUD_RATE 115200

// these values are found with the servo tester
#define AZIMUTH_MID_PWM 1431 //Azumith (>1437 clockwise, 1426<=stop<=1437)
#define ELEVATOR_MID_PWM 1533 //Elevator (>1542 angle down, 1524<=stop=<1542)

// these values calibrated at the la quinta inn in maryland on 6/25/2024, using the calibration script within the antenna tracker repo
// #define X_OFFSET -140
// #define Y_OFFSET -47

// these values calibrated at the TUAS lab on 6/14/2025, using the calibration script within the antenna tracker repo
// #define X_OFFSET -300
// #define Y_OFFSET 3
// #define X_SCALE 1
// #define Y_SCALE 1

#define X_OFFSET -246.28
#define Y_OFFSET -26.16
#define X_SCALE 0.992223
#define Y_SCALE 1.007900

const double pi = 3.14159265359;

//set up to connect to an existing network (e.g. mobile hotspot from laptop that will run the python code)
const char* ssid = "INNOUT";
const char* password = "animalfries";
IPAddress local_IP(192, 168, 1, 36);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4); 

// Buffer to hold incoming packet, expecting "lat,lon,altitude" (in meters), default to top of Geisel
char packetBuffer[UDP_TX_PACKET_MAX_SIZE] = "32.88114,-117.23758,150";

WiFiUDP Udp;
unsigned int localUdpPort = 4000;  //  port to listen on

TinyGPSPlus gpsStation;

Servo azimuthServo;
const uint16_t azimuth_servo_min_pwm = 1100; // spin fast
const uint16_t azimuth_servo_max_pwm = 1900; // spin fast but the other way

Servo elevationServo;
const uint16_t elevation_servo_min_pwm = 1000; // up
const uint16_t elevation_servo_max_pwm = 1920; // horizon

// Filter cutoff frequency in Hz
const double headingCutoff = 5;
const double latCutoff = 0.05;
const double longCutoff = 0.05;
const double altCutoff = 0.05;

// Main loop target frequency in Hz
const double frequency = 100;

// Controller gains
const double pGain_azimuth = 6;
const double iGain_azimuth = 0;
const double dGain_azimuth = 0;

const double pGain_elevation = 10;
const double iGain_elevation = 0;
const double dGain_elevation = 0.05;

// For timing the main loop
uint32_t previousMillis;

// WGS84 - (World Geodetic System) is a standard that defines a geodetic datum for Earth. Functions that input this assume the reference ellipsoid defined by WGS84.
struct WGS84 {
  double lat; //Lattitude in radians
  double lon; //Longitude in radians
  double alt; //Altitude in meters
};

// ECEF - Earth Centered, Earth Fixed coordinate system. 0,0,0 is the center of the Earth.
struct ECEF {
  double x; //Meters in the plane of the equator in the direction of the prime meridian
  double y; //Meters in the plane of the equator in the direction of 90 degrees East
  double z; //Meters in the direction of the North pole
};

// ENU - East, North, Up coordinate system. Used to show an offset from a certain location on the Earth, usually a ground station.
struct ENU {
  double e; //Meters East from reference location
  double n; //Meters North from reference location
  double u; //Meters Up from reference location
};

// Converts a lattitude, longitude and altitude to ECEF coordinates using the WGS84 Ellipsoid
ECEF WGS84toECEF(WGS84 wgs84) {
  double a = 6378137;  //Earth semi-major axis in meters
  double b = 6356752;  //Earth semi-minor axis in meters
  double e2 = 1 - (b*b)/(a*a);
  ECEF ecef;
  ecef.x = (wgs84.alt + a/(sqrt(1-e2*sin(wgs84.lat)*sin(wgs84.lat))))*cos(wgs84.lat)*cos(wgs84.lon);
  ecef.y = (wgs84.alt + a/(sqrt(1-e2*sin(wgs84.lat)*sin(wgs84.lat))))*cos(wgs84.lat)*sin(wgs84.lon);
  ecef.z = (wgs84.alt + (1-e2)*a/(sqrt(1-e2*sin(wgs84.lat)*sin(wgs84.lat))))*sin(wgs84.lat);
  return ecef;
}

// Converts two sets of ECEF coordinates to an ENU offset from the perspective of the given lattitude and longitude
ENU ECEFtoENU(WGS84 ground, ECEF station, ECEF aircraft) {
  ECEF delta;
  delta.x = aircraft.x - station.x;
  delta.y = aircraft.y - station.y;
  delta.z = aircraft.z - station.z;
  ENU enu;
  enu.e = (delta.x)*(-sin(ground.lon)) + (delta.y)*(cos(ground.lon));
  enu.n = (delta.x)*(-sin(ground.lat)*cos(ground.lon)) + (delta.y)*(-sin(ground.lat)*sin(ground.lon)) + (delta.z)*(cos(ground.lat));
  enu.u = (delta.x)*(cos(ground.lat)*cos(ground.lon)) + (delta.y)*(cos(ground.lat)*sin(ground.lon)) + (delta.z)*(sin(ground.lat));
  return enu;
}

// PID controller
double PIDcontroller_azimuth(double state, double setPoint) {
    // Calculate error in terms of shortest path to setpoint
    double errorDegrees = -(fmod(state - setPoint + 540, 360) - 180);

    double proportional;
    static double integral;
    double derivative;
    static double previousError;

    proportional = errorDegrees;
    integral = integral + (errorDegrees / frequency);
    derivative = (errorDegrees - previousError) * frequency;
    previousError = errorDegrees;
    
    Serial.print("Azimuth error:");
    Serial.print(errorDegrees, 1);
    Serial.print("P:");
    Serial.print(pGain_azimuth*proportional, 1);
    Serial.print("  I:");
    Serial.print(iGain_azimuth*integral, 1);
    Serial.print("  D:");
    Serial.print(dGain_azimuth*derivative, 1);
    Serial.print("  ");
    
    double controllerOutput = pGain_azimuth*proportional + iGain_azimuth*integral + dGain_azimuth*derivative;
    return controllerOutput;
}



double PIDcontroller_elevation(double state, double setPoint) {
    // Calculate error in terms of shortest path to setpoint
    double errorDegrees = fmod(state - setPoint + 540, 360) - 180;

    double proportional;
    static double integral;
    double derivative;
    static double previousError;

    proportional = errorDegrees;
    integral = integral + (errorDegrees / frequency);
    derivative = (errorDegrees - previousError) * frequency;
    previousError = errorDegrees;
    
    // Serial.print("Elevetor error:");
    // Serial.print(errorDegrees, 1);
    // Serial.print("P:");
    // Serial.print(pGain_elevation*proportional, 1);
    // Serial.print("  I:");
    // Serial.print(iGain_elevation*integral, 1);
    // Serial.print("  D:");
    // Serial.print(dGain_elevation*derivative, 1);
    // Serial.print("  ");
    
    double controllerOutput = pGain_elevation*proportional + iGain_elevation*integral + dGain_elevation*derivative;
    return controllerOutput;
}

void updateFilteredGPS(WGS84& station, WGS84& station_filtered) {
  static const double latSmoothingFactor = latCutoff * 2*M_PI / frequency
                             / (1 + latCutoff * 2*M_PI / frequency);
  static const double longSmoothingFactor = longCutoff * 2*M_PI / frequency
                             / (1 + longCutoff * 2*M_PI / frequency);
  static const double altSmoothingFactor = altCutoff * 2*M_PI / frequency
                             / (1 + altCutoff * 2*M_PI / frequency);

  static bool inited = false;

  if (!inited &&
    station.lat != 0.0 &&
    station.lon != 0.0 &&
    station.alt != 0.0) 
  {
    station_filtered = station;
    inited = true;
    return;
  }

  if (!inited){
    station_filtered.lat = station.lat;
    station_filtered.lon = station.lon;
    station_filtered.alt = station.alt;
  }
    
  station_filtered.lat += latSmoothingFactor * (station.lat - station_filtered.lat);
  station_filtered.lon += longSmoothingFactor * (station.lon - station_filtered.lon);
  station_filtered.alt += altSmoothingFactor * (station.alt - station_filtered.alt);
}

/*
  Initializes the QMC5883L magnetometer and returns -1 if an error occurred, 0 otherwise.
*/
int qmc5883lInit() {
  // Reset magnetometer
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(0x0A);
  Wire.write(0b10000000); // Set soft reset bit
  Wire.endTransmission();

  // Configure set/reset period (I have no idea what this does tbh but it's in the datasheet)
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(0x0B);
  Wire.write(0x01);
  Wire.endTransmission();

  // Set config settings
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(0x09);
  Wire.write(0b00011101); // Config register 1: continuous mode, output data rate of 200Hz, 8G scale, over sample ratio of 64
  Wire.endTransmission();

  // Basic I2C test
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(0x0D);
  Wire.endTransmission();

  uint8_t chipID = 0;
  int numBytes = Wire.requestFrom(I2C_ADDR, 1);
  if (numBytes) {
    Wire.readBytes(&chipID, numBytes);
    Serial.printf("Received chip ID: %02X (expected 0xFF)\n", chipID);
  } else {
    Serial.println("ERROR: COULD NOT VERIFY CHIP ID");
  }

  if (chipID != 0xFF) {
    return -1;
  }

  return 0;
}

/*
  Waits for the QMC5883L DRDY bit to be set, then reads X, Y, and Z data from the magnetometer into the given int16_t pointers.
  Returns -1 if an error occurred in transmission and 0 otherwise.
*/
int qmc5883lGetData(int16_t* x, int16_t* y, int16_t* z) {
  // wait for DRDY bit to be set
  uint8_t status = 0;
  while (!(status & 0b00000001)) {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(0x06);
    Wire.endTransmission();

    int statusBytesReceived = Wire.requestFrom(I2C_ADDR, 1);
    if (statusBytesReceived) {
      Wire.readBytes(&status, 1);
    } else {
      Serial.println("I2C ERROR");
    }
  }

  // Data is ready, read fresh mag values
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
 
  uint8_t magData[6] = {0};
  int bytesReceived = Wire.requestFrom(I2C_ADDR, 6);
  if (bytesReceived) {
    Wire.readBytes(magData, 6);
    *x = (magData[1] << 8) | magData[0];
    *y = (magData[3] << 8) | magData[2];
    *z = (magData[5] << 8) | magData[4];
    return 0;
  } else {
    Serial.println("ERROR: no data received");
    return -1;
  }
}

void setup() {  
  //Begin serial connection to computer
  Serial.begin(115200);

  // Begin serial connection to GPS
  Serial1.begin(GPS_BAUD_RATE, SERIAL_8N1, 16, 17);

  delay(1000);


  // Initialize wifi connection to INNOUT router
  WiFi.begin(ssid,password);

  // Wait for connection
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected!");
  Udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);

  // Start I2C on corresponding Huzzah32 SDA/SCL pins
  Wire.begin(SDA_PIN, SCL_PIN);

  int magstatus = -1;
  while (magstatus != 0) {
    magstatus = qmc5883lInit();
  }

  azimuthServo.attach(AZIMUTH_SERVO_PIN);
  elevationServo.attach(ELEVATION_SERVO_PIN);

  // Stop spinning the azimuth servo and set the elevation servo to 45ish degrees
  azimuthServo.writeMicroseconds(1500);
  elevationServo.writeMicroseconds(1500);

  // Give time to read debug serial messages
  Serial.println("Setup complete");
  delay(1000);
}

void loop() {
  uint32_t currentMillis = millis();

  if (currentMillis - previousMillis >= 1000 / frequency) {
    previousMillis = currentMillis;

    // Feed characters to the gps object
    while (Serial1.available() > 0) {
      gpsStation.encode(Serial1.read());
    }

    // Calculate current elevation
    int adcValue = analogRead(POT_PIN); // Read raw value (0-4095)
    double elevation = ((double)adcValue-1104)*(90.0/(2510-1104)); //1104 = 0 deg (min), 2510 = 90 deg (max)
    Serial.print("  Elevation:");
    Serial.print(elevation);

    // Update station state
    WGS84 station;
    station.lat = gpsStation.location.lat() * pi/180;
    station.lon = gpsStation.location.lng() * pi/180;
    station.alt = gpsStation.altitude.meters();

    //Low pass filter for station coordinates 
    static WGS84 station_filtered;
    updateFilteredGPS(station, station_filtered);

    // static const double latSmoothingFactor = latCutoff * 2 * pi / frequency / (1 + 2 * pi * latCutoff / frequency);
    // static const double longSmoothingFactor = longCutoff * 2 * pi / frequency / (1 + 2 * pi * longCutoff / frequency);
    // static const double altSmoothingFactor = altCutoff * 2 * pi / frequency / (1 + 2 * pi * altCutoff / frequency);

    // static double station_filteredlat = station.lat;
    // static double station_filteredlon = station.lon;
    // static double station_filteredalt = station.alt;

    // station_filteredlat = station_filteredlat + latSmoothingFactor * (station.lat - station_filteredlat);
    // station_filteredlon = station_filteredlon + longSmoothingFactor * (station.lon - station_filteredlon);
    // station_filteredalt = station_filteredalt + altSmoothingFactor * (station.alt - station_filteredalt);

    // static WGS84 station_filtered;
    // station_filtered.lat = station_filteredlat;
    // station_filtered.lon = station_filteredlon;
    // station_filtered.alt = station_filteredalt;

    //EBU lab coordinates
    // station.lat = 32.882047 * pi/180;
    // station.lon = -117.235520 * pi/180;
    // station.alt = 128.10;

    //competition site coordinates
    // station.lat = 38.316016 * pi/180;
    // station.lon = -76.54860 * pi/180;
    // station.alt = 8.05;


    // Write new packets to packetBuffer
    uint16_t packetSize = Udp.parsePacket();
    if (packetSize) {
      // Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);

      int bytesRead = Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE - 1);
      packetBuffer[bytesRead] = '\0';

      Serial.print("Bytes read: ");
      Serial.println(bytesRead);
    }

    Serial.print("Packet received: ");
    Serial.println(packetBuffer);

    // Update aircraft state
    WGS84 aircraft;
    char packetParse[UDP_TX_PACKET_MAX_SIZE];
    strcpy(packetParse, packetBuffer);
    char *token;
    token = strtok(packetParse, ",");
    aircraft.lat = String(token).toDouble() * pi / 180;
    token = strtok(NULL, ",");
    aircraft.lon = String(token).toDouble() * pi / 180;
    token = strtok(NULL, ",");
    aircraft.alt = String(token).toDouble();

    // Calculate ENU offset from station to aircraft
    ENU enu = ECEFtoENU(station, WGS84toECEF(station_filtered), WGS84toECEF(aircraft));

    // Get mag data
    int16_t magXVal = 0;
    int16_t magYVal = 0;
    int16_t magZVal = 0;
    int status = qmc5883lGetData(&magXVal, &magYVal, &magZVal);
    if (status != 0) {
      Serial.println("ERROR: MAGNETOMETER DATA NOT RECEIVED");
    } else {
      double heading = -atan2((magYVal - Y_OFFSET)*Y_SCALE, (magXVal - X_OFFSET)*X_SCALE) * 180/pi; // Accounting for board orientation
      double declination = -10.9333; // For Maryland, need to implement adjustments for other locations
      heading += declination;
      // heading += 18.0;

      if (heading<-180){ 
        heading = 360 - abs(heading);
      }else if (heading>180){
        heading = -360 + abs(heading);
      }

      // Single-pole IIR low-pass filter
      static const double headingSmoothingFactor = headingCutoff * 2 * pi / frequency / (1 + 2 * pi * headingCutoff / frequency);
      static double headingFiltered = heading;
      headingFiltered = headingFiltered + headingSmoothingFactor * (heading - headingFiltered);

      // Calculate desired azimuth and elevation
      double azimuthSetpoint = atan2(enu.e, enu.n) * 180/pi;
      double elevationSetpoint = constrain(atan2(enu.u, sqrt(enu.e*enu.e + enu.n*enu.n)) * 180/pi, 0, 90);

      // Calculate PWM values to write to servos
      double controllerOutput_azimuth = PIDcontroller_azimuth(heading, azimuthSetpoint);
      double azimuthCommand = constrain(AZIMUTH_MID_PWM + controllerOutput_azimuth, azimuth_servo_min_pwm, azimuth_servo_max_pwm);
      double controllerOutput_elevation = PIDcontroller_elevation(elevation, elevationSetpoint);
      double elevationCommand = constrain(ELEVATOR_MID_PWM + controllerOutput_elevation, azimuth_servo_min_pwm, azimuth_servo_max_pwm);

      azimuthServo.writeMicroseconds(azimuthCommand);
      elevationServo.writeMicroseconds(elevationCommand);
      Serial.println("azimuth servo target: " + String(azimuthCommand));
      Serial.println("elevation servo target: " + String(elevationCommand));
      
      // Time the main loop
      static uint32_t tic;
      uint32_t toc = millis();
      uint16_t time = toc - tic;
      tic = millis();

      Serial.print("  Aircraft: ");
      Serial.print(aircraft.lat * 180 / pi, 6);
      Serial.print(", ");
      Serial.print(aircraft.lon * 180 / pi, 6);
      Serial.print(", ");
      Serial.print(aircraft.alt, 2);
      Serial.print("  Station: ");
      Serial.print(station.lat * 180 / pi, 6);
      Serial.print(", ");
      Serial.print(station.lon * 180 / pi, 6);
      Serial.print(", ");
      Serial.print(station.alt, 2);

      Serial.print("  Station Filtered: ");
      Serial.print(station_filtered.lat * 180 / pi, 6);
      Serial.print(", ");
      Serial.print(station_filtered.lon * 180 / pi, 6);
      Serial.print(", ");
      Serial.print(station_filtered.alt, 2);
      
      Serial.print("  ElevationSet:");
      Serial.print(elevationSetpoint, 2);
      Serial.print("  AzimuthSet:");
      Serial.print(azimuthSetpoint, 2);
      Serial.print("  Heading:");
      Serial.print(heading, 2);
      Serial.print("  HeadingFiltered:");
      Serial.print(headingFiltered, 2);
      Serial.print("  Time:");
      Serial.print(time);
      Serial.println();
    }
  }
}