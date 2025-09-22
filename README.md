# Antenna Tracker Project

## Project Overview
The Antenna Tracker Project is designed to track and maintain communication with a moving object, such as a drone or satellite, by dynamically adjusting the orientation of an antenna. This project leverages GPS data, magnetometer calibration, and servo control to achieve precise tracking.

## Directory Structure and File Descriptions

### 1. `GPS_tester/GPS_tester.ino`
This script tests the GPS module by reading and displaying latitude, longitude, and altitude data. It uses the `TinyGPS++` library to parse GPS data and outputs the results via the serial monitor.

### 2. `GPS_tester_send/GPS_tester_send.ino`
Similar to `GPS_tester.ino`, this script reads GPS data but also transmits the raw GPS data over a serial connection. It includes LED indicators for activity.

### 3. `Magentormeter_calibration/Magentormeter_calibration.py`
This Python script calibrates the magnetometer by fitting an ellipsoid to the raw data. It calculates hard iron offsets and soft iron transformation matrices to correct the magnetometer readings. The script uses libraries like `numpy`, `matplotlib`, and `scipy` for data processing and visualization.

### 4. `Main/Main.ino`
The main control script for the antenna tracker. It integrates GPS data, WiFi communication, and servo control to dynamically adjust the antenna's azimuth and elevation. Key features include:
- WiFi setup for communication with a ground station.
- UDP packet handling for receiving GPS coordinates.
- Servo motor control for antenna orientation.
- Calibration parameters for precise tracking.

### 5. `Servo_tester/Servo_tester.ino`
This script tests the servo motors by allowing manual control via the serial monitor. Users can input PWM values to adjust the servo position and observe the response.

### 6. `UDP_GPS_send_tester/UDP_GPS_send_tester.ino`
This script tests the UDP communication by sending GPS data over a network. It ensures that the GPS data can be transmitted and received correctly.

## Additional Resources

### CAD Files
The `CAD_files` directory contains 3D models and components for the antenna tracker. These files can be used for manufacturing or 3D printing the hardware. Key files include:
- `Antenn tracker 3D model.step`: Complete 3D model of the antenna tracker.
- `Antenna_Tracker-elevator gear.step`: Gear for the elevator mechanism.
- `Antenna_Tracker-arm_left.step` and `Antenna_Tracker-arm_right.step`: Left and right arms of the tracker.
- `Antenna_Tracker-battery holder left.step` and `Antenna_Tracker-battery holder right.step`: Battery holders.
- `Antenna_Tracker-Spur gear (18 teeth).step` and `Antenna_Tracker-Spur gear (20 teeth).step`: Spur gears for the mechanism.

### Images
The `assets/images` directory contains images of the project and CAD renderings. These images provide a visual reference for the hardware setup and design. Key images include:
- `atrCAD1.png` to `atrCAD6.png`: Rendered CAD images of the antenna tracker.
- `atrimg1.jpg` to `atrimg7.jpg`: Photos of the assembled hardware and components.

### Usage
- Use the CAD files to 3D print or manufacture the components.
- Refer to the images for assembly guidance and to understand the hardware layout.

## Setup Instructions

### Hardware Requirements
- ESP32 microcontroller
- GPS module (e.g., M10Q-250 or BN-880)
- Magnetometer
- Servo motors for azimuth and elevation control
- WiFi network for communication

### Software Requirements
- Arduino IDE
- Python 3.x
- Required Python libraries: `numpy`, `matplotlib`, `scipy`
- TinyGPS++ library for Arduino
- ESP32Servo library for Arduino

### Steps to Set Up
1. **Install Arduino IDE**
   - Download and install the Arduino IDE from [arduino.cc](https://www.arduino.cc/).

2. **Install Required Libraries**
   - Open the Arduino IDE and go to `Sketch > Include Library > Manage Libraries`.
   - Search for and install the following libraries:
     - `TinyGPS++`
     - `ESP32Servo`

3. **Upload Scripts**
   - Connect the ESP32 to your computer via USB.
   - Open the desired `.ino` file in the Arduino IDE.
   - Select the correct board (`ESP32 Dev Module`) and port from the `Tools` menu.
   - Click the upload button to flash the script onto the ESP32.

4. **Calibrate the Magnetometer**
   - Connect the magnetometer to your computer.
   - Run `Magentormeter_calibration.py` using Python.
   - Follow the on-screen instructions to collect and calibrate data.

5. **Test Components**
   - Use `GPS_tester.ino` to verify GPS functionality.
   - Use `Servo_tester.ino` to test servo motor control.
   - Use `UDP_GPS_send_tester.ino` to test UDP communication.

6. **Run the Main Script**
   - Upload `Main.ino` to the ESP32.
   - Ensure all hardware components are connected.
   - Monitor the serial output for debugging and status updates.

## Notes
- Ensure the GPS module is connected to the correct pins as specified in the scripts.
- Calibration values for the magnetometer and servos may need to be adjusted based on your setup.
- The WiFi credentials in `Main.ino` should be updated to match your network.

