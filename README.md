# ESP32-Webserver-Final-RTOS-CS452
This is the final implementation of the ESP32 webserver project which uses various sensors and communicates data to a server hosted on AWS.

The full list of parts used in the project are:

HDC 1080 Temperature / Humidity Sensor
Stepper Motor
OLED Display
GPS Device
ESP32
Vandaluino3 PCB

This program is set up in PlatformIO using the Arduino framework in a FreeRTOS environment with the ESP32. 
The main functionality of this program is to host a web server from the ESP32 which recieves and displays data from the
HDC 1080 Temperature / Humidity Sensor and Stepper Motor. Additionoally, the Stepper Motor direction is controllable from the 
web page as well as the on-board push buttons. 

Temperature, Humidity, GPS Data (Latitude, Longitude and Altitude) and current time in UTC are uploaded to a webserver
hosted on AWS at the University of Idaho. 

GPS Data is additionally displayed on the OLED Display.

The LED lights are display the motor's current direction through color codes. LEDs 0 and 1 will be Bright Green if the motor
is moving in a clockwise direction, and Bright Cyan if moving in a Counter-Clockwise Direction. 
Led 2 emits a Purple blink whenever data is communicated to the school webserver, and Led 3 emites a Yellow blink
when data is sent to the web page hosted by the ESP32. 

Motor Function include:

Rotating Clockwise continuously. 
Rotating Counter-Clockwise continuously. 
Rotating on Temperature differences.
Rotatin on Humididty differences. 
Rotating one full rotation Clockwise, the Counter-clockwise and repeating. 

This project was created while attending CS452 at the University of Idaho. 
