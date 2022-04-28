#include <Arduino.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <FreeRTOS.h>
#include <Arduino_JSON.h>
#include <AsyncTCP.h>
#include <HTTPClient.h>
#include "TinyGPS++.h"
#include <iomanip>
#include "ACROBOTIC_SSD1306.h"



//Deinfe HTTP port to run server on
#define HTTP_PORT 80

//Define HDC1080 I2C addresses
// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
#define HDC1080TEMPREG 0x00
#define HDC1080HUMREG 0x01
#define HDC1080CONFIGREG 0x02
#define HDC1080ADDRESS 0x40
#define HDC1080SN1 0xFB
#define HDC1080SN2 0xFC
#define HDC1080SN3 0xFD
#define HDC1080DEVICEIDREG 0xFE
#define HDC1080DEVICEID 0xFF

//Define button pins
#define BUTTON1 18
#define BUTTON2 15
#define BUTTON3 32

//Define Step Motor Pins
#define StepMotorIN1 26
#define StepMotorIN2 27
#define StepMotorIN3 25
#define StepMotorIN4 14

//onboard LEDs
#define PIN 19
#define NUM_LEDS 4
#define BRIGHTNESS 50

#include "PixelFunctions.h"

//longest string lenght of int
#define INT_STR_LEN (sizeof(int)*CHAR_BIT/3 + 2)

//Declare Queues
QueueHandle_t tempDisplayQueue;
QueueHandle_t humDisplayQueue;
QueueHandle_t smButtonQueue;
QueueHandle_t motorStatQueue;
QueueHandle_t gpsTimeQueue;
QueueHandle_t gpsLatQueue;
QueueHandle_t gpsLongQueue;
QueueHandle_t gpsAltQueue;
QueueHandle_t motledColorQueue;
QueueHandle_t oledledColorQueue;
QueueHandle_t oledLatQueue;
QueueHandle_t oledLngQueue;
QueueHandle_t oledAltQueue;

//Semaphore for i2c BUS
SemaphoreHandle_t buttonSem;
SemaphoreHandle_t i2cSem;

//Wifi connection credentials for my network
const char *wifi_SSID = "********";
const char *wifi_PWD = "********";

//Declare webserver
AsyncWebServer server(80);

//Event handler to update webpage with temperature and humidity values
AsyncEventSource events ("/events");

JSONVar readings;
JSONVar serverData;
JSONVar oledData;

//FreeRTOS Tasks
void webserver_Setup_Task(void *pvParameter);
void HDC1080_Control_Task(void *pvParameter);
void rtos_Server_Task(void *pvParameter);
void step_Motor_Task(void *pvParameter);
void buttons_Task(void *pvParameter);
void read_Gps(void *pvParameter);
void oled_Task(void *pvParameter);
void led_Color_Task(void *pvParameter);

//Wifi / Server Function Prototypes
void wifi_connect();
void start_spiffs();
void start_server();
String sendSensorReading();

//HDC1080 Function Prototypes
uint16_t readHDC1080(uint8_t baseAddress, uint8_t regAddress);
int readSN1(uint8_t address);
int readSN2(uint8_t address);
int readSN3(uint8_t address);
int readConfig(uint8_t address);
int readMfID(uint8_t address);
int readHumidity(uint8_t address);
int readTemperature(uint8_t address);

//Step Motor Function Prototypes
void rotateCW();
void rotateCCW();
void fullRotateFB();
void rotateOnTemp();
void rotateOnHum();
void emergencyStop();

//Buttons Task Function Prototypes
void getButtons();

//rtos server Task function prototypes
void rtosDetectServer();
String rtosRegisterServer();
void rtosQueryCommands(String authCode);
void rtosSendData(String authCode, JSONVar jData);

//function to set LED behavior / color
void ledControl();

//String to store auth code
String authCode;

//define core to run tasks on
static int task_core = 1;

unsigned long startMS;

void setup() {

  startMS = millis();

  //Start Wire for reading I2C
  Wire.begin();

  //Start serial communication to print into to console
  Serial.begin(115200);

  //start serial comm for gps
  Serial1.begin(9600);
  Serial2.begin(9600);

  //Create Queues
  tempDisplayQueue = xQueueCreate(2, sizeof(int));
  humDisplayQueue = xQueueCreate(2, sizeof(int));
  smButtonQueue = xQueueCreate(2, sizeof(int));
  motorStatQueue = xQueueCreate(2, sizeof(int));
  gpsTimeQueue = xQueueCreate(2, 64);
  gpsLatQueue = xQueueCreate(2, sizeof(float));
  gpsLongQueue = xQueueCreate(2, sizeof(float));
  gpsAltQueue = xQueueCreate(2, sizeof(float));
  motledColorQueue = xQueueCreate(4, sizeof(int));
  oledledColorQueue = xQueueCreate(4, sizeof(int));
  oledLatQueue = xQueueCreate(2, sizeof(float));
  oledLngQueue = xQueueCreate(2, sizeof(float));
  oledAltQueue = xQueueCreate(2, sizeof(float));

  //Create Semaphores
  vSemaphoreCreateBinary(buttonSem);
  vSemaphoreCreateBinary(i2cSem);

  //Initialize on-board buttons
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  pinMode(BUTTON3, INPUT);

  //Initialize StepMotor pins
  pinMode(StepMotorIN1, OUTPUT);
  pinMode(StepMotorIN2, OUTPUT);
  pinMode(StepMotorIN3, OUTPUT);
  pinMode(StepMotorIN4, OUTPUT);

  dumpSysInfo();
  getMaxMalloc(1 * 1024, 16 * 1024 * 1024);

  if (digitalLeds_initStrands(STRANDS, STRANDCNT)) {
    Serial.println("Init FAILURE: halting");
    while (true) {};
  }
  for (int i = 0; i < STRANDCNT; i++) {
    strand_t * pStrand = &STRANDS[i];
    Serial.print("Strand ");
    Serial.print(i);
    Serial.print(" = ");
    Serial.print((uint32_t)(pStrand->pixels), HEX);
    Serial.println();
#if DEBUG_ESP32_DIGITAL_LED_LIB
    dumpDebugBuffer(-2, digitalLeds_debugBuffer);
#endif
    digitalLeds_resetPixels(pStrand);
#if DEBUG_ESP32_DIGITAL_LED_LIB
    dumpDebugBuffer(-1, digitalLeds_debugBuffer);
#endif
  }

  //Create Task for wifi set up and connection, pin to core arduino is running on
  xTaskCreatePinnedToCore(webserver_Setup_Task, "webserver_Setup", 5000, NULL, 1, NULL, CONFIG_ARDUINO_RUNNING_CORE);

  //Create Task for reading from HDC1080 Sensor, pin to core 0. 
  xTaskCreatePinnedToCore(HDC1080_Control_Task, "HDC1080_Control_Task", 5000, NULL, 1, NULL, task_core);

  //Create Task to connect and send commands to the rtos server
  xTaskCreatePinnedToCore(rtos_Server_Task, "rtos_Server", 5000, NULL, 1, NULL, task_core);

  //Create Task for Step Motor Driver
  xTaskCreatePinnedToCore(step_Motor_Task, "SM_Task", 5000, NULL, 1, NULL, task_core);

  //Create Task for on-board buttons
  xTaskCreatePinnedToCore(buttons_Task, "buttons_Task", 5000, NULL, 1, NULL, task_core);

  xTaskCreatePinnedToCore(read_Gps, "GPS Task", 5000, NULL, 1, NULL, task_core);

  xTaskCreatePinnedToCore(oled_Task, "oled_Task", 5000, NULL, 1, NULL, task_core);

  xTaskCreatePinnedToCore(led_Color_Task, "led_Color_Task", 5000, NULL, 1, NULL, task_core);
}

void loop(){
  // put your main code here, to run repeatedly:
  vTaskDelay(portMAX_DELAY);
  //vTaskDelete(NULL);
}


/////////////////////////////FreeRTOS Tasks START///////////////////////////////////

//Task to connect to UofI Rtos server
void rtos_Server_Task(void *pvParameter){

  bool registered = false;

  while(true){

      //Only run these commands if connected to Wifi
      if(WiFi.status() == WL_CONNECTED){
        rtosDetectServer();

        if(!registered){

          authCode = rtosRegisterServer();
          registered = true;
        }

        printf("Auth Code from server %s\n", authCode.c_str());


        //Query server for commands
        rtosQueryCommands(authCode);
        
        vTaskDelay(10000/portTICK_PERIOD_MS);
    }
  }
}

//Task to setup up wifi connection and start server
void webserver_Setup_Task(void *pvParameter){

  //On first run, load webpage data from image stored in SPIFFS
  //connect to wifi and start server
  start_spiffs();
  wifi_connect();
  start_server();    

  while(true){

    //Additional check to keep wifi connected
    //This section will reconnect to wifi if the connection is lost or 
    //dropped
    if(WiFi.status() != WL_CONNECTED){
     
      wifi_connect();
      vTaskDelay(100/portTICK_PERIOD_MS);

    }
    else{
      //wifi is already connected. Delay 30 sectonds and continue While loop.
      printf("Already connected to wifi\n");
      yield();
      vTaskDelay(30000/portTICK_PERIOD_MS);
    }
  }
}

void HDC1080_Control_Task(void *pvParameter) {

  vTaskDelay(1000/portTICK_PERIOD_MS);

    //Initialize variables
    int configStat;
    int mfID;
    int serialNum1;
    int serialNum2;
    int serialNum3;
    int temperatureInC;
    int temperatureInF;
    int humidity;
    //int nDevices;

    uint8_t i2cAddress = HDC1080ADDRESS;

  //Read HDC info once on initial start of task 
  //Basically acts a verification of the device connected
  configStat = readConfig(i2cAddress);
  mfID = readMfID(i2cAddress);
  serialNum1 = readSN1(i2cAddress);
  serialNum2 = readSN2(i2cAddress);
  serialNum3 = readSN3(i2cAddress);


    printf("Configuration Register = 0x%X\n", configStat);
    printf("Manufacturer ID = 0x%X\n", mfID);
    printf("Serial Number = %X-%X-%X\n", serialNum1, serialNum2, serialNum3);

    while(true){

        xSemaphoreTake(i2cSem, 1);
      
        //Get current Temperature in C
        temperatureInC = readTemperature(i2cAddress);

        xSemaphoreGive(i2cSem);

        vTaskDelay(10/portTICK_PERIOD_MS);

        xSemaphoreTake(i2cSem, 1);

        //Convert Temperature in C to F
        temperatureInF = (temperatureInC * 1.8) + 32;

        //Get current Humidity
        humidity = readHumidity(i2cAddress);

        xSemaphoreGive(i2cSem);

        vTaskDelay(10/portTICK_PERIOD_MS);

        xSemaphoreTake(i2cSem, 1);

        //printf("Temperature in C: %d\n", temperatureInC);
        //printf("Temperature in F: %d\n", temperatureInF);
        //printf("Humidity %d\n", humidity);

        //Send humidity data to queue and delay for 5 seconds
        xQueueSend(humDisplayQueue, &humidity, 0);
        
        //send temperature data in F and delay for 5 seconds
        xQueueSend(tempDisplayQueue, &temperatureInF, 0);
        vTaskDelay(200/portTICK_PERIOD_MS);
        
        //send sensor data to webpage
        sendSensorReading();

        vTaskDelay(1000/portTICK_PERIOD_MS);

        xSemaphoreGive(i2cSem);


    }
}

void oled_Task(void *pvParameter){

  float lat;
  float prevLat;
  float lng;
  float prevLong;
  float alt;
  float prevAlt;
  char latitude[10];
  char longitude[10];
  char altitude[10];

  oled.init();                      // Initialze SSD1306 OLED display

  while(true){

    xSemaphoreTake(i2cSem, 1);

    if(xQueueReceive(oledLatQueue, &lat, 0) != pdPASS){
    lat = prevLat;
    }
    else{
      prevLat = lat;
    }
    vTaskDelay(10/portTICK_PERIOD_MS);
    
    if(xQueueReceive(oledLngQueue,&lng, 0) != pdPASS){
      lng = prevLong;
    }
    else{
      prevLong = lng;
    }
    vTaskDelay(10/portTICK_PERIOD_MS);

    if(xQueueReceive(oledAltQueue, &alt, 0) != pdPASS){
      alt = prevAlt;
    }
    else{
      prevAlt = alt;
    }
    vTaskDelay(10/portTICK_PERIOD_MS);

    sprintf(latitude, "%.4f", lat);
    sprintf(longitude, "%.4f", lng);
    sprintf(altitude, "%.4f", alt);

    oled.clearDisplay();              // Clear screen
    oled.setTextXY(0,0);              // Set cursor position, start of line 0
    oled.putString("GPS Data: ");
    oled.setTextXY(1,0);              // Set cursor position, start of line 1
    oled.putString("Lat:");
    oled.setTextXY(1,5);
    oled.putString(latitude);
    oled.setTextXY(2,0);              // Set cursor position, start of line 2
    oled.putString("Long:");
    oled.setTextXY(2, 6);
    oled.putString(longitude);
    oled.setTextXY(3,0);             // Set cursor position, line 2 10th character
    oled.putString("Alt");
    oled.setTextXY(3, 6);
    oled.putString(altitude);
    oled.setTextXY(4, 0);
    oled.putString(" ");
    oled.setTextXY(5,0);
    oled.putString("Made By:");
    oled.setTextXY(6,0);
    oled.putString("Justin Harris");


    xSemaphoreGive(i2cSem);
    vTaskDelay(4000/portTICK_PERIOD_MS);
  }
}

//Step Motor Task that controls the how the step motor moves
void step_Motor_Task(void *pvParameters){
    //set up variables for how to move
    int buttonSig;
    int moveTemp = 11;
    int moveHum = 12;
    int stopEE = 13;
    int moveCW = 21;
    int moveCCW = 22;
    int fullMove = 23;
    
    //motor status
    int status;
    int colorDisplay;

    while(true){

        xSemaphoreTake(buttonSem, 1);
        xQueueReceive(smButtonQueue, &buttonSig, 0);

        //ledControl();

        //signals received from button 1
        if(buttonSig == moveTemp){
            status = 994;
            colorDisplay = 4;
            xQueueSend(motorStatQueue, &status, 0);
            //xQueueSend(ledColorQueue, &colorDisplay, 0);
            vTaskDelay(10/portTICK_PERIOD_MS);
            rotateOnTemp();
            xSemaphoreGive(buttonSem);
        }
        else if(buttonSig == moveHum){
            status = 995;
            colorDisplay = 5;
            xQueueSend(motorStatQueue, &status, 0);
            //xQueueSend(ledColorQueue, &colorDisplay, 0);
            vTaskDelay(10/portTICK_PERIOD_MS);
            rotateOnHum();
            xSemaphoreGive(buttonSem);
        }
        else if(buttonSig == stopEE){
            //status = 999;
            //xQueueSend(motorStatQueue, &status, 0);
            vTaskDelay(10/portTICK_PERIOD_MS);
            emergencyStop();

            //resume test rotation
            buttonSig = 23;
            xSemaphoreGive(buttonSem);
        }

        //signals received from Button 2
        else if(buttonSig == moveCW){
            status = 996;
            //colorDisplay = 1;
            xQueueSend(motorStatQueue, &status, 0);
            vTaskDelay(10/portTICK_PERIOD_MS);
            //xQueueSend(ledColorQueue, &colorDisplay, 0);
            vTaskDelay(10/portTICK_PERIOD_MS);
            rotateCW();
            //xSemaphoreGive(buttonSem);
        }
        else if(buttonSig == moveCCW){
            status = 997;
            //colorDisplay = 2;
            xQueueSend(motorStatQueue, &status, 0);
            //xQueueSend(ledColorQueue, &colorDisplay, 0);
            vTaskDelay(10/portTICK_PERIOD_MS);
            rotateCCW();
            //xSemaphoreGive(buttonSem);
        }
        else if(buttonSig == fullMove){
            fullRotateFB();
            xSemaphoreGive(buttonSem);
        }

        //signal received from Button 3
        else if(buttonSig == 33){
            //smStatus(status);
            xSemaphoreGive(buttonSem);

        }
        //if no button signal received, give up semaphore
        else{
            xSemaphoreGive(buttonSem);
        }

        
    }
}

//Task to read the Air530Z GPS
void read_Gps(void *pvParameter){

  TinyGPSPlus gps;
  
  uint8_t month;
  uint8_t day;
  uint16_t year;
  uint8_t hour;
  uint8_t minute;
  uint8_t seconds;
  float lat = 0.000000;
  float lng = 0.000000;
  float alt = 0.000000;

  while(true){

    const char dataFormat[] = "%d-%d-%d %d:%d:%d";
    char gpsTimeData[64];

      while(gps.encode(Serial1.read())){

        month = gps.date.month();
        day = gps.date.day();
        year = gps.date.year();
        hour = gps.time.hour();
        minute = gps.time.minute();
        seconds = gps.time.second();
        lat = gps.location.lat();
        lng = gps.location.lng();
        alt = gps.altitude.meters();

        vTaskDelay(100/portTICK_PERIOD_MS);

        //Printouts to verify data
        Serial.print("GPS Data Lat: "); Serial.println(lat, 6);
        Serial.print("GPS Data Long: "); Serial.println(lng, 6);
        Serial.print("GPS Data Alt: "); Serial.println(alt, 6);
        Serial.print("GPS Data Date: "); 
        Serial.print(month); Serial.print("-"); Serial.print(day); Serial.print("-"); Serial.print(year);
        Serial.println("");
        Serial.print("GPS Data Time: ");
        Serial.print(hour); Serial.print(":"); Serial.print(minute); Serial.print(":"); Serial.print(seconds);
        Serial.println("\n");
        
        sprintf(gpsTimeData, dataFormat, year, month, day, hour, minute, seconds);
        printf("Total GPS Time Data: %s\n", gpsTimeData);

        //Send data to appropriate queues
        xQueueSend(gpsLatQueue, &lat, 0);
        xQueueSend(oledLatQueue, &lat, 0);
        vTaskDelay(10/portTICK_PERIOD_MS);

        xQueueSend(gpsLongQueue, &lng, 0);
        xQueueSend(oledLngQueue, &lng, 0);
        vTaskDelay(10/portTICK_PERIOD_MS);

        xQueueSend(gpsAltQueue, &alt, 0);
        xQueueSend(oledAltQueue, &alt, 0);
        vTaskDelay(10/portTICK_PERIOD_MS);

        xQueueSend(gpsTimeQueue, &gpsTimeData, 0);
        printf("GPS Time data sent: %s\n", gpsTimeData);
        vTaskDelay(10/portTICK_PERIOD_MS);   

        vTaskDelay(4000/portTICK_PERIOD_MS);
      }

  }
}

void led_Color_Task(void *pvParameter){

  while(true){

    ledControl();
    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}

/////////////////////////////FreeRTOS Tasks END///////////////////////////////////

/////////////////////////Wifi and Server Setup API START/////////////////////////////

//Function to establish a wifi connection
void wifi_connect(){

  //set up wifi connection
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_SSID, wifi_PWD);

  printf("Connecting to: [%s]", WiFi.macAddress().c_str());

  while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      vTaskDelay(500/portTICK_PERIOD_MS);
  }
  //print connected IP
  printf("\n Local IP: %s\n", WiFi.localIP().toString().c_str());

}

//Function to start spiffs image, uploaded through platform.io previously. 
void start_spiffs(){

  if(!SPIFFS.begin()){
    printf("Unable to mount SPIFFS volume\n");
  }
  else{
    printf("Spiffs started\n");
  }
}


//Function to start webserver and sync client side
void start_server(){


  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.on("/cw", HTTP_GET, [](AsyncWebServerRequest *request){
    int rotateCW = 21;
    xQueueSend(smButtonQueue, &rotateCW, 0);
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.on("/ccw", HTTP_GET, [](AsyncWebServerRequest *request){
    int rotateCCW = 22;
    xQueueSend(smButtonQueue, &rotateCCW, 0);
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.serveStatic("/", SPIFFS, "/");

  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      printf("Client reconnected\n");
    }
   

    //send test message to client on connect
    client->send("Test!", NULL, millis(), 10000);
  });

  //add handler for events to pass temp / humidity data
  server.addHandler(&events);
  server.begin();

}
/////////////////////////Wifi and Server Setup API END/////////////////////////////

/////////////////////////HDC1080 API START/////////////////////////////////////////

//Function to read and return 1st section of Serial Number from HDC1080
int readSN1(uint8_t address){

  uint16_t sn1 = readHDC1080(address, HDC1080SN1);
  return sn1;
}

//Function to read and return 2nd section of Serial Number from HDC1080
int readSN2(uint8_t address){

  uint16_t sn2 = readHDC1080(address, HDC1080SN2);
  return sn2;
}

//Function to read and return 3rd section of Serial Number from HDC1080
int readSN3(uint8_t address){

  uint16_t sn3 = readHDC1080(address, HDC1080SN3);
  return sn3;
  
}

//Function to read Configuration Register from HDC1080
int readConfig(uint8_t address){

  uint16_t configStat = readHDC1080(address, HDC1080CONFIGREG);
  return configStat;
}

//Function to read Manufacturer ID from HDC1080
int readMfID(uint8_t address){

  uint16_t mfID = readHDC1080(address, HDC1080DEVICEIDREG);
  return mfID;
}

//Function to read and return current temperature from HDC1080 sensor
int readTemperature(uint8_t address){

  uint16_t temperature = readHDC1080(address, HDC1080TEMPREG);

  vTaskDelay(10/portTICK_PERIOD_MS);
  float actualTempC = (temperature / 65536.0) * 165 - 40;

  int fullTemperatureC = round(actualTempC);

  return fullTemperatureC;
}

//Function to read and return current humidity from HDC1080 sensor
int readHumidity(uint8_t address){

  uint16_t humidity = readHDC1080(address, HDC1080HUMREG);

  vTaskDelay(10/portTICK_PERIOD_MS);

  float actualHumidity = (humidity / 65536.0)*100.0;

  int rndHumidity = round(actualHumidity);      

  return rndHumidity;
}

//General purpose function to read data from the HDC1080 Sensor. 
uint16_t readHDC1080(uint8_t baseAddress, uint8_t regAddress){

  Wire.beginTransmission(baseAddress);
  Wire.write(regAddress);
  Wire.endTransmission();

  vTaskDelay(100/portTICK_PERIOD_MS);

  Wire.requestFrom(baseAddress, (uint8_t)2);

  byte b0 = Wire.read();
  byte b1 = Wire.read();

  return b0 << 8 | b1;

}


//Function to send temperature reading to 
//webpage in JSON format.
String sendSensorReading(){

  int temp;
  int hum;
  int motStat;
  int prevStat;
  static int prevTemp;
  static int prevHum;
  char *status;
  float lat;
  float lng;
  float alt;
  float prevLat;
  float prevLong;
  float prevAlt;
  int ledColor = 4;
  unsigned long currentMS;
  unsigned long serverTimer = 50000;

  //const char dataFormat[] = "%d-%d-%d %d:%d:%d";
  char gpsTimeData[64];
  char prevGpsTimeData[64];

  if(xQueueReceive(tempDisplayQueue, &temp, 0) != pdPASS){
    temp = prevTemp;
  }

  vTaskDelay(10/portTICK_PERIOD_MS);

  if(xQueueReceive(humDisplayQueue, &hum, 0) != pdPASS){
    hum = prevHum;
  }

  //temp and hum == these values when error occurs
  //or it reads garbage data
  if((temp == 257 && hum == 100) || temp == 257 || hum == 100){
    temp = prevTemp;
    hum = prevHum;
  }
  else{
    prevTemp = temp;
    prevHum = hum;
  }

  if(xQueueReceive(motorStatQueue, &motStat, 0) != pdPASS){
    motStat = prevStat;
  }
  else{
    prevStat = motStat;
  }

    switch(motStat){

      case 994:
      status = "Rotating on Temperature";
      break;

      case 995:
      status = "Rotating on Humidity";
      break;
      
      case 996:
      status = "Rotating CW";
      break;

      case 997:
      status = "Rotating CCW";
      break;

      case 998:
      status = "Full Rotation CW-CCW";
      break;

      case 999:
      status = "Emergency Stop";
      break;

      default:
      status = "Off";
      break;

    }

  if(xQueueReceive(gpsLatQueue, &lat, 0) != pdPASS){
    lat = prevLat;
  }
  else{
    prevLat = lat;
  }
  vTaskDelay(10/portTICK_PERIOD_MS);
  
  if(xQueueReceive(gpsLongQueue,&lng, 0) != pdPASS){
    lng = prevLong;
  }
  else{
    prevLong = lng;
  }
  vTaskDelay(10/portTICK_PERIOD_MS);

  if(xQueueReceive(gpsAltQueue, &alt, 0) != pdPASS){
    alt = prevAlt;
  }
  else{
    prevAlt = alt;
  }
  vTaskDelay(10/portTICK_PERIOD_MS);

  if(xQueueReceive(gpsTimeQueue, &gpsTimeData, 0) != pdPASS){
    strcpy(gpsTimeData, prevGpsTimeData);
  }
  else{
    strcpy(prevGpsTimeData, gpsTimeData);
  }
  vTaskDelay(10/portTICK_PERIOD_MS);

  printf("GPS Time Received: %s\n", gpsTimeData);

  vTaskDelay(10/portTICK_PERIOD_MS);

  //set JSONvar to send to webpage
  readings["temperature"] = String(temp);
  readings["humidity"] = String(hum);
  readings["motStat"] = String(status);

  String jsonData = JSON.stringify(readings);

  events.send(jsonData.c_str(), "new_reading");
  xQueueSend(oledledColorQueue, &ledColor, 0);
  vTaskDelay(100/portTICK_PERIOD_MS);

  currentMS = millis();

  if(currentMS - startMS >= serverTimer){

  //set JSONvar to send to server
  serverData["auth_code"] = String(authCode);
  serverData["temperature"] = temp;
  serverData["humidity"] = hum;
  serverData["light"] = 0;
  serverData["latitude"] = lat;
  serverData["longitude"] = lng;
  serverData["altitude"] = alt;
  serverData["time"] = String(gpsTimeData);

  vTaskDelay(100/portTICK_PERIOD_MS);

  if(authCode != NULL && gpsTimeData != NULL){
  //send data to server
  rtosSendData(authCode, serverData);
  }

  startMS = currentMS;

  }


  return jsonData;
}
///////////////////////////////////////HDC1080 API End///////////////////////////

////////////////////////////////////rtos_Server_Task API Start///////////////////

void rtosDetectServer(){

  //Set up http client and server url

  HTTPClient http;
  JSONVar serverMessage;
  String serverName = "School Webserver Address";

  http.begin(serverName.c_str());

  //add header for json format
  http.addHeader("Content-Type", "application/json");

  //insert key into serverMessage
  serverMessage["key"] = String("************");

  //convert into JSON format
  String jsonMessage = JSON.stringify(serverMessage);

  //Send to server via POST, get response
  int httpResponse = http.POST(jsonMessage);
  String serverResponse = http.getString();

  //print response from server
  printf("Server Response %d %s\n", httpResponse, serverResponse.c_str());

  //free resource
  http.end();

}

String rtosRegisterServer(){

  //Set up http client and server url
  HTTPClient http;
  JSONVar serverMessage;
  String authCode;
  String serverName = "School Webserver Address";

  http.begin(serverName.c_str());

  //add header for json format
  http.addHeader("Content-Type", "application/json");

  //insert key and IOT ID into serverMessage
  serverMessage["key"] = String("***********");
  serverMessage["iotid"] = String("********");

  //convert into JSON format
  String jsonMessage = JSON.stringify(serverMessage);

  //Send to server via POST, get response
  int httpResponse = http.POST(jsonMessage);
  String serverResponse = http.getString();

  //parse response for auth code and set to String var
  JSONVar jsonParse = JSON.parse(serverResponse);

  authCode = jsonParse["auth_code"];

  //print response from server
  printf("Server Response %d %s\n", httpResponse, serverResponse.c_str());

  //free resource
  http.end();

  return authCode;
}

void rtosQueryCommands(String authCode){

  //Set up http client and server url

  HTTPClient http;
  JSONVar serverMessage;
  String serverName = "School Webserver Address";

  http.begin(serverName.c_str());

  //add header for json format
  http.addHeader("Content-Type", "application/json");

  //insert key into serverMessage
  serverMessage["auth_code"] = String(authCode);
  serverMessage["iotid"] = String("********");

  //convert into JSON format
  String jsonMessage = JSON.stringify(serverMessage);

  printf("jsonMessage: %s\n", jsonMessage.c_str());

  //Send to server via POST, get response
  int httpResponse = http.POST(jsonMessage);
  String serverResponse = http.getString();

  //print response from server
  printf("Server Response %d %s\n", httpResponse, serverResponse.c_str());

  //free resource
  http.end();
}

void rtosSendData(String authCode, JSONVar jData){

    //Set up http client and server url
  int colorDisplay = 3;
  HTTPClient http;
  JSONVar serverMessage;
  String serverName = "School Webserver Address";

  http.begin(serverName.c_str());

  //add header for json format
  http.addHeader("Content-Type", "application/json");

  //convert into JSON format
  String jsonMessage = JSON.stringify(jData);

  printf("jsonMessage: %s\n", jsonMessage.c_str());

  //Send to server via POST, get response
  int httpResponse = http.POST(jsonMessage);
  String serverResponse = http.getString();

  xQueueSend(oledledColorQueue, &colorDisplay, 0);

  //print response from server
  printf("Server Response %d %s\n", httpResponse, serverResponse.c_str());

  //free resource
  http.end();

}

//////////////////////////////STEP MOTOR API START//////////////////////////////////////////////////////

//Function in the Step Motor API to rotate clockwise
void rotateCW(){
    //////////////////////////////////////////////
    //steps for clockwise direction, full step
    // #step    1   2   3   4
    //          --------------
    //
    //pin13     1   1   0   0
    //pin12     0   1   1   0
    //pin9      0   0   1   1
    //pin6      1   0   0   1
    //////////////////////////////////////////////

    int colorDisplay = 1;

    xQueueSend(motledColorQueue, &colorDisplay, 0);


    //step 1
    digitalWrite(StepMotorIN1, HIGH);
    digitalWrite(StepMotorIN2, LOW);
    digitalWrite(StepMotorIN3, LOW);
    digitalWrite(StepMotorIN4, HIGH);
    vTaskDelay(10/portTICK_PERIOD_MS);

    //step 2
    digitalWrite(StepMotorIN1, HIGH);
    digitalWrite(StepMotorIN2, HIGH);
    digitalWrite(StepMotorIN3, LOW);
    digitalWrite(StepMotorIN4, LOW);
    vTaskDelay(10/portTICK_PERIOD_MS);

    //step 3
    digitalWrite(StepMotorIN1, LOW);
    digitalWrite(StepMotorIN2, HIGH);
    digitalWrite(StepMotorIN3, HIGH);
    digitalWrite(StepMotorIN4, LOW);
    vTaskDelay(10/portTICK_PERIOD_MS);

    //step 4
    digitalWrite(StepMotorIN1, LOW);
    digitalWrite(StepMotorIN2, LOW);
    digitalWrite(StepMotorIN3, HIGH);
    digitalWrite(StepMotorIN4, HIGH);
    vTaskDelay(10/portTICK_PERIOD_MS);

}

//Function to move the step motor in counter clockwise direction
void rotateCCW(){
        //////////////////////////////////////////////
    //steps for counter-clockwise direction, full step
    // #step    1   2   3   4
    //          --------------
    //
    //pin13     0   0   1   1
    //pin12     0   1   1   0
    //pin9      1   1   0   0
    //pin6      1   0   0   1
    //////////////////////////////////////////////

    int colorDisplay = 2;
    xQueueSend(motledColorQueue, &colorDisplay, 0);

    //step 1
    digitalWrite(StepMotorIN1, LOW);
    digitalWrite(StepMotorIN2, LOW);
    digitalWrite(StepMotorIN3, HIGH);
    digitalWrite(StepMotorIN4, HIGH);
    vTaskDelay(10/portTICK_PERIOD_MS);

    //step 2
    digitalWrite(StepMotorIN1, LOW);
    digitalWrite(StepMotorIN2, HIGH);
    digitalWrite(StepMotorIN3, HIGH);
    digitalWrite(StepMotorIN4, LOW);
    vTaskDelay(10/portTICK_PERIOD_MS);

    //step 3
    digitalWrite(StepMotorIN1, HIGH);
    digitalWrite(StepMotorIN2, HIGH);
    digitalWrite(StepMotorIN3, LOW);
    digitalWrite(StepMotorIN4, LOW);
    vTaskDelay(10/portTICK_PERIOD_MS);

    //step 4
    digitalWrite(StepMotorIN1, HIGH);
    digitalWrite(StepMotorIN2, LOW);
    digitalWrite(StepMotorIN3, LOW);
    digitalWrite(StepMotorIN4, HIGH);
    vTaskDelay(10/portTICK_PERIOD_MS);
}

//Function to rotate 1 the step motor one full revolution clockwise
//and then 1 full revolution counter clockwise
void fullRotateFB(){

    int max_steps = 500;
    int status;
    int clearQueue;
    int colorDisplay = 2;

    //clear previous status
    //xQueueReceive(motorStatQueue, &clearQueue, 0);
    //xQueueReceive(ledColorQueue, &clearQueue, 0);
    vTaskDelay(10/portTICK_PERIOD_MS);

    status = 998;
    xQueueSend(motorStatQueue, &status, 0);
    //xQueueSend(ledColorQueue, &colorDisplay, 0);
    vTaskDelay(200/portTICK_PERIOD_MS);

    //one full rotation clockwise
    for(int i = 0; i <= max_steps; i++){
        xQueueSend(motorStatQueue, &status, 0);
        rotateCW();
    }

    //one full rotation counter-clockwise
    for(int i = 0; i <= max_steps; i++){
        xQueueSend(motorStatQueue, &status, 0);
        rotateCCW();
    }

    //delay for 10 seconds
    //vTaskDelay(1000/portTICK_PERIOD_MS);

}

//Function that rotates on changes in temperature
void rotateOnTemp(){
    int currentTemp;
    int tempCheck;
    static int prevTemp;
    int numSteps = 0;

    //look at mainControlQueue and get a copy of the current value
    if(xQueuePeek(tempDisplayQueue, &tempCheck, 0)){

        //ignore motor status numbers
        if(tempCheck < 992){
            currentTemp = tempCheck;
        }
        else{
            currentTemp = prevTemp;
        }

        //check if previous temp or current temp is larger
        //current Temp is larger, move clockswise for x steps
        if(currentTemp > prevTemp){
            numSteps = currentTemp - prevTemp;
            //printf("number of steps: %d\n", numSteps);

            prevTemp = currentTemp;

            for(int i = 0; i < numSteps; i++){
                rotateCW();
            }
        }

        //if temp decreasing, rotate ccw for x steps
        else if(currentTemp < prevTemp){
            numSteps = prevTemp - currentTemp;

            prevTemp = currentTemp;

            for(int i = 0; i < numSteps; i++){
                rotateCCW();
            }
        }
        //if no change, delay
        else{
            //xSemaphoreGive(buttonSem);
            vTaskDelay(2000/portTICK_PERIOD_MS);
            //xSemaphoreTake(buttonSem, 0);
        }
    }

}

//Function that rotates motor on changes in humidity
void rotateOnHum(){
    int currentHum;
    int humCheck;
    static int prevHum;
    int numSteps = 0;

    //look at mainControlQueue and get a copy of the current value
    if(xQueuePeek(humDisplayQueue, &humCheck, 0)){

        //ignore motor status numbers
        if(humCheck < 992){
            currentHum = humCheck;
        }
        else{
            currentHum = prevHum;
        }

        //check if previous temp or current temp is larger
        //current Temp is larger, move clockswise for x steps
        if(currentHum > prevHum){
            numSteps = currentHum - prevHum;
            printf("number of steps: %d\n", numSteps);

            prevHum = currentHum;

            for(int i = 0; i < numSteps; i++){
                rotateCW();
            }
        }

        //if temp decreasing, rotate ccw for x steps
        else if(currentHum < prevHum){
            numSteps = prevHum - currentHum;

            prevHum = currentHum;

            for(int i = 0; i < numSteps; i++){
                rotateCCW();
            }
        }
        //if no change, delay
        else{
            digitalWrite(StepMotorIN1, LOW);
            digitalWrite(StepMotorIN2, LOW);
            digitalWrite(StepMotorIN3, LOW);
            digitalWrite(StepMotorIN4, LOW);
            //xSemaphoreGive(buttonSem);
            vTaskDelay(2000/portTICK_PERIOD_MS);
            //xSemaphoreTake(buttonSem, 0);
        }
    }
}

//Function to stop motor, display EE to 7 seg
void emergencyStop(){

    //intitialize variables
    int stop = 999;
    int clearQueue;
    int status;
    int colorDisplay = 3;

    //send signal to motor Status Queue
    //xQueueSend(motorStatQueue, &stop, 0);
    //xQueueSend(ledColorQueue, &colorDisplay, 0);

    //stop motor for 5 seconds
    digitalWrite(StepMotorIN1, LOW);
    digitalWrite(StepMotorIN2, LOW);
    digitalWrite(StepMotorIN3, LOW);
    digitalWrite(StepMotorIN4, LOW);

    vTaskDelay(5000/portTICK_PERIOD_MS);

    //resume readHDC1080 task
    //vTaskResume(hdc1080);

    //clear message from queue
    //xQueueReceive(mainControlQueue, &clearQueue, 0);
    vTaskDelay(10/portTICK_PERIOD_MS);

    //send test status
    status = 998;
    xQueueSend(motorStatQueue, &status, 0);
    vTaskDelay(200/portTICK_PERIOD_MS);
    
}

//////////////////////////////STEP MOTOR API END//////////////////////////////////////////////////////

//Task to run button function
void buttons_Task(void *pvParameters){
    while(true){

        //take semaphore, run get buttons, release semaphore
       // xSemaphoreTake(buttonSem, 1);
        getButtons();
        vTaskDelay(10/portTICK_PERIOD_MS);
       // xSemaphoreGive(buttonSem);
    }
}

//////////////////////////////BUTTONS API START//////////////////////////////////////////////////////
void getButtons(){

    //set up variables to track which buttons are pressed
    bool button1 = false;
    bool button2 = false;
    bool button3 = false;
    bool b1WasPressed = false;
    bool b2WasPressed = false;
    bool b3WasPressed = false;

    //set up counters for button press totals and timer
    int button1Total = 0;
    int button2Total = 0;
    int button3Total = 0;
    int buttonReturn = 0;
    int buttonTimer = 0;
    int buttonTimerMax = 200;
    int bSend = 0;

    while(buttonTimer < buttonTimerMax){
        
        button1 = digitalRead(BUTTON1);
        button2 = digitalRead(BUTTON2);
        button3 = digitalRead(BUTTON3);

        //if button 1 is pressed, and not logged increment button 1 total and set
        //b1WasPressed to true to log press
        if(button1 == true && b1WasPressed == false){
            button1Total += 1;
            b1WasPressed = true;
        }
        //if button is not pressed, set log to false
        else if(button1 == false){
            b1WasPressed = false;
        }

        //if button 1 is pressed, and not logged increment button 1 total and set
        //b1WasPressed to true to log press
        if(button2 == true && b2WasPressed == false){
            button2Total += 1;
            b2WasPressed = true;
        }
        //if button is not pressed, set log to false
        else if(button2 == false){
            b2WasPressed = false;
        }

        //if button 1 is pressed, and not logged increment button 1 total and set
        //b1WasPressed to true to log press
        if(button3 == true && b3WasPressed == false){
            button3Total += 1;
            b3WasPressed = true;
        }
        //if button is not pressed, set log to false
        else if(button3 == false){
            b3WasPressed = false;
        }
        

            //delay and increment timing counter
            vTaskDelay(10/portTICK_PERIOD_MS);
            buttonTimer++;
    }

    //Error check, only accept 1 button being pressed at at time
    //button 1 baseline
    if(button1Total > 0 && ((button2Total > 0 && button3Total > 0) || ((button2Total > 0) || (button3Total > 0)))){
        printf("Error: only press 1 button at a time\n");
        button1Total = 0;
    }
    //button 2 baseline
    else if(button2Total > 0 && ((button1Total > 0 && button3Total > 0) || ((button1Total > 0) || (button3Total > 0)))){
        printf("Error: only press 1 button at a time\n");
        button2Total = 0;
    }
    //button 3 baseline
    else if(button3Total > 0 && ((button2Total > 0 && button1Total > 0) || ((button2Total > 0) || (button1Total > 0)))){
        printf("Error: only press 1 button at a time\n");
        button3Total = 0;
    }
    //don't accept any values over 3
    else if(button1Total > 3 || button2Total > 3 || button3Total > 3){
        printf("Error: button presses must be < 3 in 2 seconds\n");
        buttonReturn = 0;
    }
    //valid button 1 pressed
    else if(button1Total > 0){
        printf("button1 pressed %d times\n", button1Total);
        //move on temp
        if(button1Total == 1){
            bSend = 11;
            xQueueSend(smButtonQueue, &bSend, 0);
        }
        //move on humidity
        else if(button1Total == 2){
            bSend = 12;
            xQueueSend(smButtonQueue, &bSend, 0);
        }
        //emergency stop
        else if(button1Total == 3){
            bSend = 13;
            xQueueSend(smButtonQueue, &bSend, 0);
        }    
    }
    //valid button 2 pressed
    else if(button2Total > 0){
        printf("button2 pressed %d times\n", button2Total);
        
        //move motor CW
        if(button2Total == 1){
            bSend = 21;
            xQueueSend(smButtonQueue, &bSend, 0);
        }
        //move motor CCW
        else if(button2Total == 2){
            bSend = 22;
            xQueueSend(smButtonQueue, &bSend, 0);
        }
        //full CC CCW repeat
        else if(button2Total == 3){
            bSend = 23;
            xQueueSend(smButtonQueue, &bSend, 0);
        }
    }
    //valid button 3 pressed
    else if(button3Total > 0){
        printf("button3 pressed %d times\n", button3Total);
        
        //display temperature
        if(button3Total == 1){
            bSend = 31;
            //xQueueSend(sevSegDisQueue, &bSend, 0);
        }
        //display humidity
        else if(button3Total == 2){
            bSend = 32;
            //xQueueSend(sevSegDisQueue, &bSend, 0);
        }
        //display step motor status
        else if(button3Total == 3){
            bSend = 33;
            //xQueueSend(sevSegDisQueue, &bSend, 0);
        }
    }

}
//////////////////////////////BUTTONS API END//////////////////////////////////////////////////////

//Function to control on-board LEDS
void ledControl(){

  int motledColor;
  int oledledColor;
  
  strand_t * pStrand = &STRANDS[0];

  //ledTest(pStrand, 4, 2000);

  //BC24ThreeBlink(BrightGreen, 100);

  xQueueReceive(motledColorQueue, &motledColor, 0);
  xQueueReceive(oledledColorQueue, &oledledColor, 0);

  //Set color depending on which direction the motor is moving
  if(motledColor == 1 || motledColor == 2){

    if(motledColor == 1){

      displayTwoPixels(0, 1, BrightGreen);
    }
    else if(motledColor == 2){

      displayTwoPixels(0, 1, BrightCyan);
    }
    else{
      displayTwoPixels(0, 1, Black);
    }
  }

  //Blink when data is transmitted to the server
  if(oledledColor == 3){

    displaySinglePixelNC(2, DarkPurple);

    vTaskDelay(100/portTICK_PERIOD_MS);

    displaySinglePixelNC(2, Black);
  }

  //Blink when data is sent to the ESP32 webserver
  if(oledledColor == 4){

    displaySinglePixelNC(3, BrightYellow);

    vTaskDelay(100/portTICK_PERIOD_MS);

    displaySinglePixelNC(3, Black);
  }



  vTaskDelay(10/portTICK_PERIOD_MS);

}
