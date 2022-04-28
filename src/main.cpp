#include <Arduino.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <FreeRTOS.h>
#include <Arduino_JSON.h>
#include <AsyncTCP.h>

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

//Declare Queues
QueueHandle_t tempDisplayQueue;
QueueHandle_t humDisplayQueue;

//Wifi connection credentials for my network
const char *wifi_SSID = "********";
const char *wifi_PWD = "*********";

//Declare webserver
AsyncWebServer server(80);

//Event handler to update webpage with temperature and humidity values
AsyncEventSource events ("/events");

JSONVar readings;

//FreeRTOS Tasks
void webserver_Setup_Task(void *pvParameter);
void HDC1080_Control_Task(void *pvParameter);

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

static int task_core = 1;

void setup() {

  //Start Wire for reading I2C
  Wire.begin();

  //Start serial communication to print into to console
  Serial.begin(115200);

  //Create Queues
  tempDisplayQueue = xQueueCreate(2, sizeof(int));
  humDisplayQueue = xQueueCreate(2, sizeof(int));

  //Create Task for wifi set up and connection, pin to core arduino is running on
  xTaskCreatePinnedToCore(webserver_Setup_Task, "webserver_Setup", 5000, NULL, 1, NULL, CONFIG_ARDUINO_RUNNING_CORE);

  //Create Task for reading from HDC1080 Sensor, pin to core 0. 
  xTaskCreatePinnedToCore(HDC1080_Control_Task, "HDC1080_Control_Task", 5000, NULL, 1, NULL, task_core);

  //start scheduler
  //vTaskStartScheduler();

}

void loop() {
  // put your main code here, to run repeatedly:
  //vTaskDelay(portMAX_DELAY);
  //vTaskDelete(NULL);
}


/////////////////////////////FreeRTOS Tasks START///////////////////////////////////

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
      
        //Get current Temperature in C
        temperatureInC = readTemperature(i2cAddress);

        vTaskDelay(10/portTICK_PERIOD_MS);

        //Convert Temperature in C to F
        temperatureInF = (temperatureInC * 1.8) + 32;

        //Get current Humidity
        humidity = readHumidity(i2cAddress);

        vTaskDelay(10/portTICK_PERIOD_MS);

        printf("Temperature in C: %d\n", temperatureInC);
        printf("Temperature in F: %d\n", temperatureInF);
        printf("Humidity %d\n", humidity);

        //Send humidity data to queue and delay for 5 seconds
        xQueueSend(humDisplayQueue, &humidity, 0);
        
        //send temperature data in F and delay for 5 seconds
        xQueueSend(tempDisplayQueue, &temperatureInF, 0);
        vTaskDelay(100/portTICK_PERIOD_MS);
        
        //send sensor data to webpage
        sendSensorReading();

        vTaskDelay(1000/portTICK_PERIOD_MS);


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
  static int prevTemp = 0;
  static int prevHum = 0;

  if(xQueueReceive(tempDisplayQueue, &temp, 0) != pdPASS){
    temp = prevTemp;
  }
  else{
    prevTemp = temp;
  }
  vTaskDelay(10/portTICK_PERIOD_MS);

  if(xQueueReceive(humDisplayQueue, &hum, 0) != pdPASS){
    hum = prevHum;
  }
  else{
    prevHum = hum;
  }
  vTaskDelay(10/portTICK_PERIOD_MS);

  readings["temperature"] = String(temp);
  readings["humidity"] = String(hum);

  String jsonData = JSON.stringify(readings);

  events.send(jsonData.c_str(), "new_reading");
  vTaskDelay(100/portTICK_PERIOD_MS);

  return jsonData;
}
