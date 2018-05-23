#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP8266WiFi.h>
//#include <ESP8266WiFiMulti.h>

//#define USE_NTP  // Get current time from NTP server
//#define USE_OTA
#define USE_WEBSOCKETS
//#define USE_EXT_LOG
//#include <ESP8266HTTPClient.h>

#ifdef USE_NTP
#include <WiFiUdp.h>
#endif
#ifdef USE_OTA
#include <ArduinoOTA.h>
#endif
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#ifdef USE_WEBSOCKETS
#include <WebSocketsServer.h>
#endif
#include <FS.h>

#include <Wire.h>
#include <MechaQMC5883.h>

#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson

#include "localconfig.h" // not included in git, contains passwords etc.

#define ONE_HOUR 3600000UL // one hour in milliseconds

// Wemos D1 R2 pinout
// TX 	TXD 	TXD
// RX 	RXD 	RXD
// A0 	Analog input 	A0
// D0 	I/O 	GPIO16
// D1 	I/O, SCL 	GPIO5
// D2 	I/O, SDA 	GPIO4
// D3 	I/O, 10k pull-up 	GPIO0
// D4 	I/O, 10k pull-up, BUILTIN_LED 	GPIO2
// D5 	I/O, SCK 	GPIO14
// D6 	I/O, MISO 	GPIO12
// D7 	I/O, MOSI 	GPIO13
// D8 	I/O, 10k pull-down, SS 	GPIO15
// GND 	Ground 	GND
// 5V 	5V 	
// 3V3 	3.3V 	3.3V
// RST 	Reset 	RST

// Pin connection definitions
// D1 and D2 used by I2C
#define TEMP_SENSOR_PIN D3  // D3 has 10k pull-up
#define LED_PIN D4 // D4 has 10k pull-up and builtin LED
#define RPWM_PIN D5
#define LPWM_PIN D6
#define MOTOR_RELAY_PIN D0  // D0 is also used for boot
#define HEATER_RELAY_PIN D8  // D8 has 10k pull-down, is used for boot
#define SEAT_SENSOR_PIN D7

OneWire oneWire(TEMP_SENSOR_PIN);        // Set up a OneWire instance to communicate with OneWire devices
DallasTemperature tempSensors(&oneWire); // Create an instance of the temperature sensor class

MechaQMC5883 qmc;

ESP8266WebServer server(80);       // create a web server on port 80
#ifdef USE_WEBSOCKETS
WebSocketsServer webSocket = WebSocketsServer(81);    // create a websocket server on port 81
#endif
ESP8266HTTPUpdateServer httpUpdater;

File fsUploadFile;                                    // a File variable to temporarily store the received file

//ESP8266WiFiMulti wifiMulti;    // Create an instance of the ESP8266WiFiMulti class, called 'wifiMulti'

#ifdef USE_OTA
const char *OTAName = "ESP8266";         // A name and a password for the OTA service
const char *OTAPassword = "put-your-password-here";
#endif

const char* mdnsName = "composttoilet";        // Domain name for the mDNS responder

const char *ssid = "Composttoilet"; // The name of the Wi-Fi network that will be created
const char *password = PASSWORD;   // The password required to connect to it, leave blank for an open network

#ifdef USE_EXT_LOG
const char *logHost = LOGHOST;
//WiFiClient client;
#endif

// 0b00001111
// 0b01101111
// 0b00000001
// 0b01001001

struct Programma {
  int w;
  int y;
  int z;
  int x;
  int y2;
  int z2;
  int x2;
  int t;
};

// Configuration that we'll store on disk
struct Config {
  char ssid[32];
  char password[32];
  int vakantieTijd;
  int logInterval;
  Programma programma[5];
};

//const char *filename = "/config.txt";  // <- SD library uses 8.3 filenames
Config config;                         // <- global configuration object

enum ProgramMode {DIRECT, PROGRAM, MANUAL, STALL, PAUSE, VAKANTIE};
const String programModeTxt[6] = {"direct","programma ","manual","stall","pause","vakantie"};
ProgramMode programMode = PROGRAM;

#ifdef USE_NTP
WiFiUDP UDP;                   // Create an instance of the WiFiUDP class to send and receive UDP messages

IPAddress timeServerIP;        // The time.nist.gov NTP server's IP address
const char* ntpServerName = "time.nist.gov";

const int NTP_PACKET_SIZE = 48;          // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[NTP_PACKET_SIZE];      // A buffer to hold incoming and outgoing packets
#endif
/*__________________________________________________________SETUP__________________________________________________________*/

void setup() {
  Serial.begin(115200);        // Start the Serial communication to send messages to the computer
  delay(10);
  Serial.println("\r\n");
  
  Wire.begin();
  Wire.setClock(50000L);
  qmc.init();
  //qmc.setMode(Mode_Continuous,ODR_200Hz,RNG_2G,OSR_256);

  tempSensors.setWaitForConversion(false); // Don't block the program while the temperature sensor is reading
  tempSensors.begin();                     // Start the temperature sensor

  if (tempSensors.getDeviceCount() == 0) {
    Serial.printf("No DS18x20 temperature sensor found on pin %d.\r\n", TEMP_SENSOR_PIN);
    Serial.flush();
    //ESP.reset();
  }
  
  pinMode(LED_PIN, OUTPUT);
  
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  digitalWrite(RPWM_PIN, LOW);
  digitalWrite(LPWM_PIN, LOW);
  
  pinMode(SEAT_SENSOR_PIN, INPUT_PULLUP);
  pinMode(HEATER_RELAY_PIN, OUTPUT);
  pinMode(MOTOR_RELAY_PIN, OUTPUT);
  
  startSPIFFS();               // Start the SPIFFS and list all contents
  
  loadConfiguration();

  startWiFi();                 // Start a Wi-Fi access point, and try to connect to some given access points. Then wait for either an AP or STA connection
  digitalWrite(LED_PIN, 1);

#ifdef USE_OTA
  startOTA();                  // Start the OTA service
#endif

#ifdef USE_WEBSOCKETS
  startWebSocket();            // Start a WebSocket server
#endif

  startMDNS();                 // Start the mDNS responder
  
  httpUpdater.setup(&server);
  startServer();               // Start a HTTP server with a file read handler and an upload handler

#ifdef USE_NTP
  startUDP();                  // Start listening for UDP messages to port 123

  if(WiFi.hostByName(ntpServerName, timeServerIP)){ // Get the IP address of the NTP server
    // timeUNIX = 1;
  }
  Serial.print("Time server IP:\t");
  Serial.println(timeServerIP);

  sendNTPpacket(timeServerIP);
  delay(500);
#endif
}

/*__________________________________________________________LOOP__________________________________________________________*/
#ifdef USE_NTP
const unsigned long intervalNTP = ONE_HOUR; // Update the time every hour
unsigned long prevNTP = 0;
unsigned long lastNTPResponse = millis();
#endif

int speed;
int speedMin, speedMax;
float rotation;
const unsigned long intervalRotation = 100;   // rotation measurement interval
unsigned long prevRotation = 0;
unsigned long lastRotation = 0;

float speedControlP = 2.0; // 1023 = 24V, 360°/4 = 100°/s
int programSpeed = 0;
int programDirection = 1;
int motorPower = 0;
float weerstand;
float weerstandAvg = 0.0;
int weerstandCount = 0;
unsigned long programStartTime = 0;
int currentProgram = 0;
int programState = 0;

unsigned long stallTime = 0;
const int stallTimeTreshold = 10000;

float temp;
const unsigned long intervalTemp = 1000;   // temperature measurement interval
unsigned long prevTemp = 0;
bool tmpRequested = false;
const unsigned long DS_delay = 750;         // Reading the temperature from the DS18x20 can take up to 750ms
const float maxTempError = 1.0;
int heater = 0;
unsigned long heaterStart = 0;
float heaterAvg = 0.0;

int brilOpen = 0;
unsigned long lastBrilChange = 0;
int programTemperature = 0;

unsigned long lastLogTime = 0;

int ledPattern = 0b00001111;
int ledStep = 0;
int intervalLed = 250;
unsigned long prevLed = 0;

uint32_t timeUNIX = 0;                      // The most recent timestamp received from the time server

int errCount = 0;

void loop() {
  unsigned long currentMillis = millis();

#ifdef USE_NTP
  if (currentMillis - prevNTP > intervalNTP) { // Request the time from the time server every hour
    prevNTP = currentMillis;
    sendNTPpacket(timeServerIP);
  }

  uint32_t time = getTime();                   // Check if the time server has responded, if so, get the UNIX time
  if (time) {
    timeUNIX = time;
    Serial.print("NTP response:\t");
    Serial.println(timeUNIX);
    lastNTPResponse = millis();
  } else if ((millis() - lastNTPResponse) > 24UL * ONE_HOUR) {
    Serial.println("More than 24 hours since last NTP response. Rebooting.");
    Serial.flush();
    ESP.reset();
  }
#endif

  // Measure rotation
  if (currentMillis - prevRotation > intervalRotation) {
    int32_t x, y, z, mag;//, azimuth;
    float azimuth; //is supporting float too
    int err = qmc.read(&x, &y, &z, &azimuth);
    //Serial.printf("x: %d, y: %d, z: %d\r\n", x, y, z);
    if (err) {
      errCount++;
      //Serial.println("rotation sensor error " + String(err));
      webSocket.broadcastTXT("x:rotation sensor error " + String(err) + " (" + String(errCount) + "x)");
      
    } else { 
      errCount = 0;
      //azimuth = qmc.azimuth(&y,&x);//you can get custom azimuth
      mag = (x/512)*(x/512) + (y/512)*(y/512) + (z/512)*(z/512); // max magnitude 49152
      speed = (float)((int)(1000 * (rotation - azimuth + 360 + 180))%360000 - 180000) / (currentMillis - lastRotation);
      if (speed < speedMin) speedMin = speed;
      if (speed > speedMax) speedMax = speed;
      // Serial.print("mag: ");
      // Serial.print(mag);
      // Serial.print(" a: ");
      // Serial.print(azimuth);
      // Serial.print(" speed: ");
      // Serial.print(speed);
      // Serial.println();
      rotation = azimuth;
      weerstand = speed ? (motorPower / speed) : motorPower;
      lastRotation = currentMillis;
    }
    if (programMode == PROGRAM) {
      Programma* p = &config.programma[currentProgram];
      if (programState == 1) {
        weerstandAvg += weerstand;
        weerstandCount++;
      }
      if (programState == 4 && (currentMillis - programStartTime) > 1000 * p->y2) {
        // Restart program
        programState = 0;
      }
      if (programState == 0) {
        // Turn right with speed z for x seconds
        programState = 1;
        programStartTime = currentMillis;
        digitalWrite(MOTOR_RELAY_PIN, HIGH);
        programSpeed = programDirection * p->z;
        weerstandAvg = 0.0;
        weerstandCount = 0;
      } else if (programState == 1 && (currentMillis - programStartTime) > 1000 * p->x) {
        // Pause
        programState = 2;
        programStartTime = currentMillis;
        digitalWrite(MOTOR_RELAY_PIN, LOW);
        motorPower = 0;
        programSpeed = 0;
        if (weerstandCount) {weerstandAvg /= weerstandCount;}
        
        // for (int n=0; n<5; n++) {
        //   if (weerstandAvg < config.programma[n].w) {
        //     programma = n;
        //     motorPower = config.programma[n].x;
        //     setMotor('R', motorPower);
        //     break;
        //   }
        // }
      } else if (programState == 2 && (currentMillis - programStartTime) > 1000 * p->y) {
        // Turn left
        programState = 3;
        programStartTime = currentMillis;
        digitalWrite(MOTOR_RELAY_PIN, HIGH);
        programSpeed = programDirection * -p->z2;
      } else if (programState == 3 && (currentMillis - programStartTime) > 1000 * p->x2) {
        // Pause
        programState = 4;
        programStartTime = currentMillis;
        digitalWrite(MOTOR_RELAY_PIN, LOW);
        motorPower = 0;
        programSpeed = 0;
        // Go into holiday mode
        if ((currentMillis - lastBrilChange) > ONE_HOUR * config.vakantieTijd) {
          //programMode = VAKANTIE;
          currentProgram = 4;
        }
      }
      
      // Motor speed feedback loop
      if (!err) {
        float speedError = programSpeed - speed;
        motorPower = motorPower + speedControlP * speedError;
        #define MAXPOWER (8*1023/24)
        if (abs(motorPower) > MAXPOWER) {
          motorPower = motorPower > 0 ? MAXPOWER : -MAXPOWER;
          // Detect motor stall
          if (abs(speed) < 1.0) {
            if (!stallTime) {
              stallTime = currentMillis;
            } else if (stallTime + stallTimeTreshold > currentMillis) {
              stallTime = 0;
              programMode = STALL;
              motorPower = 0;
              digitalWrite(MOTOR_RELAY_PIN, LOW);
            }
          } else {
            stallTime = 0;
          }
        }
      } else if (errCount > 10) { // if sensor doesn't work fallback to voltage based control
        motorPower = programSpeed * 1023 / 24;
      }
      setMotor(motorPower);
    }
    if (programMode == STALL) {
      if (abs(speed) > 5.0) {
        programMode = MANUAL;
      }
    }
    if (programMode == MANUAL) {
      if (abs(speed) < 1.0) { // TODO: also require 90 deg rotation to continue
        programMode = PROGRAM;
        programState = 0;
      }
    }
#ifdef USE_WEBSOCKETS
    String p = programModeTxt[programMode];
    if (programMode == PROGRAM) {
      p = p + " " + String(currentProgram+1) + "-" + String(programState);
    }
    webSocket.broadcastTXT("r:" + String(rotation,1) + ",s:" + String(speed)
                           + ",w:" + String(weerstand) + ",u:" + String((24.0/1023.0) * motorPower,1)
                           + ",v:" + String(heater) + ",p:" + p);
#endif
    
    
    int brilSensor = !digitalRead(SEAT_SENSOR_PIN);
    if (brilOpen != brilSensor) {
      lastBrilChange = currentMillis;
      brilOpen = brilSensor;
      if (brilOpen) {
        programMode = PAUSE;
        motorPower = 0;
        digitalWrite(MOTOR_RELAY_PIN, LOW);
        setMotor(0);
      } else {
        programMode = PROGRAM;
        currentProgram = 0;
        programState = 0;
        programDirection = -programDirection;
      }
      lastLogTime = 0; // 
#ifdef USE_WEBSOCKETS
      if (brilOpen) {
        webSocket.broadcastTXT("b:open");
      } else {
        webSocket.broadcastTXT("b:dicht");
      }
#endif
    }

    prevRotation = currentMillis;
  } // rotation
  
  // Measure temperature
  if (currentMillis - prevTemp > intervalTemp) {  // Every minute, request the temperature
    tempSensors.requestTemperatures(); // Request the temperature from the sensor (it takes some time to read it)
    tmpRequested = true;
    prevTemp = currentMillis;
    // Serial.println("Temperature requested");
  }
  if (currentMillis - prevTemp > DS_delay && tmpRequested) { // 750 ms after requesting the temperature
    tmpRequested = false;
    float temp = tempSensors.getTempCByIndex(0); // Get the temperature from the sensor
    temp = round(temp * 10.0) / 10.0; // round temperature to 1 digits
    
    if (!heater && temp < config.programma[currentProgram].t - maxTempError) {
      // turn heater on
      heater = 1;
      digitalWrite(HEATER_RELAY_PIN, 1);
      heaterStart = currentMillis;
    } else if (heater && temp > config.programma[currentProgram].t + maxTempError) {
      // turn heater off
      heater = 0;
      digitalWrite(HEATER_RELAY_PIN, 0);
      heaterAvg += currentMillis - heaterStart;
    }
    
#ifdef USE_WEBSOCKETS
    webSocket.broadcastTXT("t:" + String(temp,1));
#endif
#ifdef USE_NTP
    if (timeUNIX != 0) {
      uint32_t actualTime = timeUNIX + (currentMillis - lastNTPResponse) / 1000;
      // The actual time is the last NTP time plus the time that has elapsed since the last NTP response
      Serial.printf("Appending temperature to file: %lu,", actualTime);
      Serial.println(temp);
      File tempLog = SPIFFS.open("/log.csv", "a"); // Write the time and the temperature to the csv file
      tempLog.print(actualTime);
      tempLog.print(',');
      tempLog.println(temp);
      tempLog.close();
    } else {                                    // If we didn't receive an NTP response yet, send another request
      sendNTPpacket(timeServerIP);
      delay(500);
    }
#endif
  }
  

#ifdef USE_EXT_LOG
  if (currentMillis - lastLogTime > 1000 * config.logInterval) {
    Serial.print("Sending data to ");
    Serial.println(logHost);
    unsigned long tmp = millis();
    if (client.connect(logHost, 80)) {
      if (heater) {heaterAvg += currentMillis - heaterStart;}
      heaterAvg /= currentMillis - lastLogTime;
      String data = String("temp=") + String(temp, 1);
      data += "&speed_min=" + String(speedMin);
      data += "&speed_max=" + String(speedMax);
      data += "&volt=" + String((1023.0/24.0) * motorPower, 1);
      data += "&bril=" + String(brilOpen);
      data += "&heater=" + String(100 * heaterAvg, 0);
      Serial.print("Sending data: ");
      Serial.println(data);
      
      // HTTPClient http;  //Declare an object of class HTTPClient
      // http.begin(String(LOGURL) + data);  //Specify request destination
      // int httpCode = http.GET(); 
      
      client.print(String("GET /") + " HTTP/1.1\r\n" +
                   "Host: " + logHost + "\r\n" +
                   "Connection: close\r\n\r\n");
      speedMin = speed;
      speedMax = speed;
      heaterAvg = 0;
      heaterStart = currentMillis;
    } else {
      Serial.println("Connection failed!");
    }
    Serial.println(millis() - tmp);
    lastLogTime = currentMillis;
  }
  Read all the lines of the reply from server and print them to Serial
  while (client.available()) { // client.connected() && 
    String line = client.readStringUntil('\r');
    Serial.print(line);
  }
#endif
  
  // LED blinker
  if (currentMillis - prevLed > intervalLed) {
    ledStep = (ledStep + 1)%8;
    digitalWrite(LED_PIN, !((ledPattern >> ledStep) & 0x01));
    prevLed = currentMillis;
  }
  
  server.handleClient();                      // run the server
#ifdef USE_WEBSOCKETS
  webSocket.loop();                           // constantly check for websocket events
#endif
#ifdef USE_OTA
  ArduinoOTA.handle();                        // listen for OTA events
#endif
}

/*__________________________________________________________SETUP_FUNCTIONS__________________________________________________________*/

void startWiFi() { // Try to connect to some given access points. Then wait for a connection
  WiFi.softAP(ssid, password);
  Serial.println ("Access Point created");
  WiFi.begin(config.ssid, config.password); 
  Serial.print("Connecting to ");
  Serial.print(config.ssid);
  int n = 0;
  while (WiFi.status() != WL_CONNECTED) {  // Wait for the Wi-Fi to connect
    delay(250);
    n += 250;
    if (n > 10000) { // | !digitalRead(SEAT_SENSOR_PIN)) {
      Serial.println("Connection failed");
      ledPattern = 0b01101111;
      break;
    }
    Serial.print('.');
  }
  
  Serial.println("\r\n");
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID());             // Tell us what network we're connected to
  Serial.print("IP address:\t");
  Serial.print(WiFi.localIP());            // Send the IP address of the ESP8266 to the computer
  Serial.println("\r\n");
}

#ifdef USE_NTP
void startUDP() {
  Serial.println("Starting UDP");
  UDP.begin(123);                          // Start listening for UDP messages to port 123
  Serial.print("Local port:\t");
  Serial.println(UDP.localPort());
}
#endif

#ifdef USE_OTA
void startOTA() { // Start the OTA service
  ArduinoOTA.setHostname(OTAName);
  ArduinoOTA.setPassword(OTAPassword);

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\r\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("OTA ready\r\n");
}
#endif

void startSPIFFS() { // Start the SPIFFS and list all contents
  SPIFFS.begin();                             // Start the SPI Flash File System (SPIFFS)
  Serial.println("SPIFFS started. Contents:");
  {
    Dir dir = SPIFFS.openDir("/");
    while (dir.next()) {                      // List the file system contents
      String fileName = dir.fileName();
      size_t fileSize = dir.fileSize();
      Serial.printf("\tFS File: %s, size: %s\r\n", fileName.c_str(), formatBytes(fileSize).c_str());
    }
    Serial.printf("\n");
  }
}

void startMDNS() { // Start the mDNS responder
  MDNS.begin(mdnsName);                        // start the multicast domain name server
  Serial.print("mDNS responder started: http://");
  Serial.print(mdnsName);
  Serial.println(".local");
}

void startServer() { // Start a HTTP server with a file read handler and an upload handler
  server.on("/edit.html",  HTTP_POST, []() {  // If a POST request is sent to the /edit.html address,
    server.send(200, "text/plain", "");
  }, handleFileUpload);                       // go to 'handleFileUpload'
  
  server.on("/version", []() {
    server.send(200, "text/plain", "\"" SRC_REVISION "\"");
  });
  
  server.on("/", handleConfig);
  // server.on("/", HTTP_POST, []() {
  //   server.send(200, "text/plain", "");
  // }, handleConfig);

  server.onNotFound(handleNotFound);          // if someone requests any other file or page, go to function 'handleNotFound'
  // and check if the file exists

  server.begin();                             // start the HTTP server
  Serial.println("HTTP server started.");
}

#ifdef USE_WEBSOCKETS
void startWebSocket() { // Start a WebSocket server
  webSocket.begin();                          // start the websocket server
  webSocket.onEvent(webSocketEvent);          // if there's an incomming websocket message, go to function 'webSocketEvent'
  Serial.println("WebSocket server started.");
}
#endif

// Loads the configuration from a file
void loadConfiguration() {
  // Open file for reading
  File file = SPIFFS.open("/config.json", "r");

  // Allocate the memory pool on the stack.
  // Don't forget to change the capacity to match your JSON document.
  // Use arduinojson.org/assistant to compute the capacity.
  StaticJsonBuffer<2048> jsonBuffer;

  // Parse the root object
  JsonObject &root = jsonBuffer.parseObject(file);

  // Copy values from the JsonObject to the Config
  //config.port = root["port"] | 2731;
  strlcpy(config.ssid,                   // <- destination
          root["ssid"] | "",  // <- source
          sizeof(config.ssid));          // <- destination's capacity
  strlcpy(config.password,                   // <- destination
          root["password"] | "",  // <- source
          sizeof(config.password));          // <- destination's capacity

  config.vakantieTijd = root["vakantieTijd"] | 72;
  config.logInterval = root["logInterval"] | 10;

  for (int n=0; n<5; n++) {
    config.programma[n].w = root["programma"][n]["w"] | (n+1)*10;
    config.programma[n].x = root["programma"][n]["x"] | 120;
    config.programma[n].y = root["programma"][n]["y"] | 7200;
    config.programma[n].z = root["programma"][n]["z"] | 5;
    config.programma[n].x2 = root["programma"][n]["x2"] | 120;
    config.programma[n].y2 = root["programma"][n]["y2"] | 7200;
    config.programma[n].z2 = root["programma"][n]["z2"] | 5;
    config.programma[n].t = root["programma"][n]["t"] | 20;
  }
  
  // Close the file (File's destructor doesn't close the file)
  file.close();
  
  if (!root.success()) {
    Serial.println(F("Failed to read file, using default configuration"));
    //saveConfiguration(); // TODO: seems to cause a crash
  }
}

// Saves the configuration to a file
void saveConfiguration() {
  // Allocate the memory pool on the stack
  // Don't forget to change the capacity to match your JSON document.
  // Use https://arduinojson.org/assistant/ to compute the capacity.
  StaticJsonBuffer<2048> jsonBuffer;

  // Parse the root object
  JsonObject &root = jsonBuffer.createObject();

  // Set the values
  root["ssid"] = config.ssid;
  root["password"] = config.password;
  
  root["vakantieTijd"] = config.vakantieTijd;
  root["logInterval"] = config.logInterval;
  
  JsonArray& programma = root.createNestedArray("programma");
  for (int n=0; n<5; n++) {
    JsonObject& programma_0 = programma.createNestedObject();
    programma_0["w"] = config.programma[n].w;
    programma_0["y"] = config.programma[n].y;
    programma_0["z"] = config.programma[n].z;
    programma_0["x"] = config.programma[n].x;
    programma_0["y2"] = config.programma[n].y2;
    programma_0["z2"] = config.programma[n].z2;
    programma_0["x2"] = config.programma[n].x2;
    programma_0["t"] = config.programma[n].t;
  }

  root.printTo(Serial);

  // Open file for writing
  File file = SPIFFS.open("/config.json", "w");
  if (!file) {
    Serial.println("failed to open config file for writing");
    return;
  }
  // Serialize JSON to file
  if (root.printTo(file) == 0) {
    Serial.println(F("Failed to write to file"));
  }
  // Close the file (File's destructor doesn't close the file)
  file.close();
}

/*__________________________________________________________SERVER_HANDLERS__________________________________________________________*/

void handleNotFound() { // if the requested file or page doesn't exist, return a 404 not found error
  if (!handleFileRead(server.uri())) {        // check if the file exists in the flash memory (SPIFFS), if so, send it
    server.send(404, "text/plain", "404: File Not Found");
  }
}

bool handleFileRead(String path) { // send the right file to the client (if it exists)
  Serial.println("handleFileRead: " + path);
  if (path.endsWith("/")) path += "index.html";          // If a folder is requested, send the index file
  String contentType = getContentType(path);             // Get the MIME type
  String pathWithGz = path + ".gz";
  if (SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)) { // If the file exists, either as a compressed archive, or normal
    if (SPIFFS.exists(pathWithGz))                         // If there's a compressed version available
      path += ".gz";                                         // Use the compressed verion
    File file = SPIFFS.open(path, "r");                    // Open the file
    size_t sent = server.streamFile(file, contentType);    // Send it to the client
    file.close();                                          // Close the file again
    Serial.println(String("\tSent file: ") + path);
    return true;
  }
  Serial.println(String("\tFile Not Found: ") + path);   // If the file doesn't exist, return false
  return false;
}

void handleFileUpload() { // upload a new file to the SPIFFS
  HTTPUpload& upload = server.upload();
  String path;
  if (upload.status == UPLOAD_FILE_START) {
    path = upload.filename;
    if (!path.startsWith("/")) path = "/" + path;
    if (!path.endsWith(".gz")) {                         // The file server always prefers a compressed version of a file
      String pathWithGz = path + ".gz";                  // So if an uploaded file is not compressed, the existing compressed
      if (SPIFFS.exists(pathWithGz))                     // version of that file must be deleted (if it exists)
        SPIFFS.remove(pathWithGz);
    }
    Serial.print("handleFileUpload Name: "); Serial.println(path);
    fsUploadFile = SPIFFS.open(path, "w");               // Open the file for writing in SPIFFS (create if it doesn't exist)
    path = String();
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize); // Write the received bytes to the file
  } else if (upload.status == UPLOAD_FILE_END) {
    if (fsUploadFile) {                                   // If the file was successfully created
      fsUploadFile.close();                               // Close the file again
      Serial.print("handleFileUpload Size: "); Serial.println(upload.totalSize);
      server.sendHeader("Location", "/success.html");     // Redirect the client to the success page
      server.send(303);
    } else {
      server.send(500, "text/plain", "500: couldn't create file");
    }
  }
}

void handleConfig() {
  if (server.args() > 0) {
    Serial.print("Config update: ");
    Serial.print(server.args());
    Serial.println(" args");
    if (server.arg("ssid") != "") {
      strlcpy(config.ssid, server.arg("ssid").c_str(), sizeof(config.ssid));
      Serial.print("ssid: ");
      Serial.println(config.ssid);
    }
    if (server.arg("password") != "") {
      strlcpy(config.password, server.arg("password").c_str(), sizeof(config.password));
      Serial.print("password: ");
      Serial.println(config.password);
    }
    if (server.arg("vakantieTijd") != "") {
      config.vakantieTijd = server.arg("vakantieTijd").toInt();
    }
    if (server.arg("logInterval") != "") {
      config.logInterval = server.arg("logInterval").toInt();
    }
    for (int n=0; n<5; n++) {
      String base = "w" + String(n) + "_";
      String arg = server.arg(base + "w");
      if (arg != "") {
        config.programma[n].w = arg.toInt();
      }
      arg = server.arg(base + "y");
      if (arg != "") {
        config.programma[n].y = arg.toInt();
      }
      arg = server.arg(base + "z");
      if (arg != "") {
        config.programma[n].z = arg.toInt();
      }
      arg = server.arg(base + "x");
      if (arg != "") {
        config.programma[n].x = arg.toInt();
      }
      arg = server.arg(base + "y2");
      if (arg != "") {
        config.programma[n].y2 = arg.toInt();
      }
      arg = server.arg(base + "z2");
      if (arg != "") {
        config.programma[n].z2 = arg.toInt();
      }
      arg = server.arg(base + "x2");
      if (arg != "") {
        config.programma[n].x2 = arg.toInt();
      }
      arg = server.arg(base + "t");
      if (arg != "") {
        config.programma[n].t = arg.toInt();
      }
    }
    saveConfiguration();
  }
  handleNotFound();
}

#ifdef USE_WEBSOCKETS
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) { // When a WebSocket message is received
  switch (type) {
    case WStype_DISCONNECTED:             // if the websocket is disconnected
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED: {              // if a new websocket connection is established
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
        // send current status
        if (brilOpen) {
          webSocket.broadcastTXT("b:open");
        } else {
          webSocket.broadcastTXT("b:dicht");
        }
      }
      break;
    case WStype_TEXT:                     // if new text data is received
      Serial.printf("[%u] get Text: %s\n", num, payload);
      if (payload[0] == 'M' && length >= 2) {            // motor control
        if (payload[1] == 'P') {
          programMode = PROGRAM;
          currentProgram = 0;
          programState = 0;
          programDirection = -programDirection;
        } else if (payload[1] == '-' || (payload[1] >= '0' && payload[1] <= '9')) {
          int s = String((char*)&payload[1]).toInt();
          Serial.println(s);
          programMode = DIRECT;
          digitalWrite(MOTOR_RELAY_PIN, HIGH);
          setMotor(s * 1023 / 24);
        }
      }
      break;
  }
}
#endif
/*__________________________________________________________HELPER_FUNCTIONS__________________________________________________________*/

String formatBytes(size_t bytes) { // convert sizes in bytes to KB and MB
  if (bytes < 1024) {
    return String(bytes) + "B";
  } else if (bytes < (1024 * 1024)) {
    return String(bytes / 1024.0) + "KB";
  } else if (bytes < (1024 * 1024 * 1024)) {
    return String(bytes / 1024.0 / 1024.0) + "MB";
  }
}

String getContentType(String filename) { // determine the filetype of a given filename, based on the extension
  if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  else if (filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}

#ifdef USE_NTP
unsigned long getTime() { // Check if the time server has responded, if so, get the UNIX time, otherwise, return 0
  if (UDP.parsePacket() == 0) { // If there's no response (yet)
    return 0;
  }
  UDP.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
  // Combine the 4 timestamp bytes into one 32-bit number
  uint32_t NTPTime = (packetBuffer[40] << 24) | (packetBuffer[41] << 16) | (packetBuffer[42] << 8) | packetBuffer[43];
  // Convert NTP time to a UNIX timestamp:
  // Unix time starts on Jan 1 1970. That's 2208988800 seconds in NTP time:
  const uint32_t seventyYears = 2208988800UL;
  // subtract seventy years:
  uint32_t UNIXTime = NTPTime - seventyYears;
  return UNIXTime;
}

void sendNTPpacket(IPAddress& address) {
  Serial.println("Sending NTP request");
  memset(packetBuffer, 0, NTP_PACKET_SIZE);  // set all bytes in the buffer to 0
  // Initialize values needed to form NTP request
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode

  // send a packet requesting a timestamp:
  UDP.beginPacket(address, 123); // NTP requests are to port 123
  UDP.write(packetBuffer, NTP_PACKET_SIZE);
  UDP.endPacket();
}
#endif

void setMotor(int pwm){
  if (pwm >= 0){
    analogWrite(LPWM_PIN, 0);
    analogWrite(RPWM_PIN, pwm);
  } else {
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, -pwm);
  }
}
