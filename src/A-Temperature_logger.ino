#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP8266WiFi.h>
//#include <ESP8266WiFiMulti.h>

#define USE_NTP  // Get current time from NTP server
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
#include <ESP8266mDNS.h>
#include <DNSServer.h>
#include <FS.h>

#include <Wire.h>
#include <MechaQMC5883.h>

#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson

#include "localconfig.h" // not included in git, contains passwords etc.

typedef long time_t;
typedef int  itime_t;

void saveConfiguration(void);
void setMotor(int pwm);

struct Programma {
  int w;              // bij weerstand w
  itime_t y;          // y seconden linksom draaien
  int z;              // met snelheid z
  itime_t x;          // x seconden wachten
  itime_t y2;         // y2 seconden rechtsom draaien
  int z2;             // met snelheid z2
  itime_t x2;         // x2 seconden wachten
  int t;              // houdt temperatuur minimaal t graden
  itime_t p1;         // luchtpomp p1 seconden aan
  itime_t p0;         // luchtpomp p0 seconden uit
};

// Configuration that we'll store on disk
struct Config {
  char ssid[32];
  char password[32];
  itime_t vakantieTijd;
  itime_t logInterval;
  float speedControlP;
  Programma programma[5];
};

//const char *filename = "/config.txt";  // <- SD library uses 8.3 filenames
Config config;                         // <- global configuration object

#define ONE_HOUR 3600000L // one hour in milliseconds

// Wemos D1 R2 pinout
// TX 	TXD 	TXD
// RX 	RXD 	RXD
// A0 	Analog input (0-3.2V, with voltage divider) 	A0
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
#define AIRPUMP_RELAY_PIN D0  // D0 is also used for boot
#define HEATER_RELAY_PIN D8  // D8 has 10k pull-down, is used for boot
#define SEAT_SENSOR_PIN D7

#define MOTOR_SUPPLY_VOLTAGE 12
#define MAXPOWER (12*1023/MOTOR_SUPPLY_VOLTAGE)
#define SPEED_VOLT_FACTOR 0.5f

OneWire oneWire(TEMP_SENSOR_PIN);        // Set up a OneWire instance to communicate with OneWire devices
DallasTemperature tempSensors(&oneWire); // Create an instance of the temperature sensor class

MechaQMC5883 qmc;

//ESP8266WiFiMulti wifiMulti;    // Create an instance of the ESP8266WiFiMulti class, called 'wifiMulti'

#ifdef USE_OTA
const char *OTAName = "ESP8266";         // A name and a password for the OTA service
const char *OTAPassword = "put-your-password-here";
#endif

const char* mdnsName = "composttoilet";        // Domain name for the mDNS responder

const char *ssid = "Composttoilet"; // The name of the Wi-Fi network that will be created
const char *password = PASSWORD;   // The password required to connect to it, leave blank for an open network

// DNS server
const byte DNS_PORT = 53;
DNSServer dnsServer;

#ifdef USE_EXT_LOG
const char *logHost = LOGHOST;
//WiFiClient client;
#endif

// 0b00001111
// 0b01101111
// 0b00000001
// 0b01001001

enum ProgramMode {DIRECT, PROGRAM, MANUAL, STALL, PAUSE, VAKANTIE};
const String programModeTxt[6] = {"direct","programma ","manual","stall","pause","vakantie"};
ProgramMode programMode = PROGRAM;

#ifdef USE_NTP
WiFiUDP UDP;                   // Create an instance of the WiFiUDP class to send and receive UDP messages

IPAddress timeServerIP;        // The NTP server's IP address
const char* ntpServerName = "pool.ntp.org";

const int NTP_PACKET_SIZE = 48;          // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[NTP_PACKET_SIZE];      // A buffer to hold incoming and outgoing packets
#endif

#ifdef USE_NTP
const time_t intervalNTP = ONE_HOUR; // Update the time every hour
time_t prevNTP = 0;
time_t lastNTPResponse = millis();
#endif

//flag to use from web update to reboot the ESP
bool shouldReboot = false;

int speed;
int speedMin, speedMax;
float motorCurrent = 0;
float rotation;
const time_t intervalRotation = 100;   // rotation measurement interval
time_t prevRotation = 0;
time_t lastRotation = 0;

float speedControlP = 2.0; // 1023 = 24V, 360°/4 = 100°/s
int programSpeed = 0;
int programDirection = 1;
int motorPower = 0;
float weerstand;
float weerstandAvg = 0.0;
float weerstandSum = 0.0;
int weerstandCount = 0;
time_t programStartTime = 0;
int currentProgram = 0;
int programState = 0;

time_t stallTime = 0;
const int stallTimeTreshold = 10000;

const int maxLogSize = 600*1024;
const int minFreeSpace = 100*1024;

float temp;
const time_t intervalTemp = 1000;   // temperature measurement interval
time_t prevTemp = 0;
bool tmpRequested = false;
const time_t DS_delay = 750;         // Reading the temperature from the DS18x20 can take up to 750ms
const float maxTempError = 0.1;

int heater = 0;
time_t heaterStart = 0;
float heaterAvg = 0.0;

int airpump = 0;
time_t airpumpStart = 0;
float airpumpAvg = 0.0;
time_t airpumpStartTime = -1000000000;

int brilOpen = 0;
time_t lastBrilChange = 0;
int programTemperature = 0;

time_t lastLogTime = 0;
int logNow = 0;

int ledPattern = 0b00001111;
int ledStep = 0;
itime_t intervalLed = 250;
time_t prevLed = 0;

uint32_t timeUNIX = 0;                      // The most recent timestamp received from the time server

int errCount = 0;
int errCountTemp = 0;

#include "webserver.h"

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
  pinMode(AIRPUMP_RELAY_PIN, OUTPUT);
  
  startSPIFFS();               // Start the SPIFFS and list all contents
  
  loadConfiguration();

  startWiFi();                 // Start a Wi-Fi access point, and try to connect to some given access points. Then wait for either an AP or STA connection
  digitalWrite(LED_PIN, 1);

#ifdef USE_OTA
  startOTA();                  // Start the OTA service
#endif

  startMDNS();                 // Start the mDNS responder
  
  startWebServer();

#ifdef USE_NTP
  startUDP();                  // Start listening for UDP messages to port 123

  if (WiFi.hostByName(ntpServerName, timeServerIP)) { // Get the IP address of the NTP server
    // timeUNIX = 1;
  }
  Serial.print("Time server IP:\t");
  Serial.println(timeServerIP);
  
  if (WiFi.status() == WL_CONNECTED) {
    sendNTPpacket(timeServerIP);
    delay(500);
  }
#endif
}

/*__________________________________________________________LOOP__________________________________________________________*/
void loop() {
  time_t currentMillis = millis();

#ifdef USE_NTP
  if (WiFi.status() == WL_CONNECTED) {
    if (currentMillis - prevNTP > intervalNTP) { // Request the time from the time server every hour
      prevNTP = currentMillis;
      WiFi.hostByName(ntpServerName, timeServerIP);
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
  }
#endif

  // Measure rotation
  if (currentMillis - prevRotation > intervalRotation) {
#ifdef USE_CURRENT_SENSOR
    float adc = 0;
    for (int n=0; n<3; n++) {
      adc += analogRead(A0);
    }
    adc /= 3;
    motorCurrent = 0.8*motorCurrent + 0.2*((831 - adc) * 32); //0.9*motorCurrent + 0.1*
#endif

    int32_t x, y, z, mag;//, azimuth;
    float azimuth; //is supporting float too
    int err = qmc.read(&x, &y, &z, &azimuth);
    //Serial.printf("x: %d, y: %d, z: %d\r\n", x, y, z);
    if (err) {
      errCount++;
      //Serial.println("rotation sensor error " + String(err));
      if (!(errCount%10)) {
        ws.textAll("x:rotation sensor error " + String(err) + " (" + String(errCount) + "x)");
      }
    } else { 
      errCount = 0;
      //azimuth = qmc.azimuth(&y,&x);//you can get custom azimuth
      time_t now = millis();
      mag = (x/512)*(x/512) + (y/512)*(y/512) + (z/512)*(z/512); // max magnitude 49152
      speed = (float)((int)(1000 * (rotation - azimuth + 360 + 180))%360000 - 180000) / (now - lastRotation);
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
      lastRotation = now;
    }
    if (programMode == PROGRAM) {
      Programma* p = &config.programma[currentProgram];
      if (programState == 1 || programState == 3) {
        weerstandSum += weerstand;
        weerstandCount++;
      }
      if (programState == 4 && (currentMillis - programStartTime) > 1000 * p->y2) {
        // Restart program
        programState = 0;
      }
      if (programState == 0) {
        // Turn right with speed z for x seconds
        programState = 1;
        stallTime = 0;
        programStartTime = currentMillis;
        programSpeed = programDirection * p->z;
        motorPower = SPEED_VOLT_FACTOR * programSpeed * (1023 / MOTOR_SUPPLY_VOLTAGE);
        weerstandSum = 0.0;
        weerstandCount = 0;
      } else if (programState == 1 && (currentMillis - programStartTime) > 1000 * p->x) {
        // Pause
        programState = 2;
        programStartTime = currentMillis;
        programSpeed = 0;
        motorPower = 0;
        if (weerstandCount) {weerstandAvg = weerstandSum / weerstandCount;}
        logNow = true;
        for (int n=0; n<4; n++) {
          if (weerstandAvg < config.programma[n].w) {
            currentProgram = n;
            break;
          }
        }
      } else if (programState == 2 && (currentMillis - programStartTime) > 1000 * p->y) {
        // Turn left
        programState = 3;
        stallTime = 0;
        programStartTime = currentMillis;
        programSpeed = programDirection * -p->z2;
        motorPower = SPEED_VOLT_FACTOR * programSpeed * (1023 / MOTOR_SUPPLY_VOLTAGE);
      } else if (programState == 3 && (currentMillis - programStartTime) > 1000 * p->x2) {
        // Pause
        programState = 4;
        programStartTime = currentMillis;
        programSpeed = 0;
        motorPower = 0;
        // Go into holiday mode
        if ((currentMillis - lastBrilChange) > ONE_HOUR * config.vakantieTijd) {
          //programMode = VAKANTIE;
          currentProgram = 4;
        }
      }
      
      // Motor speed feedback loop
      if (!err && programSpeed) {
        float speedError = programSpeed - speed;
        motorPower = motorPower + speedControlP * speedError;
        if (abs(motorPower) > MAXPOWER) {
          motorPower = motorPower > 0 ? MAXPOWER : -MAXPOWER;
          // Detect motor stall
          if (abs(speed) < 1.0) {
            if (!stallTime) {
              stallTime = currentMillis;
            } else if (currentMillis - stallTime > stallTimeTreshold) {
              programMode = STALL;
              motorPower = 0;
            }
          } else {
            stallTime = 0;
          }
        }
      } else if (errCount > 10) { // if sensor doesn't work fallback to voltage based control
        motorPower = SPEED_VOLT_FACTOR * programSpeed * (1023 / MOTOR_SUPPLY_VOLTAGE);
      }
      setMotor(motorPower);
    }
    if (programMode == STALL) {
      if (abs(speed) > 5.0) {
        programMode = MANUAL;
      } else if (currentMillis - stallTime > 2 * stallTimeTreshold) {
        programMode = PROGRAM;
        programDirection = -programDirection;
        programSpeed = -programSpeed;
        motorPower = -motorPower;
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
    ws.textAll("r:" + String(rotation,1) + ",s:" + String(speed)
                           + ",w:" + String(weerstandAvg,1) + ",u:" + String((MOTOR_SUPPLY_VOLTAGE/1023.0) * motorPower,1)
                           + ",v:" + String(heater) + ",l:" + String(airpump) + ",p:" + p);
#endif
    
    
    int brilSensor = !digitalRead(SEAT_SENSOR_PIN);
    if (brilOpen != brilSensor && lastBrilChange+1000 < currentMillis) {
      lastBrilChange = currentMillis;
      brilOpen = brilSensor;
      logNow = true;
      if (brilOpen) {
        programMode = PAUSE;
        motorPower = 0;
        setMotor(0);
        // turn airpump off
        airpump = 0;
        digitalWrite(AIRPUMP_RELAY_PIN, 0);
        airpumpStartTime = currentMillis - 1000 * config.programma[currentProgram].p0;
      } else {
        programMode = PROGRAM;
        currentProgram = 0;
        if (programState < 3) {programDirection = -programDirection;}
        programState = 0;
      }
      lastLogTime = 0; // 
#ifdef USE_WEBSOCKETS
      if (brilOpen) {
        ws.textAll("b:open");
      } else {
        ws.textAll("b:dicht");
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
    float tmp = tempSensors.getTempCByIndex(0); // Get the temperature from the sensor
    if (tmp < -120) {
      //Temperature sensor error
      errCountTemp++;
      ws.textAll("x:temperature sensor error " + String(tmp,0) + " (" + String(errCountTemp) + "x)");
    } else {
      errCountTemp = 0;
      temp = round(tmp * 10.0) / 10.0; // round temperature to 1 digits
  #ifdef USE_WEBSOCKETS
      ws.textAll("t:" + String(temp,1));
  #endif
    }
    
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
  }

  if (airpump == 0) {
    if (!brilOpen &&
        config.programma[currentProgram].p1 &&
        currentMillis - airpumpStartTime > 1000 * config.programma[currentProgram].p0) {
      // turn airpump on
      airpump = 1;
      digitalWrite(AIRPUMP_RELAY_PIN, 1);
      airpumpStart = airpumpStartTime = currentMillis;
    }
  } else if (config.programma[currentProgram].p0 &&
             currentMillis - airpumpStartTime > 1000 * config.programma[currentProgram].p1) {
    // turn airpump off
    airpump = 0;
    digitalWrite(AIRPUMP_RELAY_PIN, 0);
    airpumpAvg += currentMillis - heaterStart;
    airpumpStartTime = currentMillis;
  }

  if (logNow || currentMillis - lastLogTime > 1000 * config.logInterval) {
#ifdef USE_NTP
    if (timeUNIX != 0) {
      time_t actualTime = timeUNIX + (currentMillis - lastNTPResponse) / 1000;
      // The actual time is the last NTP time plus the time that has elapsed since the last NTP response
      Serial.printf("Appending data to log.csv: %lu,", actualTime);
      Serial.println(temp);
      File tempLog = SPIFFS.open("/log.csv", "a"); // Write the time and the temperature to the csv file
      if (tempLog.size() > maxLogSize) {
        tempLog.close();
        SPIFFS.remove("/log-2.csv");
        SPIFFS.rename("/log-1.csv", "/log-2.csv");
        SPIFFS.rename("/log.csv", "/log-1.csv");
        tempLog = SPIFFS.open("/log.csv", "a");
      }
      if (tempLog.size() == 0) {
        tempLog.print("time,temp,weerstand,verwarming,luchtpomp,bril\n");
      }
      FSInfo fs_info;
      SPIFFS.info(fs_info);
      if (tempLog.size() <= maxLogSize && fs_info.totalBytes - fs_info.usedBytes > minFreeSpace) {
        if (heater) {heaterAvg += currentMillis - heaterStart;}
        heaterAvg /= currentMillis - lastLogTime;
        if (airpump) {airpumpAvg += currentMillis - airpumpStart;}
        airpumpAvg /= currentMillis - lastLogTime;

        tempLog.printf("%ld,%.1f,%.0f,%.1f,%.1f,%d\n", actualTime, temp, weerstandAvg, 10*heaterAvg, 10*airpumpAvg, brilOpen);

        heaterAvg = 0;
        heaterStart = currentMillis;
        airpumpAvg = 0;
        airpumpStart = currentMillis;
      }
      tempLog.close();
    } else if (WiFi.status() == WL_CONNECTED) {     // If we didn't receive an NTP response yet, send another request
      WiFi.hostByName(ntpServerName, timeServerIP);
      sendNTPpacket(timeServerIP);
      //delay(500);
    }
#endif
#ifdef USE_EXT_LOG
    Serial.print("Sending data to ");
    Serial.println(logHost);
    time_t tmp = millis();
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
    // Read all the lines of the reply from server and print them to Serial
    while (client.available()) { // client.connected() && 
      String line = client.readStringUntil('\r');
      Serial.print(line);
    }
#endif
  lastLogTime = currentMillis;
  logNow = 0;
}
  
  // LED blinker
  if (currentMillis - prevLed > intervalLed) {
    ledStep = (ledStep + 1)%8;
    digitalWrite(LED_PIN, !((ledPattern >> ledStep) & 0x01));
    prevLed = currentMillis;
  }
  
#ifdef USE_OTA
  ArduinoOTA.handle();                        // listen for OTA events
#endif

  dnsServer.processNextRequest();

  if (shouldReboot) {
    Serial.println("Rebooting...");
    delay(100);
    ESP.restart();
  }
}

/*__________________________________________________________SETUP_FUNCTIONS__________________________________________________________*/

void startWiFi() { // Try to connect to some given access points. Then wait for a connection
  WiFi.softAP(ssid, password);
  Serial.println ("Access Point created");

  /* Setup the DNS server redirecting all the domains to the apIP */
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

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
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connected to ");
    Serial.println(WiFi.SSID());             // Tell us what network we're connected to
    Serial.print("IP address:\t");
    Serial.print(WiFi.localIP());            // Send the IP address of the ESP8266 to the computer
    Serial.println("\r\n");
  }
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
  if (MDNS.begin(mdnsName)) {                        // start the multicast domain name server
    Serial.print("mDNS responder started: http://");
    Serial.print(mdnsName);
    Serial.println(".local");
  } else {
    Serial.println("mDNS responder error!");
  }
}

// Loads the configuration from a file
void loadConfiguration() {
  // Open file for reading
  File file = SPIFFS.open("/config.json", "r");

  // Allocate the memory pool on the stack.
  // Don't forget to change the capacity to match your JSON document.
  // Use arduinojson.org/assistant to compute the capacity.
  StaticJsonDocument<2048> doc;

  // Parse the root object
  DeserializationError error = deserializeJson(doc, file);

  // Copy values from the JsonObject to the Config
  //config.port = root["port"] | 2731;
  strlcpy(config.ssid,                   // <- destination
          doc["ssid"] | "",  // <- source
          sizeof(config.ssid));          // <- destination's capacity
  strlcpy(config.password,                   // <- destination
          doc["password"] | "",  // <- source
          sizeof(config.password));          // <- destination's capacity

  config.vakantieTijd = doc["vakantieTijd"] | 72;
  config.logInterval = doc["logInterval"] | 10;
  config.speedControlP = doc["speedControlP"] | 0.5;

  for (int n=0; n<5; n++) {
    config.programma[n].w = doc["programma"][n]["w"] | (n+1)*10;
    config.programma[n].x = doc["programma"][n]["x"] | 120;
    config.programma[n].y = doc["programma"][n]["y"] | 7200;
    config.programma[n].z = doc["programma"][n]["z"] | 5;
    config.programma[n].x2 = doc["programma"][n]["x2"] | 120;
    config.programma[n].y2 = doc["programma"][n]["y2"] | 7200;
    config.programma[n].z2 = doc["programma"][n]["z2"] | 5;
    config.programma[n].t = doc["programma"][n]["t"] | 20;
    config.programma[n].p1 = doc["programma"][n]["p1"] | 30;
    config.programma[n].p0 = doc["programma"][n]["p0"] | 3600;
  }
  
  // Close the file (File's destructor doesn't close the file)
  file.close();
  
  if (error) {
    Serial.println(F("Failed to read file, using default configuration"));
    //saveConfiguration(); // TODO: seems to cause a crash
  }
}

// Saves the configuration to a file
void saveConfiguration() {
  // Allocate the memory pool on the stack
  // Don't forget to change the capacity to match your JSON document.
  // Use https://arduinojson.org/assistant/ to compute the capacity.
  StaticJsonDocument<2048> doc;;

  // Set the values
  doc["ssid"] = config.ssid;
  doc["password"] = config.password;
  
  doc["vakantieTijd"] = config.vakantieTijd;
  doc["logInterval"] = config.logInterval;
  doc["speedControlP"] = config.speedControlP;
  
  JsonArray programma = doc.createNestedArray("programma");
  for (int n=0; n<5; n++) {
    JsonObject programma_0 = programma.createNestedObject();
    programma_0["w"] = config.programma[n].w;
    programma_0["y"] = config.programma[n].y;
    programma_0["z"] = config.programma[n].z;
    programma_0["x"] = config.programma[n].x;
    programma_0["y2"] = config.programma[n].y2;
    programma_0["z2"] = config.programma[n].z2;
    programma_0["x2"] = config.programma[n].x2;
    programma_0["t"] = config.programma[n].t;
    programma_0["p1"] = config.programma[n].p1;
    programma_0["p0"] = config.programma[n].p0;
  }

  serializeJson(doc, Serial);

  // Open file for writing
  File file = SPIFFS.open("/config-tmp.json", "w");
  if (!file) {
    Serial.println("failed to open config file for writing");
    return;
  }
  // Serialize JSON to file
  if (serializeJson(doc, file) == 0) {
    Serial.println(F("Failed to write to file"));
  }
  // Close the file (File's destructor doesn't close the file)
  file.close();
  SPIFFS.remove("/config-2.json");
  SPIFFS.rename("/config-1.json", "/config-2.json");
  SPIFFS.rename("/config.json", "/config-1.json");
  SPIFFS.rename("/config-tmp.json", "/config.json");
}

/*__________________________________________________________HELPER_FUNCTIONS__________________________________________________________*/

String formatBytes(size_t bytes) { // convert sizes in bytes to KB and MB
  if (bytes < 1024) {
    return String(bytes) + "B";
  } else if (bytes < (1024 * 1024)) {
    return String(bytes / 1024.0) + "KB";
  } else {
    return String(bytes / 1024.0 / 1024.0) + "MB";
  }
}

#ifdef USE_NTP
time_t getTime() { // Check if the time server has responded, if so, get the UNIX time, otherwise, return 0
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
  if (pwm > 1023) pwm = 1023;
  else if (pwm < -1023) pwm = -1023;
  if (pwm >= 0){
    analogWrite(LPWM_PIN, 0);
    analogWrite(RPWM_PIN, pwm);
  } else {
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, -pwm);
  }
}
