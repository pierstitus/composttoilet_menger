#include <Arduino.h>
#include <Hash.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFSEditor.h>
#include <FS.h>

AsyncWebServer server(80);

#ifndef USE_WEBSOCKETS
#define USE_WEBSOCKETS
#endif

#ifdef USE_WEBSOCKETS
AsyncWebSocket ws("/ws");
#endif

SPIFFSEditor edit("", "", SPIFFS);

void handleConfig(AsyncWebServerRequest *request) {
  if (request->args() > 0) {
    Serial.print("Config update: ");
    Serial.print(request->args());
    Serial.println(" args");

    String arg = request->arg("ssid");
    if (arg != "") {
      strlcpy(config.ssid, arg.c_str(), sizeof(config.ssid));
      Serial.print("ssid: ");
      Serial.println(config.ssid);
    }
    arg = request->arg("password");
    if (arg != "") {
      strlcpy(config.password, arg.c_str(), sizeof(config.password));
      Serial.print("password: ");
      Serial.println(config.password);
    }
    arg = request->arg("vakantieTijd");
    if (arg != "") {
      config.vakantieTijd = arg.toInt();
    }
    arg = request->arg("logInterval");
    if (arg != "") {
      config.logInterval = arg.toInt();
    }
    arg = request->arg("speedControlP");
    if (arg != "") {
      config.speedControlP = arg.toFloat();
    }
    arg = request->arg("richtingAfwisselen");
    if (arg != "") {
      config.richtingAfwisselen = true;
    } else {
      config.richtingAfwisselen = false;
    }
    for (int n=0; n<5; n++) {
      String base = "w" + String(n) + "_";
      arg = request->arg(base + "w");
      if (arg != "") {
        config.programma[n].w = arg.toInt();
      }
      arg = request->arg(base + "y");
      if (arg != "") {
        config.programma[n].y = arg.toInt();
      }
      arg = request->arg(base + "z");
      if (arg != "") {
        config.programma[n].z = arg.toInt();
      }
      arg = request->arg(base + "x");
      if (arg != "") {
        config.programma[n].x = arg.toInt();
      }
      arg = request->arg(base + "y2");
      if (arg != "") {
        config.programma[n].y2 = arg.toInt();
      }
      arg = request->arg(base + "z2");
      if (arg != "") {
        config.programma[n].z2 = arg.toInt();
      }
      arg = request->arg(base + "x2");
      if (arg != "") {
        config.programma[n].x2 = arg.toInt();
      }
      arg = request->arg(base + "t");
      if (arg != "") {
        config.programma[n].t = arg.toInt();
      }
      arg = request->arg(base + "p1");
      if (arg != "") {
        config.programma[n].p1 = arg.toInt();
      }
      arg = request->arg(base + "p0");
      if (arg != "") {
        config.programma[n].p0 = arg.toInt();
      }
    }
    saveConfiguration();
  }

  request->send(SPIFFS, "/index.html");
}

#ifdef USE_WEBSOCKETS
void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  if (type == WS_EVT_CONNECT) {
    Serial.printf("ws[%s][%u] connect\n", server->url(), client->id());
    // send current status
    if (brilOpen) {
      client->text("b:open");
    } else {
      client->text("b:dicht");
    }
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("ws[%s][%u] disconnect\n", server->url(), client->id());
  } else if (type == WS_EVT_DATA) { //data packet
    AwsFrameInfo * info = (AwsFrameInfo*)arg;
    if(info->final && info->index == 0 && info->len == len){
      //the whole message is in a single frame and we got all of it's data
      Serial.printf("ws[%s][%u] %s-message[%llu]: ", server->url(), client->id(), (info->opcode == WS_TEXT)?"text":"binary", info->len);
      if(info->opcode == WS_TEXT){
        data[len] = 0;
        Serial.printf("%s\n", (char*)data);
        if (data[0] == 'M' && len >= 2) {            // motor control
          if (data[1] == 'P') {
            programMode = PROGRAM;
            currentProgram = 0;
            if (programState < 3) {programDirection = -programDirection;}
            programState = 0;
          } else if (data[1] == '-' || (data[1] >= '0' && data[1] <= '9')) {
            int s = String((char*)&data[1]).toInt();
            Serial.println(s);
            programMode = DIRECT;
            motorPower = s * (1023 / MOTOR_SUPPLY_VOLTAGE);
            setMotor(motorPower);
          }
        }
      }
    }
  }
}
#endif

void startWebServer() { // Start a HTTP server with a file read handler and an upload handler

  // attach AsyncWebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // editor on /edit
  server.addHandler(new SPIFFSEditor("",""));

  server.on("/version", HTTP_GET, [](AsyncWebServerRequest *request) {
    String info = "\"" SRC_REVISION " ";
    info += WiFi.localIP().toString();
    info += "\"";
    request->send(200, "text/plain", info);
  });
  
  server.on("/", HTTP_POST, handleConfig);
  // server.on("/", HTTP_POST, []() {
  //   server.send(200, "text/plain", "");
  // }, handleConfig);

  // Simple Firmware Update Form
  server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", "<form method='POST' action='/update' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>");
  });
  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request){
    programMode = PAUSE;
    // turn motor and airpump off
    motorPower = 0;
    setMotor(0);
    airpump = 0;
    digitalWrite(AIRPUMP_RELAY_PIN, 0);

    shouldReboot = !Update.hasError();
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", shouldReboot?"OK":"FAIL");
    response->addHeader("Connection", "close");
    request->send(response);
  },[](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
    if(!index){
      Serial.printf("Update Start: %s\n", filename.c_str());
      Update.runAsync(true);
      if(!Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000)){
        Update.printError(Serial);
      }
    }
    if(!Update.hasError()){
      if(Update.write(data, len) != len){
        Update.printError(Serial);
      }
    }
    if(final){
      if(Update.end(true)){
        Serial.printf("Update Success: %uB\n", index+len);
      } else {
        Update.printError(Serial);
      }
    }
  });

  server.serveStatic("/generate_204", SPIFFS, "/index.html"); //Android captive portal. Maybe not needed. Might be handled by notFound handler.
  server.serveStatic("/fwlink", SPIFFS, "/index.html"); //Microsoft captive portal. Maybe not needed. Might be handled by notFound handler.

  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

  // server.onNotFound([](AsyncWebServerRequest *request){
  //   request->send(SPIFFS, "/index.html");
  // });

  server.begin();                             // start the HTTP server
  Serial.println("HTTP server started.");
}