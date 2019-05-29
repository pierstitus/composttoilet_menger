#include <FS.h>

#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#ifdef USE_WEBSOCKETS
#include <WebSocketsServer.h>
#endif

ESP8266WebServer server(80);       // create a web server on port 80
#ifdef USE_WEBSOCKETS
WebSocketsServer webSocket = WebSocketsServer(81);    // create a websocket server on port 81
#endif
ESP8266HTTPUpdateServer httpUpdater;

File fsUploadFile;                                    // a File variable to temporarily store the received file


String getContentType(String filename) { // determine the filetype of a given filename, based on the extension
  if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  else if (filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}

/*__________________________________________________________SERVER_HANDLERS__________________________________________________________*/

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

void handleNotFound() { // if the requested file or page doesn't exist, return a 404 not found error
  if (!handleFileRead(server.uri())) {        // check if the file exists in the flash memory (SPIFFS), if so, send it
    server.send(404, "text/plain", "404: File Not Found");
  }
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
          if (programState < 3) {programDirection = -programDirection;}
          programState = 0;
        } else if (payload[1] == '-' || (payload[1] >= '0' && payload[1] <= '9')) {
          int s = String((char*)&payload[1]).toInt();
          Serial.println(s);
          programMode = DIRECT;
          setMotor(s * (1023 / MOTOR_SUPPLY_VOLTAGE));
        }
      }
      break;
  }
}
#endif

void startWebServer() {
#ifdef USE_WEBSOCKETS
  webSocket.begin();                          // start the websocket server
  webSocket.onEvent(webSocketEvent);          // if there's an incomming websocket message, go to function 'webSocketEvent'
  Serial.println("WebSocket server started.");
#endif

  httpUpdater.setup(&server);

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