#include "tz_wifi.h"

const char* wifi_network_ssid     = TZWIFINETSSID;
const char* wifi_network_password =  TZWIFINETSSIDPSW;
const char *soft_ap_ssid          = TZWIFIAPSSID;
const char *soft_ap_password      = TZWIFIAPSSIDPSW;

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastTime = 0;
// Timer set to 10 minutes (600000)
//unsigned long timerDelay = 600000;
// Set timer to 5 seconds (5000)
unsigned long timerDelay = 5000;

String wifi_date = "-";

//IPAddress local_ip(192,168,1,1);
// IPAddress gateway(192,168,1,1);
// IPAddress subnet(255,255,255,0);
WebServer server(80);

// Global variables
int totalLength;       //total size of firmware
int currentLength = 0; //current size of written firmware

String httpsGETRequest(const char* serverName) {
  //WiFiClient client;
  WiFiClientSecure client;// = new WiFiClientSecure;
  //HTTPClient http;
  HTTPClient https;
    
  // Your Domain name with URL path or IP address with path
  // http.begin(client, serverName);
  // http.addHeader("accept", "application/json");
  client.setInsecure();
  https.addHeader("accept", "application/json");
  https.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  https.begin(client, serverName);
      
  // If you need Node-RED/server authentication, insert user and password below
  //http.setAuthorization("REPLACE_WITH_SERVER_USERNAME", "REPLACE_WITH_SERVER_PASSWORD");
  
  // Send HTTP GET request
  // int httpResponseCode = http.GET();
  int httpResponseCode = https.GET();

  String payload = "{}"; 
  
  if (httpResponseCode>0) {
    log_i("HTTP Response code: %i", httpResponseCode);
    // payload = http.getString();
    payload = https.getString();
  }
  else {
    log_i("Error code: %i", httpResponseCode);
    Serial.println(httpResponseCode);
  }
  // Free resources
  // http.end();
  https.end();

  client.stop();

  return payload;
}

void handle_OnConnect() {
  // call to check latest firmware version
  String api_get = httpsGETRequest(VERSIONCHECKURL);
  JSONVar myObject = JSON.parse(api_get);
  JSONVar keys = myObject.keys();
  //JSONVar value = myObject["datetime"];
  //wifi_date = JSON.stringify(value);
  uint16_t ver = myObject["version"];
  log_i("Latest built version: %i", ver);
  
  if (ver > status.firmware_v){
    server.send(200, "text/html", SendHTML(true,String(ver),false)); 
  }else{
    server.send(200, "text/html", SendHTML(false, String(status.firmware_v),false)); 
  }

}

// Set time via NTP, as required for x.509 validation
void setClock() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");  // UTC
 
  log_i("Waiting for NTP time sync: ");
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    yield();
    delay(500);
    now = time(nullptr);
  }
 
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  log_i("Current time: %s",asctime(&timeinfo));
}
 
// Function to update firmware incrementally
// Buffer is declared to be 128 so chunks of 128 bytes
// from firmware is written to device until server closes
void updateFirmware(uint8_t *data, size_t len){
  Update.write(data, len);
  currentLength += len;
  // Print dots while waiting for update to finish
  // if current length of written firmware is not equal to total firmware size, repeat
  if(currentLength != totalLength) return;
  Update.end(true);
  log_i("\nUpdate Success, Total Size: %u\nRebooting...\n", currentLength);
  // Restart ESP32 to see changes 
  ESP.restart();
}

void handle_OnUpdate(){

  server.send(200, "text/html", SendHTML(true,"Updating...",true)); 
  setClock();

  //WiFiClient client;
  WiFiClientSecure client;// = new WiFiClientSecure;
  //HTTPClient http;
  HTTPClient https;
    
  // Your Domain name with URL path or IP address with path
  // http.begin(client, serverName);
  // http.addHeader("accept", "application/json");
  client.setInsecure();
  https.addHeader("accept", "*/*");
  https.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  https.begin(client, DOWNLOADURL);
      
  // If you need Node-RED/server authentication, insert user and password below
  //http.setAuthorization("REPLACE_WITH_SERVER_USERNAME", "REPLACE_WITH_SERVER_PASSWORD");
  
  // Send HTTP GET request
  // int httpResponseCode = http.GET();
  int httpResponseCode = https.GET();

  // If file is reachable, start downloading
  if(httpResponseCode > 0){
      // get length of document (is -1 when Server sends no Content-Length header)
      totalLength = https.getSize();
      // transfer to local variable
      int len = totalLength;
      // this is required to start firmware update process
      Update.begin(UPDATE_SIZE_UNKNOWN);
      log_i("FW Size: %u\n",totalLength);
      // create buffer for read
      uint8_t buff[128] = { 0 };
      // get tcp stream
      WiFiClient * stream = https.getStreamPtr();
      // read all data from server
      log_i("Updating firmware...");
      while(https.connected() && (len > 0 || len == -1)) {
           // get available data size
           size_t size = stream->available();
           if(size) {
              // read up to 128 byte
              int c = stream->readBytes(buff, ((size > sizeof(buff)) ? sizeof(buff) : size));
              // pass to function
              updateFirmware(buff, c);
              if(len > 0) {
                 len -= c;
              }
           }
           delay(1);
      }
  }
}

void handle_NotFound(){
  server.send(404, "text/plain", "Not found");
}

String SendHTML(bool update, String version, bool updating){
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr +="<title>TzInstruments</title>\n";
  ptr +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";
  ptr +=".button {display: block;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n";
  ptr +=".button-on {background-color: #3498db;}\n";
  ptr +=".button-on:active {background-color: #2980b9;}\n";
  ptr +=".button-off {background-color: #34495e;}\n";
  ptr +=".button-off:active {background-color: #2c3e50;}\n";
  ptr +="p {font-size: 14px;color: #888;margin-bottom: 10px;}\n";
  ptr +="</style>\n";
  ptr +="</head>\n";
  ptr +="<body>\n";
  ptr +="<h1>TzInstruments Configurator</h1>\n";
  ptr +="<h3>Using Access Point(AP) Mode</h3>\n";
  
  ptr += "<p>Current date: ";
  ptr += wifi_date;
  ptr += "</p>";

  if (update==false){
    ptr += "<p>Updated to latest Version: <b>";
    ptr += status.firmware_v;
    ptr += "</b></p>";
  }else{
    ptr += "<p>You are running on an old version: <b>";
    ptr += status.firmware_v;
    ptr += "</b>\nUpdate to latest available ";
    if (updating){
      ptr += "<p><b>" + version + "</b></p>";
      ptr += "<p>Wait until esp32 restarts with new firmware installed, once done reconnect through wifi.</p>";
    }else{
      ptr += "<b>" + version;
      ptr += "</b></p><a class=\"button button-on\" href=\"/update\">UPDATE Firmware</a>";
    }
  }

  ptr +="</body>\n";
  ptr +="</html>\n";
  return ptr;
}

void TzWifiBegin(){
    // WIFI AP

    WiFi.mode(WIFI_AP_STA);
    log_i("[*] Creating ESP32 AP");
    WiFi.softAP(soft_ap_ssid, soft_ap_password);
    // WiFi.softAPConfig(local_ip, gateway, subnet);
    log_i("[+] AP Created with IP Gateway: %s", WiFi.softAPIP().toString().c_str());

    server.on("/", handle_OnConnect);
    server.on("/update", handle_OnUpdate);
    server.onNotFound(handle_NotFound);

//    server.on("/update", ...);

    server.begin();
    log_i("HTTP server started");

    // WIFI AP ---

    // WIFI Station

    WiFi.begin(wifi_network_ssid, wifi_network_password);
    log_i("[*] Connecting to WiFi Network");

    while(WiFi.status() != WL_CONNECTED)
    {
        log_i(".");
        delay(100);
    }

    log_i("[+] Connected to the WiFi network with local IP : %s", WiFi.localIP().toString().c_str());

    // WIFI Station ---

    lastTime = 0;
}

void TzWifiOff(){

    server.stop();
    log_i("HTTP server stopped");
    WiFi.disconnect(true);
    log_i("[+] WiFi network disconnected");
    WiFi.mode(WIFI_OFF);
    log_i("[+] WiFi OFF");
}

void HandleMyClients(){
    server.handleClient();

    if ((millis() - lastTime) > timerDelay) {
        //Check WiFi connection status
        if(WiFi.status()== WL_CONNECTED & status.updating != true){

            String api_get = httpsGETRequest(WORLDDATETIME);
            JSONVar myObject = JSON.parse(api_get);
            JSONVar keys = myObject.keys();
            //JSONVar value = myObject["datetime"];
            //wifi_date = JSON.stringify(value);
            const char* date = myObject["datetime"];
            wifi_date = date;
            log_i("API date: %s", wifi_date.c_str());

        }
        lastTime = millis();
    }

}