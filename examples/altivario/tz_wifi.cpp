#include "tz_wifi.h"

// const char* wifi_network_ssid     = status.wifi_ssid;
// const char* wifi_network_password =  status.wifi_psw;
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

  if (status.wifi_connected){

    //call to get current date
    String api_date = httpsGETRequest(WORLDDATETIME);
    JSONVar dateobj = JSON.parse(api_date);
    //JSONVar value = myObject["datetime"];
    //wifi_date = JSON.stringify(value);
    const char* date = dateobj["datetime"];
    wifi_date = date;
    log_i("API date: %s", wifi_date.c_str());

    // call to check latest firmware version
    String api_get = httpsGETRequest(VERSIONCHECKURL);
    JSONVar myObject = JSON.parse(api_get);
    //JSONVar value = myObject["datetime"];
    //wifi_date = JSON.stringify(value);
    uint16_t ver = myObject["version"];
    log_i("Latest built version: %i", ver);
    
    if (ver > status.firmware_v){
      server.send(200, "text/html", SendHTML(true,String(ver))); 
    }else{
      server.send(200, "text/html", SendHTML(false, String(status.firmware_v))); 
    }
  }else{
      server.send(200, "text/html", SendHTML(false, String(status.firmware_v))); 
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

  delay(1000);
  // Restart ESP32 to see changes 
  ESP.restart();
}

void handle_OnUpdate(){
  server.sendHeader("Location", "/",true);  
  server.send(302, "text/plain", ""); 
  setClock();
  delay(50);
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

void handle_settings(){

  // validate and write settings to SPIFFS
  if (!server.hasArg("settings")) {
    server.send(404, "text/plain", "Settings Not found");
  }
  String body = server.arg("settings");
  JSONVar settings = JSON.parse(body);

  bool validate=true;
  if(!settings.hasOwnProperty(String("volume"))){
    validate = false;
  }

  // TODO add all checks for complete attributes here, validate input in form before send
  // ...

  // SAVE TO SPIFFS
  File sfile = SPIFFS.open(SETTINGS_FileName, FILE_WRITE);
  if (!sfile) {
    log_e("There was an error opening the file for writing");
    return;
  }
  if (sfile.print(body.c_str())) {
    log_i("Setting Updated.");
  } else {
    log_e("File write failed");
  }
  sfile.close();
  delay(100);
  loadConfiguration();
  updateUi();

  delay(100);

  server.sendHeader("Location", "/",true);  
  server.send(302, "text/plain", ""); 
}

void handle_NotFound(){
  server.send(404, "text/plain", "Not found");
}

String SendHTML(bool update, String version){
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
  ptr +="#all-settings{text-align: left;display: grid;width: 90%;max-width: 400px;margin: auto;}";
  ptr +="</style>\n";
  ptr +="</head>\n";
  ptr +="<body>\n";
  ptr +="<h1>TzInstruments Configurator</h1>\n";
  ptr +="<h3>Using Access Point(AP) Mode</h3>\n";
  
  if (status.wifi_connected){

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
        ptr += "<b>" + version;
        ptr += "</b></p><a id=\"update\" class=\"button button-on\" href=\"/update\">UPDATE Firmware</a>";
        ptr += "<script>";
        ptr += "const u = document.getElementById('update');";
        ptr += "u.addEventListener('click',function(){u.innerHTML = 'Updating...';";
        ptr += "alert('Wait unitl esp32 restarts with new firmware, then reconnect to wifi if needed.');";
        ptr += "});</script>";
    }
  }else{
    ptr += "<p>WiFi connection not available<br>";
    ptr += "please check WiFi SSID and PSW are correct or Network is available and retry.<br>";
    ptr += "You can edit SSID and PSW in setting here below.</p>";
  }
  // TODO test update settings from webserver
  ptr += "<div id='all-settings'></div>";
  ptr += "<form action='/settings' method='POST'><div>";
  ptr += "<label for='settings'>Edit raw settings here below or paste from <a href='#'>Online Configurator</a></label>";
  ptr += "<p><textarea id='settings' name='settings' rows='25' cols='50'>";
  String jsonsettings = JSON.stringify(status.jsonSettings);
  //jsonsettings.replace(",\"",",\n\"");
  jsonsettings.replace(",{\"vval\":null}","");
  ptr += jsonsettings;
  ptr += "</textarea></p>";
  ptr += "<button id='update-settings'>Update Settings</button></div></form>";

  ptr += "<script type='text/javascript'>";
  ptr += String(js_script);
  ptr += "</script>";
  ptr +="</body>\n";
  ptr +="</html>\n";
  return ptr;
}

void TzWifiBegin(){

    WiFi.mode(WIFI_AP_STA);

    // WIFI Station

    if (WiFi.begin(status.wifi_ssid, status.wifi_psw)){
      log_i("[*] Connecting to WiFi Network");
      uint32_t startTime = millis();
      while(WiFi.status() != WL_CONNECTED)
      {
          log_i(".");
          delay(500);
          if (millis() - startTime > 10000){
            log_i("Wifi Connection failed, wrong or missing ssid/password or network not available");
            status.wifi_connected=false;
            WiFi.disconnect(false,true);
            break;
          }

      }

      if (WiFi.status() == WL_CONNECTED){
        status.wifi_connected = true;

        log_i("[+] Connected to the WiFi network with local IP : %s", WiFi.localIP().toString().c_str());

        // WIFI Station ---

        lastTime = 0;
      }
    }

    // WIFI AP

    log_i("[*] Creating ESP32 AP");
    delay(50);
    WiFi.softAP(soft_ap_ssid, soft_ap_password);
    // WiFi.softAPConfig(local_ip, gateway, subnet);
    log_i("[+] AP Created with IP Gateway: %s", WiFi.softAPIP().toString().c_str());

    server.on("/", HTTP_GET, handle_OnConnect);
    server.on("/update", HTTP_GET, handle_OnUpdate);
    server.on("/settings", HTTP_POST, handle_settings);
    server.onNotFound(handle_NotFound);

//    server.on("/update", ...);

    server.begin();
    log_i("HTTP server started");

    // WIFI AP ---

}

void TzWifiOff(){

    server.stop();
    log_i("HTTP server stopped");
    WiFi.disconnect(true);
    log_i("[+] WiFi network disconnected");
    WiFi.mode(WIFI_OFF);
    log_i("[+] WiFi OFF");
    status.wifi_connected = false;
}

void HandleMyClients(){
    server.handleClient();
}