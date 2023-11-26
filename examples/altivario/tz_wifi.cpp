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

String httpGETRequest(const char* serverName) {
  WiFiClient client;
  HTTPClient http;
    
  // Your Domain name with URL path or IP address with path
  http.begin(client, serverName);
  http.addHeader("accept", "application/json");
      
  // If you need Node-RED/server authentication, insert user and password below
  //http.setAuthorization("REPLACE_WITH_SERVER_USERNAME", "REPLACE_WITH_SERVER_PASSWORD");
  
  // Send HTTP GET request
  int httpResponseCode = http.GET();

  String payload = "{}"; 
  
  if (httpResponseCode>0) {
    log_i("HTTP Response code: %i", httpResponseCode);
    payload = http.getString();
  }
  else {
    log_i("Error code: %i", httpResponseCode);
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}

void handle_OnConnect() {
  // call to check latest firmware version
  String api_get = httpGETRequest(VERSIONCHECKURL);
  JSONVar myObject = JSON.parse(api_get);
  JSONVar keys = myObject.keys();
  //JSONVar value = myObject["datetime"];
  //wifi_date = JSON.stringify(value);
  const char* ver = myObject["version"];
  String latest_ver = ver;
  log_i("Latest built version: %s", latest_ver.c_str());

  if (latest_ver.toInt() > status.firmware_v.toInt()){
    server.send(200, "text/html", SendHTML(true,latest_ver)); 
  }else{
    server.send(200, "text/html", SendHTML(false, status.firmware_v)); 
  }

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
    ptr += "</b>\nUpdate to latest available <b>";
    ptr += version;
    ptr += "</b></p><a class=\"button button-on\" href=\"/led1off\">UPDATE Firmware</a>";
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
        if(WiFi.status()== WL_CONNECTED){

            String api_get = httpGETRequest("http://worldtimeapi.org/api/timezone/Europe/Rome");
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