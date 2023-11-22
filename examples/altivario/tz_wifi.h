#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <Arduino_JSON.h>

#define TZWIFINETSSID "Vodafone-A41327011"
#define TZWIFINETSSIDPSW "p2qEEmMGAGx429yY"
#define TZWIFIAPSSID "Tz_S3OLED_AP"
#define TZWIFIAPSSIDPSW NULL

String httpGETRequest(const char* serverName);
String SendHTML(uint8_t led1stat,uint8_t led2stat);

void TzWifiBegin();
void TzWifiOff();
void handle_OnConnect();
void handle_NotFound();
void HandleMyClients();