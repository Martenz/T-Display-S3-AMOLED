/*!
 * @file tz_wifi.h
 *
 *
 */

/**/

extern struct statusData status;

#ifndef __TZ_WIFI_H__
#define __TZ_WIFI_H__

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <Arduino_JSON.h>

#include "altivario.h"

#define TZWIFINETSSID "Vodafone-A41327011"
#define TZWIFINETSSIDPSW "p2qEEmMGAGx429yY"
#define TZWIFIAPSSID "Tz_S3OLED_AP"
#define TZWIFIAPSSIDPSW NULL

#define DOWNLOADURL "https://github.com/Martenz/T-Display-S3-AMOLED/releases/download/tzi-altivario-v1.0.0/latest_firmware.bin"

String httpGETRequest(const char* serverName);
String SendHTML(bool update);

void TzWifiBegin();
void TzWifiOff();
void handle_OnConnect();
void handle_NotFound();
void HandleMyClients();

#endif
