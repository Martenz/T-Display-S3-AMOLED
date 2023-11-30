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
#include "SPIFFS.h"
#include <Update.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Arduino_JSON.h>
#include <time.h>

#include "altivario.h"

#define TZWIFINETSSID "Vodafone-A41327011"
#define TZWIFINETSSIDPSW "p2qEEmMGAGx429yY"
#define TZWIFIAPSSID "Tz_S3OLED_AP"
#define TZWIFIAPSSIDPSW NULL

#define WORLDDATETIME "https://worldtimeapi.org/api/timezone/Europe/Rome"
#define VERSIONCHECKURL "https://github.com/Martenz/T-Display-S3-AMOLED/raw/tzi/altivario/firmware/version.json"
#define DOWNLOADURL "https://github.com/Martenz/T-Display-S3-AMOLED/raw/tzi/altivario/firmware/latest_firmware.bin"

String httpsGETRequest(const char* serverName);
String SendHTML(bool update, String version);

void updateFirmware(uint8_t *data, size_t len);

void TzWifiBegin();
void TzWifiOff();
void handle_OnConnect();
void handle_OnUpdate();
void handle_settings();
void handle_NotFound();
void HandleMyClients();

#endif
