#pragma once

#include <Arduino.h>

struct statusData{
    bool ble_on=false;
    bool wifi_on=false;
    bool gps_fix=false;
    bool baro_ok=false;

    static uint16_t elevation;
    static uint16_t vario_display;
    static int16_t vario_dcm;
    static int16_t bussola;

};

bool gotomainpage = true;
static uint8_t bat_idx=4;
static uint8_t vol_idx=2;

static char* volumes[3] = {   
    "#4f0a4f \xEF\x80\xA6#", // LV_SYMBOL_MUTE
    "#ff00ff \xEF\x80\xA7#", // LV_SYMBOL_VOLUME_MID
    "#ff00ff \xEF\x80\xA8#"  // LV_SYMBOL_VOLUME_MAX
};

const char* barook = "#ff00ff \xEF\x80\x8C#";
const char* baroko = "#4f0a4f \xEF\x80\x8C#";

const char* wfon = "#ff00ff \xEF\x87\xAB#";
const char* wfoff = "#4f0a4f \xEF\x87\xAB#";

const char* bleon = "#ff00ff \xEF\x8a\x93#";
const char* bleoff = "#4f0a4f \xEF\x8a\x93#";

const char* gpsfix = "#ff00ff \xEF\x84\xA4#";
const char* gpsnofix = "#4f0a4f \xEF\x84\xA4#";

const char* pv = "#d5ff03 \xEF\x81\xA7#";
const int pbg = 0x5d750c;
const char* mv = "#ff1900 \xEF\x81\xA8#";
const int mbg = 0xb51919;

void led_task(void *param);
void elev_task(void *param);
void vario_task(void *param);
void gps_task(void *param);
