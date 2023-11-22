#pragma once

#include <Arduino.h>

#define MAXSTRING 255

#define GPSBAUD 9600
#define GPSRXPIN 44 // from tx BIANCO
#define GPSTXPIN 43 // from rx VERDE

#define BARO_SDA 41
#define BARO_SCL 40

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#define VPARRAYSIZE 8


struct statusData{

    uint32_t chipId;
    uint8_t mainpage = 2;
    bool lowPower = false;
    
    bool ble_init = false;
    bool BLE_connected=false;
    
    bool wifi_on=false;
    
    bool GPS_Fix=false;
    double GPS_Lat=0.;
    double GPS_Lon=0.;
    int16_t GPS_alt=0;
    uint16_t GPS_year=2023;
    uint8_t GPS_month=1;
    uint8_t GPS_day=1;
    uint8_t GPS_hour=0;
    uint8_t GPS_min=0;
    uint8_t GPS_sec=0;
    double GPS_speed=0.;
    double GPS_course = 0.;
    char GPS_course_c[3] = "--";
    uint8_t GPS_sat=0;
    bool restart_GPS=false;
    bool resetGpsHz=false;
    uint8_t gps_hz=5;
    uint8_t min_sat_av=5;
    
    bool baro_ok=false;

    float altitude=0.f;
    float vario=0.f;
    float vario_avg=0.f;
    float vario_avg_b=0.f;
    uint16_t vario_avg_ms=50;
    uint16_t vario_avg_ms_b=50;

    bool thermalling=false;
    bool thermal_detect=false;
    uint16_t thermal_avg=0;
    int16_t GPS_course_prev = 0;
    int8_t avg_course = 0;
    int turn = 0;

//    int16_t elevation = 1017;
    int16_t vario_dcm = 0;
    int16_t pressure = 1013;
    float batterymV = 4.2f;
    int16_t battery = 80;

    String LK8EX1_s;
    bool nmeaReady = false;
    String NMEA_raw;

};

bool gotomainpage = true;
uint8_t bat_idx=4;
uint8_t vol_idx=2;

char* volumes[3] = {   
    "#4f0a4f \xEF\x80\xA6#", // LV_SYMBOL_MUTE
    "#ff00ff \xEF\x80\xA7#", // LV_SYMBOL_VOLUME_MID
    "#ff00ff \xEF\x80\xA8#",  // LV_SYMBOL_VOLUME_MAX
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
void taskBaro(void *param);
void taskBluetooth(void *param);
void taskGPSU7(void *param);
void taskOledUpdate(void *param);

void resetUBX();
void changeGpsHz();
void setSentences();

TaskHandle_t xHandleBluetooth = NULL;
TaskHandle_t xHandleGPSU7 = NULL;

void sendUBX(const unsigned char *progmemBytes, size_t len );

