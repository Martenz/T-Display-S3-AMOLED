#ifndef __ALTIVARIO_H__
#define __ALTIVARIO_H__

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
    uint16_t firmware_v = 0;
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

#endif