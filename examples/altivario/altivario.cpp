#include <Arduino.h>
#include <altivario.h>
#include <SoftwareSerial.h>

#include "melodies.h"

#include "SPIFFS.h"
#include "OneButton.h" /* https://github.com/mathertel/OneButton.git */
#include "lvgl.h"      /* https://github.com/lvgl/lvgl.git */

#include "esp_log.h"
#include "rm67162.h"
#include "setup_img.h"
#include "tzi_gui.h"
#include "ble.h"

#include <WiFi.h>
#include <WebServer.h>
#include <tz_wifi.h>

#include <TinyGPS++.h>
#include "ubloxConfigSentences.h"

TinyGPSPlus gps;

#include <BMP280_DEV.h>    
#include <SimpleKalmanFilter.h>
  float temperature=0.;
  float pressure = 101325;
  float altitude = 0.;            // Create the temperature, pressure and altitude variables
  uint16_t t_avg_vario = 0;
  uint16_t t_avg_vario_b = 0;
  uint16_t t_avg_vario_start = millis();
  uint16_t t_avg_vario_b_start = millis();
  uint16_t t_avg_n = 1;
  uint16_t t_avg_b_n = 1;
  float vario_mavg = 0.;
  float vario_mavg_b = 0.;
  SimpleKalmanFilter altKalmanFilter(0.2, 0.2, 0.05);
BMP280_DEV bmp280(BARO_SDA, BARO_SCL);   

// Battery
SimpleKalmanFilter voltKalmanFilter(0.1,0.1,0.1);
float voltages[VPARRAYSIZE] = {3500, 3600, 3700, 3800, 3900, 4000, 4050, 4200 };
uint8_t percentages[VPARRAYSIZE] = {  0,  20,  50,  80,  95,  98,   99, 100 };

#if ARDUINO_USB_CDC_ON_BOOT != 1
#warning "If you need to monitor printed data, be sure to set USB CDC On boot to ENABLE, otherwise you will not see any data in the serial monitor"
#endif

#ifndef BOARD_HAS_PSRAM
#error "Detected that PSRAM is not turned on. Please set PSRAM to OPI PSRAM in ArduinoIDE"
#endif

// init variables and functions

bool gotomainpage = true;
uint8_t bat_idx=4;

// Volumes
char* volumes_levels[NVOLS] = {   
    "#4f0a4f \xEF\x80\xA6#", // LV_SYMBOL_MUTE
    "#ff00ff \xEF\x80\xA7#", // LV_SYMBOL_VOLUME_MID
    "#ff00ff \xEF\x80\xA8#",  // LV_SYMBOL_VOLUME_MAX
};
uint16_t volumes[NVOLS] = { 0, 55, 255 }; 

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
void taskBuzzer(void *param);

void resetUBX();
void changeGpsHz();
void setSentences();

TaskHandle_t xHandleBluetooth = NULL;
TaskHandle_t xHandleGPSU7 = NULL;
TaskHandle_t xHandleBUZZER = NULL;

void sendUBX(const unsigned char *progmemBytes, size_t len );

RTC_DATA_ATTR int bootCount = 0;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf;
static uint32_t last_tick = 0;
static uint32_t last_tick_s = 0;
static uint32_t last_tick_b = 0;

static char* battery[7] = {
    LV_SYMBOL_BATTERY_EMPTY,
    LV_SYMBOL_BATTERY_1,
    LV_SYMBOL_BATTERY_2,
    LV_SYMBOL_BATTERY_3,
    LV_SYMBOL_BATTERY_FULL,
    LV_SYMBOL_CHARGE,
    LV_SYMBOL_USB};

struct statusData status;

SoftwareSerial ss(GPSRXPIN, GPSTXPIN);

bool nmeaReady = false;


OneButton button1(PIN_BUTTON_1, true);
OneButton button2(PIN_BUTTON_2, true);

void my_disp_flush(lv_disp_drv_t *disp,
                   const lv_area_t *area,
                   lv_color_t *color_p)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    lcd_PushColors(area->x1, area->y1, w, h, (uint16_t *)&color_p->full);
    lv_disp_flush_ready(disp);
}

uint16_t color565(uint8_t r, uint8_t g, uint8_t b)
{
    // return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    // RGB - > GBR
    return (r)  | (g & 0xF8 << 8) | ((b & 0xFC) << 3);
}


unsigned short rgb888_to_grb565(unsigned char r, unsigned char g, unsigned char b)
{
    unsigned short grb565 = 0;
    grb565 |= (g >> 2) << 11;
    grb565 |= (r >> 3) << 5;
    grb565 |= (b >> 3);
    return grb565;
}

unsigned short rgb888_to_brg565(unsigned char r, unsigned char g, unsigned char b)
{
    unsigned short brg565 = 0;
    brg565 |= (b >> 3) << 11;
    brg565 |= (r >> 3) << 5;
    brg565 |= (g >> 2);
    return brg565;
}

int getChecksum(char *buffer, size_t size){
    uint8_t chk = 0;
    uint16_t tSize = 0;   
    while(*buffer){
        buffer++; //skip first char
        chk ^= *buffer;
        tSize++;
    }
    tSize += snprintf(buffer,size-tSize,"*%02X",chk);
    return tSize;
}

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void writeConfiguration(){

  String newSettings = "{\n";

  newSettings += "\"wifi_ssid\":" + String(status.wifi_ssid) + ",\n";
  newSettings += "\"wifi_psw\":" + String(status.wifi_psw) + ",\n";
  newSettings += "\"volume\":" + String(status.volume) + ",\n";
  newSettings += "\"gps_hz\":" + String(status.gps_hz) + ",\n";
  newSettings += "\"min_sat_av\":" + String(status.min_sat_av) + ",\n";
  newSettings += "\"rotation\":" + String(status.rotation) + ",\n";
  newSettings += "\"vario_sink_on\":" + String(status.vario_sink_on) + ",\n";
  newSettings += "\"vario_lift_on\":" + String(status.vario_lift_on) + ",\n";
  newSettings += "\"vario_avg_ms\":" + String(status.vario_avg_ms) + ",\n";
  newSettings += "\"vario_avg_ms_b\":" + String(status.vario_avg_ms_b) + ",\n";


  newSettings += "\"vario_curve\":[\n\
    {\"vval\":10,\"frq\":2000,\"ton\":100,\"toff\":100},\n\
    {\"vval\":5,\"frq\":1500,\"ton\":200,\"toff\":100},\n\
    {\"vval\":0.2,\"frq\":400,\"ton\":150,\"toff\":400},\n\
    {\"vval\":0.1,\"frq\":120,\"ton\":50,\"toff\":140},\n\
    {\"vval\":-0.2,\"frq\":80,\"ton\":50,\"toff\":140},\n\
    {\"vval\":-0.25,\"frq\":80,\"ton\":20,\"toff\":20},\n\
    {\"vval\":-2.5,\"frq\":250,\"ton\":500,\"toff\":1000},\n\
    {\"vval\":-10,\"frq\":200,\"ton\":500,\"toff\":1000}\n\
  ],\n";

  String td = status.thermal_detect ? "true" : "false";
  newSettings += "\"thermal_detect\":" + td + ",\n";
  newSettings += "\"thermal_avg\":" + String(status.thermal_avg) + "\n";
  newSettings += "}";

  File sfile = SPIFFS.open(SETTINGS_FileName, FILE_WRITE);
  if (!sfile) {
    log_e("There was an error opening the file for writing");
    return;
  }
  if (sfile.print(newSettings.c_str())) {
    log_i("Setting Updated.");
  } else {
    log_e("File write failed");
  }

  sfile.close();
}

void loadConfiguration(){

    File jsonsettings_file = SPIFFS.open(SETTINGS_FileName);
    if(!jsonsettings_file){
      writeConfiguration();
    }else{
      log_i("Reading settings from: %s",SETTINGS_FileName);
      String fileContent;
      while(jsonsettings_file.available()){
        fileContent += String((char)jsonsettings_file.read());
      }
      delay(10);
      log_i("json: %s",fileContent.c_str());

      status.jsonSettings = JSON.parse(fileContent);
      delay(10);

      uint32_t intValue; 
      bool boolValue;
      char charValue[20];
      if(status.jsonSettings.hasOwnProperty(String("volume"))){
        // Serial.print("Volume from Settings: "); 
        intValue = status.jsonSettings["volume"];
        // Serial.println(intValue);
        status.volume_idx = intValue < 0 ? 0: intValue > 2 ? 2 : intValue;
        status.volume = volumes[status.volume_idx];
        log_i("Volume from Settings: %lu",intValue); 
      }else{
        log_e("key 'volume' not found. Rewrite configurations...");
        SPIFFS.remove(SETTINGS_FileName);
        delay(10);
        writeConfiguration();
        delay(1000);
        ESP.restart();
      }

      if(status.jsonSettings.hasOwnProperty(String("wifi_ssid"))){
        strcpy(charValue,status.jsonSettings["wifi_ssid"]);
        strcpy(status.wifi_ssid,charValue);
        log_i("WiFi ssid: %s",charValue);
      }

      if(status.jsonSettings.hasOwnProperty(String("wifi_psw"))){
        strcpy(charValue,status.jsonSettings["wifi_psw"]);
        strcpy(status.wifi_psw,charValue);
        log_i("WiFi psw: %s",charValue);
      }

      if(status.jsonSettings.hasOwnProperty(String("gps_hz"))){
        // Serial.print("Gps Hz from Settings: "); 
        intValue = status.jsonSettings["gps_hz"];
        // Serial.println(intValue);
        status.gps_hz = intValue;
        log_i("Gps Hz from Settings: %lu",intValue); 
      }

      if(status.jsonSettings.hasOwnProperty(String("min_sat_av"))){
        // Serial.print("Min Sat available for valid nmea: ");
        intValue = status.jsonSettings["min_sat_av"];
        // Serial.println(intValue);
        status.min_sat_av = intValue;
        log_i("Min Sat available for valid nmea: %lu",intValue);
      }

      if(status.jsonSettings.hasOwnProperty(String("rotation"))){
        // Serial.print("Display rotation: ");
        intValue = status.jsonSettings["rotation"];
        // Serial.println(intValue);
        status.rotation = intValue;
        log_i("Display rotation: %lu",intValue);
      }

      if(status.jsonSettings.hasOwnProperty(String("sink_on"))){
        status.vario_sink_on = (double)status.jsonSettings["vario_sink_on"];
        // Serial.print("Vario Sink on: ");
        // Serial.println(status.vario_sink_on);
        log_i("Vario Sink on: %f",status.vario_sink_on);
      }

      if(status.jsonSettings.hasOwnProperty(String("vario_lift_on"))){
        status.vario_lift_on = (double)status.jsonSettings["vario_lift_on"];
        // Serial.print("Vario Lift on: ");
        // Serial.println(status.vario_lift_on);
        log_i("Vario Lift on: %f",status.vario_lift_on);
      }

      if(status.jsonSettings.hasOwnProperty(String("vario_avg_ms"))){
        status.vario_avg_ms = status.jsonSettings["vario_avg_ms"];
        // Serial.print("vario_avg_ms: ");
        // Serial.println(status.vario_avg_ms);
        log_i("vario_avg_ms: %lu",status.vario_avg_ms);
      }
      if(status.jsonSettings.hasOwnProperty(String("vario_avg_ms_b"))){
        status.vario_avg_ms_b = status.jsonSettings["vario_avg_ms_b"];
        log_i("vario_avg_ms_b: %lu",status.vario_avg_ms_b);
      }
        
      if(status.jsonSettings.hasOwnProperty(String("vario_curve"))){
        // Serial.println("Vario curve:");
        log_i("Vario curve:");
        for (int i=0;i<status.jsonSettings["vario_curve"].length();i++){
          float vval = (double)status.jsonSettings["vario_curve"][i]["vval"];
          int frq = status.jsonSettings["vario_curve"][i]["frq"];
          int ton = status.jsonSettings["vario_curve"][i]["ton"];
          int toff = status.jsonSettings["vario_curve"][i]["toff"];
          // Serial.print("  vval: ");Serial.print(vval);
          // Serial.print("  frq: ");Serial.print(frq);
          // Serial.print("  ton: ");Serial.print(ton);
          // Serial.print("  toff: ");Serial.println(toff);
          log_i("vval: %f frq: %d ton: %d toff %d",vval,frq,ton,toff);
        }
      }

      if(status.jsonSettings.hasOwnProperty(String("thermal_detect"))){
        boolValue = status.jsonSettings["thermal_detect"];
        status.thermal_detect = boolValue;
        log_i("thermal_detect: %d",boolValue);
      }
      if(status.jsonSettings.hasOwnProperty(String("thermal_avg"))){
        status.thermal_avg = status.jsonSettings["thermal_avg"];
        log_i("thermal_avg: %lu",status.thermal_avg);
      }

    jsonsettings_file.close();

  }
  status.settings_loaded = true;

}

void setup()
{

    Serial.begin(115200);

    delay(2000);
    ++bootCount;
    log_i("T-DISPLAY-S3-AMOLED FACTORY TEST");
    pinMode(PIN_BAT_VOLT, ANALOG);

    log_i("Boot number: %s" ,String(bootCount));
    //Print the wakeup reason for ESP32
    print_wakeup_reason();

    uint32_t chipId = 0;
    for(int i=0; i<17; i=i+8) {
      chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
    }
    log_i("ESP32 Chip model = %s Rev %d", ESP.getChipModel(), ESP.getChipRevision());
    log_i("This chip has %d cores\n", ESP.getChipCores());
    log_i("Chip ID: %i",chipId); 

    status.chipId = chipId;

    // check version
    if(!SPIFFS.begin(true)){
      log_e("An Error has occurred while mounting SPIFFS");
    }else{
      File file = SPIFFS.open("/version.json");
      if(!file){
        log_e("Failed to open file for reading");
      }else{
        String fileContent;
        while(file.available()){
          fileContent += String((char)file.read());
        }
        JSONVar myObject = JSON.parse(fileContent);
        JSONVar keys = myObject.keys();
        //JSONVar value = myObject["datetime"];
        //wifi_date = JSON.stringify(value);
        uint16_t ver = myObject["version"];
        status.firmware_v = ver;
        log_i("Firmware Version v.%i", status.firmware_v);
        file.close();
      }
    }

    // check if updated 
    if (status.firmware_v < VERSION){
        File file = SPIFFS.open("/version.json", FILE_WRITE);
        if (!file) {
          log_e("There was an error opening the file for writing");
          return;
        }
        String fver = "{\"version\":" + String(VERSION) + "}";
        if (file.print(fver.c_str())) {
          log_i("Firmware verison updated to v.%i", VERSION);
          status.firmware_v = VERSION;
        } else {
          log_e("File write failed");
        }
      
        file.close();
    }

    delay(50);

    loadConfiguration();

    delay(50);

    pinMode(GPSRXPIN,INPUT);
    pinMode(GPSTXPIN,OUTPUT);
    ss.begin(GPSBAUD);
    delay(500);
    resetUBX();
    changeGpsHz();
    setSentences();
    delay(50);

    ledcDetachPin(SPERKER_PIN);

    rm67162_init(); // amoled lcd initialization
    lcd_setRotation(status.rotation);
    xTaskCreatePinnedToCore(led_task, "led_task", 1024, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(taskBuzzer, "taskBuzzer", 6500, NULL, 6, &xHandleBUZZER, 0); 
    xTaskCreatePinnedToCore(taskOledUpdate, "taskOledUpdate", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(taskBaro, "taskBaro", 6500, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(taskBluetooth, "taskBluetooth", 6500, NULL, 3, &xHandleBluetooth, 0);
    xTaskCreatePinnedToCore(taskGPSU7, "taskGPSU7", 7500, NULL, 5, &xHandleGPSU7, 1); 

    /*Initialize the display*/
    lv_init();
    buf = (lv_color_t *)ps_malloc(sizeof(lv_color_t) * LVGL_LCD_BUF_SIZE);
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, LVGL_LCD_BUF_SIZE);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);
   
    ui_begin();

    // BUTTONS
    button1.attachLongPressStop(
    []() {
        uint64_t mask = 1 << PIN_BUTTON_1;

        sing(1);
        lcd_sleep();

        log_i("Disable GPS");
        sendUBX( UBLOX_POWER_OFF, sizeof(UBLOX_POWER_OFF) );  
        delay(1000);

        esp_sleep_enable_ext1_wakeup(mask, ESP_EXT1_WAKEUP_ALL_LOW);
        esp_deep_sleep_start();
    }        
    );

    button2.attachDoubleClick(
        []() {
            ui_gotomain_page(status.mainpage);
            beepSpeaker();
            beepSpeaker();
        }
    );

    button2.attachLongPressStop(
        [](){
            status.wifi_on = !status.wifi_on;
            if (status.wifi_on){
                lv_msg_send(MSG_NEW_WIFI, &wfon);
                TzWifiBegin();

            }else{
                lv_msg_send(MSG_NEW_WIFI, &wfoff);
                TzWifiOff();
            }
            beepSpeaker();
        }
    );

    // init volume
    const char* nv = volumes_levels[status.volume_idx];
    status.volume = volumes[status.volume_idx];
    lv_msg_send(MSG_NEW_VOLUME, &nv);

    button1.attachClick(
       [](){ 
            status.volume_idx = (status.volume_idx+1)%3;
            const char* nv = volumes_levels[status.volume_idx];
            status.volume = volumes[status.volume_idx];
            lv_msg_send(MSG_NEW_VOLUME, &nv);
            beepSpeaker();
        }
    );

    button2.attachClick([]() {
        ui_switch_page_up();
        beepSpeaker();
    });

    // BUTTONS ---

}

void updateUi(){
  const char* nv = volumes_levels[status.volume_idx];
  status.volume = volumes[status.volume_idx];
  lv_msg_send(MSG_NEW_VOLUME, &nv);
}

uint8_t getBatteryPerc(){
  float bp = 0;
  float bv = 0;
  uint8_t vidx=0;
  for (uint8_t vix=0; vix<VPARRAYSIZE; vix++){
    if (status.batterymV >= voltages[vix]) {
      bv = voltages[vix];    
      vidx = vix;
    }
  }
  if (bv > 0) {
    if(vidx < VPARRAYSIZE -1){
      bp =  percentages[vidx]; 
      bp += (percentages[vidx+1]-percentages[vidx]) * ((status.batterymV - bv) / (voltages[vidx+1]-voltages[vidx]));
    }else{
      bp = 100.0;
    }
  }
  return (uint8_t)bp;
}

void loop()
{
    lv_timer_handler();
    delay(2);
    button1.tick();
    button2.tick();

    if (millis() - last_tick > 500) {
        uint32_t volt = (analogReadMilliVolts(PIN_BAT_VOLT) * 2);
        uint32_t volt_avg = voltKalmanFilter.updateEstimate(volt);
        status.batterymV = volt_avg; 
        status.battery = getBatteryPerc();
        // TEST
        //log_i("Volt read average (kalman): %.f",status.batterymV);
        char* nb = battery[4];
        if (volt_avg>=uint32_t(4200)){
            nb = battery[6];
        }else if (volt_avg>=uint32_t(4050)){
            nb = battery[5];
        }else if (volt_avg>=uint32_t(3900)){
            nb = battery[4];
        }else if (volt_avg >= uint32_t(3800)){
            nb = battery[3];
        }else if (volt_avg >=uint32_t(3700)){
            nb = battery[2];
        }else if (volt_avg >= uint32_t(3600)){
            nb = battery[1];
        }else{
            nb = battery[0];
        }
        lv_msg_send(MSG_NEW_BATTERY, &nb);

        lv_msg_send(MSG_NEW_VOLT, &volt_avg);

        last_tick = millis();
    }

    if (status.wifi_on){
      HandleMyClients();
    }

    // temporary fake setup done - all sensors ok - fake data
    // if (last_tick > 5000 & (!status.baro_ok)){
    //     status.baro_ok = true;
    //     lv_msg_send(MSG_NEW_BARO,&barook);
    //     Serial.print("Test baro done icon.");
    // }

    // if (last_tick > 6000 & (!status.GPS_Fix)){
    //     status.GPS_Fix = true;
    //     lv_msg_send(MSG_NEW_GPSFIX, &gpsfix);
    //     Serial.print("Test gpsfix done icon.");
    // }

    // if (last_tick > 7000 & (!status.BLE_connected)){
    //     status.BLE_connected = true;
    //     lv_msg_send(MSG_NEW_BLE, &bleon);
    //     Serial.print("Test BLE on icons.");
    // }

    // if (millis() - last_tick_b > 500){
    //     // bat_idx = (bat_idx+1)%5;
    //     static char* nb = battery[bat_idx];
    //     uint32_t volt = 3650;//(analogReadMilliVolts(PIN_BAT_VOLT) * 2);
    //     //char* nb = battery[0];
    //     if (volt>uint32_t(4050)){
    //         nb = battery[4];
    //     }else if (volt >= uint32_t(3800)){
    //         nb = battery[3];
    //     }else if (volt >=uint32_t(3700)){
    //         nb = battery[2];
    //     }else if (volt >= uint32_t(3600)){
    //         nb = battery[1];
    //     }else{
    //         nb = battery[0];
    //     }
 
    //     lv_msg_send(MSG_NEW_BATTERY, &nb);
    //     last_tick_b = millis();
    // }

//    status.mainpage = random(0,4);

    // end of temporary fake

    // char sOut[MAXSTRING];
    // int pos = 0;
    // pos += snprintf(&sOut[pos],MAXSTRING-pos,"$LK8EX1,");
    // pos += snprintf(&sOut[pos],MAXSTRING-pos,"%d,%d,",(status.pressure*100),status.elevation);
    // pos += snprintf(&sOut[pos],MAXSTRING-pos,"%d,999,",(status.vario_dcm*10));
    // pos += snprintf(&sOut[pos],MAXSTRING-pos,"%lu,",status.battery + 1000);
    // //      pos += snprintf(&sOut[pos],MAXSTRING-pos,"%.02f,",status.batterymV);
    // pos = getChecksum(sOut,MAXSTRING);
    // //strcat(sOut,"\r\n0");
    // status.LK8EX1_s = String(sOut);    

    if (status.BLE_connected & gotomainpage){
        ui_gotomain_page(status.mainpage);
        gotomainpage = false;
        lv_msg_send(MSG_NEW_BLE, &bleon);
    }
    if (!status.BLE_connected){
        lv_msg_send(MSG_NEW_BLE, &bleoff);
    }

    if (status.GPS_course > 337.5f || status.GPS_course <= 22.5f){
      strcpy(status.GPS_course_c,"N");
    }else if(status.GPS_course > 22.5f && status.GPS_course <= 67.5f){
      strcpy(status.GPS_course_c,"NE");
    }else if(status.GPS_course > 67.5f && status.GPS_course <= 112.5f){
      strcpy(status.GPS_course_c,"E");
    }else if(status.GPS_course > 112.5f && status.GPS_course <= 157.5f){
      strcpy(status.GPS_course_c,"SE");
    }else if(status.GPS_course > -157.5f && status.GPS_course <= 202.5f){
      strcpy(status.GPS_course_c,"S");
    }else if(status.GPS_course > 202.5f && status.GPS_course <= 247.5f){
      strcpy(status.GPS_course_c,"SW");
    }else if(status.GPS_course > 247.5f && status.GPS_course <= 292.5f){
      strcpy(status.GPS_course_c,"W");
    }else if(status.GPS_course > 292.5f && status.GPS_course <= 337.5f){
      strcpy(status.GPS_course_c,"NW");
    }

}

void led_task(void *param)
{
        pinMode(PIN_LED, OUTPUT);
        while (1) {
            if (!(status.baro_ok && status.GPS_Fix)){
                digitalWrite(PIN_LED, 1);
                delay(20);
                digitalWrite(PIN_LED, 0);
                delay(980);
            // here all else if for each led action
            }else{
                delay(5);
            }
        }
}

uint32_t interpolate(float vval, const char *field){
  uint32_t interpolated = 0;

  int s = status.jsonSettings["vario_curve"].length();
  float v1 = 0.;
  float v2 = 0.;
  uint32_t f1 = 0;
  uint32_t f2 = 0;
  bool minFound = false;
  bool maxFound = false;

  // NOTE vval should be ordered from max to min eg. 10 m/s to -10 m/s

  // check less than minimum return last minimum
  if (vval < (double)status.jsonSettings["vario_curve"][s-1]["vval"]) {
    return status.jsonSettings["vario_curve"][s-1][field];
  }

  // check greater than maximum return first maximum
  if (vval > (double)status.jsonSettings["vario_curve"][0]["vval"]) {
    return status.jsonSettings["vario_curve"][0][field];
  }

  // if in between values return interpolated value
  for (int i =0;i<s;i++){
    float v = (double)status.jsonSettings["vario_curve"][i]["vval"];
    if (vval < v && !minFound){ 
      v1 = v; 
      f1 = status.jsonSettings["vario_curve"][i][field];
    }else{ minFound = true;}
    if (vval > v && !maxFound){ 
      v2 = v;
      f2 = status.jsonSettings["vario_curve"][i][field];
      maxFound = true;
    }
    // if exact match return exact value
    if (v == vval){
      return status.jsonSettings["vario_curve"][i][field];
    }
  }
  float ratio = (vval - v1)/(v2 - v1);
  int sign = (float)f2 - (float)f1 >= 0 ? 1 : -1;
  interpolated = f1 + sign * ratio * abs(((float)f2 - (float)f1)); 

  return interpolated;
}


void taskBuzzer(void *pvParameters){
  // Serial.println("Test Speaker");
  ledcSetup(LEDC_CHANNEL_0, 1000, 8);
  ledcAttachPin(SPERKER_PIN, LEDC_CHANNEL_0);
  bool von = false;

  delay(1000);
  sing(0);
  delay(3000);

  while(1){

    if (status.lowPower){
      sing(3);
      delay(500);
//      status.lowPower = false;
    }else{

      int fr = 0;
      uint32_t son = 50; //ms
      uint32_t soff = 50; //ms

      // if volume is 0 then do not beep
      if (status.volume == 0) von=false;

      if (status.volume > 0 && von == false){
        von = true;
        ledcSetup(LEDC_CHANNEL_0, 1000, 8);
        ledcAttachPin(SPERKER_PIN, LEDC_CHANNEL_0);
      }
      if (von){

        fr = interpolate(status.vario_avg_b, "frq");
        son = interpolate(status.vario_avg_b, "ton");
        soff = interpolate(status.vario_avg_b, "toff");



        bool beep = (status.vario_avg_b <= status.vario_sink_on || status.vario_avg_b >= status.vario_lift_on);
        if (beep){
            ledcWriteTone(LEDC_CHANNEL_0, fr);
            ledcWrite(LEDC_CHANNEL_0, status.volume);
            delay(son);
            ledcWriteTone(LEDC_CHANNEL_0, 0);
        }else{
            ledcWriteTone(LEDC_CHANNEL_0, 0);
            ledcWrite(LEDC_CHANNEL_0, 0);
            delay(50);
          }
        
      }else{
        ledcDetachPin(SPERKER_PIN);
      }

      delay(soff);
    }
    // if (status.lowPower || status.updating) break;
    if (status.lowPower) break;
  }

  status.volume = 0;
  ledcDetachPin(SPERKER_PIN);

  // log_i("stop task BUZZER");
  // Use the handle to delete the task.
     if( xHandleBUZZER != NULL )
     {
        //  Serial.println("stop task BUZZER");
         log_i("stop task BUZZER");
         vTaskDelete( xHandleBUZZER );
     }
}


// Display vario avg value
float average_vario(float vario){

  uint16_t avg_msv = status.thermalling && status.thermal_detect ? status.thermal_avg : status.vario_avg_ms;

  if (avg_msv < 100) return vario;
  float avg_vario = 0.0;
  if (t_avg_vario < avg_msv){
    vario_mavg += vario;
    t_avg_n += 1;
    t_avg_vario = millis() - t_avg_vario_start;
    return status.vario_avg;
  }else{
    avg_vario = vario_mavg/(float)t_avg_n;
    t_avg_n = 0;
    t_avg_vario = 0;
    t_avg_vario_start = millis();
    vario_mavg = 0.0;
  }
  return avg_vario;
}

// Buzzer vario avg value
float average_vario_b(float vario){

  uint16_t avg_msv = status.thermalling && status.thermal_detect ? status.thermal_avg : status.vario_avg_ms_b;

  if (avg_msv < 100) return vario;
  float avg_vario = 0.0;
  if (t_avg_vario_b < avg_msv){
    vario_mavg_b += vario;
    t_avg_b_n += 1;
    t_avg_vario_b = millis() - t_avg_vario_b_start;
    return status.vario_avg_b;
  }else{
    avg_vario = vario_mavg_b/(float)t_avg_b_n;
    t_avg_b_n = 0;
    t_avg_vario_b = 0;
    t_avg_vario_b_start = millis();
    vario_mavg_b = 0.0;
  }
  return avg_vario;
}

void taskBaro(void *param)
{

  float prev_altitude = 0.f;
  long start_time = millis();
  if(bmp280.begin(0x76)){
      // Serial.println("BMP280 setup done.");
      log_i("BMP280 setup done.");
      bmp280.setTimeStandby(TIME_STANDBY_62MS);     // Set the standby time to 2 seconds
      bmp280.startNormalConversion();                 // Start BMP280 continuous conversion in NORMAL_MODE
      status.baro_ok = true;
  }else{
      // Serial.println("BMP280 not found!");
      log_e("BMP280 not found!");
      status.baro_ok = false;
  }

//    uint8_t ti = 0;
    while (1) {
//        delay(50);
        // status.vario_dcm = status.vario_dcm+1 > 10 ? -10 : status.vario_dcm+1;
        // status.elevation = ti==20 ? status.elevation + 1 : status.elevation;
        // ti = ti + 1 > 20 ? 0 : ti + 1;

//        log_i("Elevation: %i - Vario: %i",status.elevation,status.vario_dcm);
        // TODO - read from bmp280 data ...

    if (bmp280.getMeasurements(temperature, pressure, altitude)){

      status.pressure = pressure;
      //status.temperature = temperature;

      float estimated_alt = altKalmanFilter.updateEstimate(altitude);
      long dt = millis()-start_time;
      float vario = 1000.0f / dt * ( estimated_alt - prev_altitude );

      prev_altitude = estimated_alt;
      start_time = millis();

      status.altitude = estimated_alt;
      status.vario = vario;
      status.vario_avg = average_vario(vario);
      status.vario_dcm = int(status.vario_avg*10);
      status.vario_avg_b = average_vario_b(vario);

      //log_i("vario_b %.02f",status.vario_avg_b);

      //$LK8EX1,101300,99999,99999,99,999,
      //LK8EX1,pressure,altitude,vario,temperature,battery,*checksum
      char sOut[MAXSTRING];
      int pos = 0;
      pos += snprintf(&sOut[pos],MAXSTRING-pos,"$LK8EX1,");
      pos += snprintf(&sOut[pos],MAXSTRING-pos,"%.f,%.f,",(status.pressure*100.0),status.altitude);
      pos += snprintf(&sOut[pos],MAXSTRING-pos,"%.f,999,",(status.vario * 100.0));
      pos += snprintf(&sOut[pos],MAXSTRING-pos,"%lu,",status.battery + 1000);
//      pos += snprintf(&sOut[pos],MAXSTRING-pos,"%.02f,",status.batterymV);
      pos = getChecksum(sOut,MAXSTRING);
      //strcat(sOut,"\r\n0");
      status.LK8EX1_s = String(sOut);

    }
    
    // if (status.lowPower || status.updating) break;
    if (status.lowPower) break;
    delay(10);

    }
}

void taskOledUpdate(void *param)
{
    while (1){
        delay(250);

        int32_t b = 3599 - int(status.GPS_course*10.); //bussola;
        lv_msg_send(MSG_NEW_BUSS, &b);
        int32_t bd = int(status.GPS_course*10.)/10;
        lv_msg_send(MSG_NEW_BUSS_D, &bd);

        uint32_t vda = abs(status.vario_dcm);
        lv_msg_send(MSG_NEW_VARIO, &vda);

        char vm[20];
        int32_t vd = status.vario_dcm;
        if (vd>=0){
            lv_msg_send(MSG_NEW_VARIO_PM, &pv);
            lv_msg_send(MSG_NEW_BG_COLOR, &pbg);
            strcpy(vm,"+");
        }else{
            lv_msg_send(MSG_NEW_VARIO_PM, &mv);
            lv_msg_send(MSG_NEW_BG_COLOR, &mbg);
            strcpy(vm,"-");
        }
        String vmc = String(abs(status.vario),1);
        strcat(vm, vmc.c_str());
        const char* vmp = vm;
        lv_msg_send(MSG_NEW_VARIO_M, &vmp);


        if (status.GPS_Fix && status.GPS_sat >= status.min_sat_av){
          lv_msg_send(MSG_NEW_GPSFIX, &gpsfix);
          int32_t el = +status.GPS_alt;
          lv_msg_send(MSG_NEW_ELEV, &el);
          char *course = status.GPS_course_c;
          lv_msg_send(MSG_NEW_NESW,&course);
          uint32_t nsat = status.GPS_sat;
          lv_msg_send(MSG_NEW_NSAT, &nsat);
        }else{
          lv_msg_send(MSG_NEW_GPSFIX, &gpsnofix);
          int32_t el = int(status.altitude);
          lv_msg_send(MSG_NEW_ELEV, &el);
          const char *course = "fix";
          lv_msg_send(MSG_NEW_NESW,&course);
          uint32_t nsat = 0;
          lv_msg_send(MSG_NEW_NSAT, &nsat);
        }

        if (status.baro_ok) lv_msg_send(MSG_NEW_BARO,&barook);

//        log_i("Elevation: %i - Vario: %i",el,vd);

        // if (status.lowPower || status.updating) break;
        if (status.lowPower) break;

    }

}

void taskBluetooth(void *param) {
  status.ble_init = false;

  while (1)
    {

        if(!status.ble_init){

          esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
          esp_bt_controller_enable(ESP_BT_MODE_BLE);    

          // Serial.println("BLE is ON, configuring ...");
          String ble_name = "TzI-" + String(status.chipId) + "-BLE";
          // Serial.println(ble_name);
          start_ble(ble_name);
          status.ble_init = true;
          log_i("BLE is ON, configured : %s",ble_name.c_str());
 //         delay(2000);
        }else{

          // only send if we have more than 31k free heap space.
          if (esp_get_free_heap_size()>BLE_LOW_HEAP)
          {
            if (status.NMEA_raw.length()>0 && status.nmeaReady)
              {
              BLESendChunks(status.NMEA_raw);
//              log_i("BLE NMEA: %s",status.NMEA_raw.c_str());
              Serial.println(status.NMEA_raw);
              status.nmeaReady = false;
              }
          }else
          {
            log_d( " BLE congested - Waiting - Current free heap: %d, minimum ever free heap: %d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
            log_d("CLEARING BLEOUT");
          }

          if (esp_get_free_heap_size()>BLE_LOW_HEAP){
            if (status.LK8EX1_s.length() > 0){
              BLESendChunks(status.LK8EX1_s);
              delay(5);
              }
          }else
          {
            log_d( " BLE congested - Waiting - Current free heap: %d, minimum ever free heap: %d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
            log_d("CLEARING BLEOUT");
          }
        }

      // if (status.lowPower || status.updating) break;
      if (status.lowPower) break;
      delay(50);
    };

  if (status.ble_init){
    stop_ble();
    status.ble_init = false;
  }

// Use the handle to delete the task.
     if( xHandleBluetooth != NULL )
     {
          // Serial.println("stop task BLE");
          log_i("stop task BLE");
         vTaskDelete( xHandleBluetooth );
     }
}

void sendUBX(const unsigned char *progmemBytes, size_t len )
{
  ss.write( 0xB5 ); // SYNC1
  ss.write( 0x62 ); // SYNC2

  uint8_t a = 0, b = 0;
  while (len-- > 0) {
    uint8_t c = pgm_read_byte( progmemBytes++ );
    a += c;
    b += a;
    ss.write( c );
  }
  ss.write( a ); // CHECKSUM A
  ss.write( b ); // CHECKSUM B
} // sendUBX

void resetUBX(){
    sendUBX( ubxReset, sizeof(ubxReset) );
    delay(500);
}

void setSentences(){

  // XCTrack supported sentences $GPRMC, $GNRMC, $GPGGA, $GNGGA

  //    Serial.println("Disable NMEA GSA, GSV, VTG, ZDA");

    log_i("Disable NMEA GLL");
     sendUBX( ubxDisableGLL, sizeof(ubxDisableGLL) );  
     delay(200);

    // log_i("Disable NMEA GGA");
    //  sendUBX( ubxDisableGGA, sizeof(ubxDisableGGA) );  
    //  delay(200);

    log_i("Disable NMEA GSA");
     sendUBX( ubxDisableGSA, sizeof(ubxDisableGSA) );  
     delay(200);

    // log_i("Disable NMEA RMC");
    //  sendUBX( ubxDisableRMC, sizeof(ubxDisableRMC) );  
    //  delay(200);

    log_i("Disable NMEA GSV");
     sendUBX( ubxDisableGSV, sizeof(ubxDisableGSV) );  
     delay(200);

   log_i("Disable NMEA VTG");
    sendUBX( ubxDisableVTG, sizeof(ubxDisableVTG) );  
    delay(200);

   log_i("Disable NMEA ZDA");
    sendUBX( ubxDisableZDA, sizeof(ubxDisableZDA) );    
    delay(200);

    sendUBX( ubxReset, sizeof(ubxReset) );
    delay(500);
}

void changeGpsHz(){

  log_i("Change Hz mode switch to %lu Hz",status.gps_hz);
  switch (status.gps_hz){
      case 1:
        sendUBX( ubxRate1Hz, sizeof(ubxRate1Hz) );
        break;
      case 5:
        sendUBX( ubxRate5Hz, sizeof(ubxRate5Hz) );
        break;
      case 10:
        sendUBX( ubxRate10Hz, sizeof(ubxRate10Hz) );
        break;
      case 16:
        sendUBX( ubxRate16Hz, sizeof(ubxRate16Hz) );
        break;
      default:
        sendUBX( ubxRate1Hz, sizeof(ubxRate1Hz) );        
    }

    delay(200);

}

void taskGPSU7(void *param){

    // wait some time to init ss serial to gps
    delay(3000);
    //Serial.println("Resetting ublox");

//    log_i("Resetting ublox");
//    sendUBX( ubxReset, sizeof(ubxReset) );
//    delay(500);

    // Serial.print("Change Hz switch to ");
    // Serial.print(status.gps_hz);
    // Serial.println("Hz mode.");
    //changeGpsHz();
//    Serial.flush();

  String readString = "$";

  status.restart_GPS = false;
  status.resetGpsHz = false;

  while(1){

//   status.bussola = status.bussola +1 > 359 ? 0 : status.bussola + 1;

    if (status.restart_GPS){
      // sendUBX( ubxReset, sizeof(ubxReset) );
      // delay(500);
      resetUBX();
      delay(500);
      status.restart_GPS = false;
      //Serial.println("Restarting GPS module.");
      log_i("Restarting GPS module.");
      //delay(500);
    }

    if (status.resetGpsHz){
      changeGpsHz();
      //Serial.println("Resetting GPS frq Hz");
      log_i("Resetting GPS frq Hz");
      delay(500);
      status.resetGpsHz = false;
    }

    while (ss.available() > 0 ){
      char c = ss.read();
      if (gps.encode(c)){
        if(gps.location.isValid()){
          status.GPS_Fix = true;
          status.GPS_sat = gps.satellites.value(); 
          status.GPS_Lat = gps.location.lat();
          status.GPS_Lon = gps.location.lng();
          status.GPS_alt = gps.altitude.meters();
          status.GPS_year = gps.date.year();
          status.GPS_month = gps.date.month();
          status.GPS_day = gps.date.day();
          status.GPS_hour = gps.time.hour();
          status.GPS_min = gps.time.minute();
          status.GPS_sec = gps.time.second();
          status.GPS_speed = gps.speed.kmph();
          status.GPS_course = gps.course.deg();
        }else{
          status.GPS_Fix = false;
          break;
        }     
      }

      if(c!= '\r' && c != '\n' && c != '$') {
          readString += c;
        }else{
          if (c == '$'){
            //if( readString.indexOf("GPGGA") > 0  || readString.indexOf("GPRMC") > 0){
            if (readString.indexOf("GNTXT") >0 ){
              log_e("GPS: %s",readString.c_str());
            }else{
              status.NMEA_raw = readString;                
              nmeaReady = true;
            }            
            //}//else{ble_data = "";}
              delay(1);
              readString="$";              
          }//else{readString = "";}
        }

    }
       
    if (millis() > 30000 && gps.charsProcessed() < 10)
    {
      Serial.println(F("No GPS detected: check wiring."));  
      log_e("No GPS detected: check wiring.");
      break;
    }

    // if (status.lowPower || status.updating) break;
    if (status.lowPower) break;
    delay(10);
  }

  ss.end();
  // log_i("stop task GPSU7");
  // Serial.println("stop task GPSU7");
  log_i("stop task GPSU7");
  vTaskDelete(xHandleGPSU7);
}