#include "OneButton.h" /* https://github.com/mathertel/OneButton.git */
#include "lvgl.h"      /* https://github.com/lvgl/lvgl.git */

#include <Arduino.h>
#include "altivario.h"
#include "rm67162.h"
#include "setup_img.h"
#include "tzi_gui.h"
#include "ble.h"

#if ARDUINO_USB_CDC_ON_BOOT != 1
#warning "If you need to monitor printed data, be sure to set USB CDC On boot to ENABLE, otherwise you will not see any data in the serial monitor"
#endif

#ifndef BOARD_HAS_PSRAM
#error "Detected that PSRAM is not turned on. Please set PSRAM to OPI PSRAM in ArduinoIDE"
#endif

RTC_DATA_ATTR int bootCount = 0;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf;
static uint32_t last_tick = 0;
static uint32_t last_tick_s = 0;
static uint32_t last_tick_b = 0;

static uint16_t elevation;
//static uint16_t vario_display;
static int16_t vario_dcm;
static int16_t bussola;

static char* battery[5] = {
    LV_SYMBOL_BATTERY_EMPTY,
    LV_SYMBOL_BATTERY_1,
    LV_SYMBOL_BATTERY_2,
    LV_SYMBOL_BATTERY_3,
    LV_SYMBOL_BATTERY_FULL};

struct statusData status;

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

void setup()
{
    elevation = 1017;
    vario_dcm = -10;
    bussola = 0;

    Serial.begin(115200);

    delay(1000);
    ++bootCount;
    Serial.println("T-DISPLAY-S3-AMOLED FACTORY TEST");
    pinMode(PIN_BAT_VOLT, ANALOG);

    Serial.println("Boot number: " + String(bootCount));
    //Print the wakeup reason for ESP32
    print_wakeup_reason();

    uint32_t chipId = 0;
    for(int i=0; i<17; i=i+8) {
      chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
    }
    Serial.printf("ESP32 Chip model = %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
    Serial.printf("This chip has %d cores\n", ESP.getChipCores());
    Serial.print("Chip ID: "); 
    Serial.println(chipId);

    status.chipId = chipId;

    rm67162_init(); // amoled lcd initialization
    lcd_setRotation(1);
    xTaskCreatePinnedToCore(led_task, "led_task", 1024, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(elev_task, "elev_task", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(vario_task, "vario_task", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(gps_task, "gps_task", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(taskBluetooth, "taskBluetooth", 6500, NULL, 3, &xHandleBluetooth, 0);

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
   
    // WIFI 
    // WIFI ---

    ui_begin();

    // BUTTONS
    button1.attachLongPressStop(
    []() {
        uint64_t mask = 1 << PIN_BUTTON_1;
        lcd_sleep();
        esp_sleep_enable_ext1_wakeup(mask, ESP_EXT1_WAKEUP_ALL_LOW);
        esp_deep_sleep_start();
    }        
    );

    button2.attachDoubleClick(
        []() {
            ui_gotomain_page(status.mainpage);
        }
    );

    button2.attachLongPressStop(
        [](){
            status.wifi_on = !status.wifi_on;
            if (status.wifi_on){
                lv_msg_send(MSG_NEW_WIFI, &wfon);
            }else{
                lv_msg_send(MSG_NEW_WIFI, &wfoff);
            }
        }
    );

    button1.attachClick(
       [](){ 
            vol_idx = (vol_idx+1)%3;
            const char* nv = volumes[vol_idx];
            lv_msg_send(MSG_NEW_VOLUME, &nv);
        }
    );

    button2.attachClick([]() {
        ui_switch_page_up();
    });

    // BUTTONS ---

}

void loop()
{
    lv_timer_handler();
    delay(2);
    button1.tick();
    button2.tick();
    if (millis() - last_tick > 100) {
        uint32_t volt = (analogReadMilliVolts(PIN_BAT_VOLT) * 2);
        lv_msg_send(MSG_NEW_VOLT, &volt);

        last_tick = millis();
    }



    // temporary fake setup done - all sensors ok - fake data
    if (last_tick > 5000 & (!status.baro_ok)){
        status.baro_ok = true;
        lv_msg_send(MSG_NEW_BARO,&barook);
        Serial.print("Test baro done icon.");
    }

    if (last_tick > 6000 & (!status.gps_fix)){
        status.gps_fix = true;
        lv_msg_send(MSG_NEW_GPSFIX, &gpsfix);
        Serial.print("Test gpsfix done icon.");
    }

    // if (last_tick > 7000 & (!status.BLE_connected)){
    //     status.BLE_connected = true;
    //     lv_msg_send(MSG_NEW_BLE, &bleon);
    //     Serial.print("Test BLE on icons.");
    // }

    if (millis() - last_tick_b > 500){
        // bat_idx = (bat_idx+1)%5;
        static char* nb = battery[bat_idx];
        uint32_t volt = 3650;//(analogReadMilliVolts(PIN_BAT_VOLT) * 2);
        //char* nb = battery[0];
        if (volt>uint32_t(4050)){
            nb = battery[4];
        }else if (volt >= uint32_t(3800)){
            nb = battery[3];
        }else if (volt >=uint32_t(3700)){
            nb = battery[2];
        }else if (volt >= uint32_t(3600)){
            nb = battery[1];
        }else{
            nb = battery[0];
        }
 
        lv_msg_send(MSG_NEW_BATTERY, &nb);
        last_tick_b = millis();
    }

    status.mainpage = random(0,4);

    // end of temporary fake

    if (status.BLE_connected && gotomainpage){
        ui_gotomain_page(status.mainpage);
        gotomainpage = false;
    }

}

void led_task(void *param)
{
        pinMode(PIN_LED, OUTPUT);
        while (1) {
            if (!status.BLE_connected){
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

void elev_task(void *param)
{
    while (1) {
        delay(990);
        elevation+=1;
        uint16_t el = elevation;
        lv_msg_send(MSG_NEW_ELEV, &el);
    }        
}

void vario_task(void *param)
{
    while (1) {
        delay(250);
        vario_dcm = vario_dcm+1 > 10 ? -10 : vario_dcm+1;
        uint16_t vario_display = abs(vario_dcm);
        lv_msg_send(MSG_NEW_VARIO, &vario_display);

        if (vario_dcm>=0){
            lv_msg_send(MSG_NEW_VARIO_PM, &pv);
            lv_msg_send(MSG_NEW_BG_COLOR, &pbg);
        }else{
            lv_msg_send(MSG_NEW_VARIO_PM, &mv);
            lv_msg_send(MSG_NEW_BG_COLOR, &mbg);
        }

        // BLE output
        //$LK8EX1,101300,99999,99999,99,999,
        //LK8EX1,pressure,altitude,vario,temperature,battery,*checksum
        char sOut[MAXSTRING];
        int pos = 0;
        pos += snprintf(&sOut[pos],MAXSTRING-pos,"$LK8EX1,");
        pos += snprintf(&sOut[pos],MAXSTRING-pos,"%d,%d,",(status.pressure*100.0),status.elevation);
        pos += snprintf(&sOut[pos],MAXSTRING-pos,"%d,999,",(status.vario_dcm * 10.0));
        pos += snprintf(&sOut[pos],MAXSTRING-pos,"%lu,",status.battery + 1000);
        //      pos += snprintf(&sOut[pos],MAXSTRING-pos,"%.02f,",status.batterymV);
        pos = getChecksum(sOut,MAXSTRING);
        //strcat(sOut,"\r\n0");
        status.LK8EX1_s = String(sOut);    }
}

void gps_task(void *param)
{
    while (1){
        delay(50);
        bussola = bussola +10 > 3599 ? 0 : bussola + 10;
        int16_t b = 3599 - bussola;
        lv_msg_send(MSG_NEW_BUSS, &b);
        int16_t bd = bussola/10;
        lv_msg_send(MSG_NEW_BUSS_D, &bd);
    }
}

void taskBluetooth(void *pvParameters) {
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
          log_i("BLE is ON, configured : %s",ble_name);
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