#include "OneButton.h" /* https://github.com/mathertel/OneButton.git */
#include "lvgl.h"      /* https://github.com/lvgl/lvgl.git */

#include <Arduino.h>
#include "altivario.h"
#include "rm67162.h"
#include "setup_img.h"
#include "tzi_gui.h"

#if ARDUINO_USB_CDC_ON_BOOT != 1
#warning "If you need to monitor printed data, be sure to set USB CDC On boot to ENABLE, otherwise you will not see any data in the serial monitor"
#endif

#ifndef BOARD_HAS_PSRAM
#error "Detected that PSRAM is not turned on. Please set PSRAM to OPI PSRAM in ArduinoIDE"
#endif

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

void setup()
{
    elevation = 1017;
    vario_dcm = -10;
    bussola = 0;

    Serial.begin(115200);
    Serial.println("T-DISPLAY-S3-AMOLED FACTORY TEST");
    pinMode(PIN_BAT_VOLT, ANALOG);

    rm67162_init(); // amoled lcd initialization
    lcd_setRotation(1);
    xTaskCreatePinnedToCore(led_task, "led_task", 1024, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(elev_task, "elev_task", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(vario_task, "vario_task", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(gps_task, "gps_task", 4096, NULL, 3, NULL, 1);

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
            ui_gotomain_page();
        }
    );

    button2.attachDuringLongPress(
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

    if (last_tick > 10000 & (!status.ble_on)){
        status.ble_on = true;
        lv_msg_send(MSG_NEW_BLE, &bleon);
        lv_msg_send(MSG_NEW_GPSFIX, &gpsfix);
        Serial.print("Test BLE on and GPS fix icons.");
    }

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


    // end of temporary fake

    if (status.ble_on && gotomainpage){
        ui_gotomain_page();
        gotomainpage = false;
    }

}

void led_task(void *param)
{
        pinMode(PIN_LED, OUTPUT);
        while (1) {
            if (!status.ble_on){
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

    }
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
