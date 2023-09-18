#pragma once

#include "Arduino.h"

#define UI_BG_COLOR    lv_color_black()
#define UI_FRAME_COLOR lv_color_hex(0x282828)
#define UI_FONT_COLOR  lv_color_white()
#define UI_PAGE_COUNT  4
#define UI_DM_COLOR lv_color_hex(0x5d750c)
#define UI_DMM_COLOR lv_color_hex(0xeb1c1c)

#define MSG_NEW_HOUR   1
#define MSG_NEW_ELEV    2
#define MSG_NEW_VOLT   3
#define MSG_NEW_TOUCH_POINT   4
#define MSG_NEW_VOLUME 5
#define MSG_NEW_BATTERY 6
#define MSG_NEW_WIFI 7
#define MSG_NEW_BLE 8
#define MSG_NEW_VARIO 9
#define MSG_NEW_VARIO_PM 10
#define MSG_NEW_BUSS 11
#define MSG_NEW_BUSS_D 12
#define MSG_NEW_BG_COLOR 13
#define MSG_NEW_GPSFIX 14
#define MSG_NEW_BARO 15

#define LV_DELAY(x)                                                                                                                                  \
  do {                                                                                                                                               \
    uint32_t t = x;                                                                                                                                  \
    while (t--) {                                                                                                                                    \
      lv_timer_handler();                                                                                                                            \
      delay(1);                                                                                                                                      \
    }                                                                                                                                                \
  } while (0);

void ui_begin();
void ui_switch_page(void);
void ui_switch_page_up(void);
void ui_switch_page_down(void);
void ui_gotomain_page(uint8_t);