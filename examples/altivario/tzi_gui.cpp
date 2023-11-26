#include "tzi_gui.h"

// TO ENABLE DEFAULT FONTS edit lv_config.h and enable specific font size
// eg. #define LV_FONT_MONTSERRAT_28 1

LV_FONT_DECLARE(font_alibaba);
LV_IMG_DECLARE(lilygo1_gif);
LV_IMG_DECLARE(tzi);
LV_IMG_DECLARE(elev_png);
LV_IMG_DECLARE(vario);
LV_IMG_DECLARE(bussola);

static lv_point_t line_points[] = {
    {80, 0},
    {536, 0}};

static void update_text_subscriber_cb(lv_event_t *e);
static void update_rotation_buss_cb(lv_event_t *e);
static void update_background_color_cb(lv_event_t *e);

static void timer_task(lv_timer_t *t);
static lv_obj_t *dis;

static uint8_t n;
static bool rotate = false;

static lv_style_t style_line;

void ui_switch_page(void)
{
//    static uint8_t n;
    n++;
    lv_obj_set_tile_id(dis, 0, n % UI_PAGE_COUNT, LV_ANIM_ON);
}

void ui_switch_page_up(void)
{
    n = n+1 < UI_PAGE_COUNT ? n+1 : 0;
    lv_obj_set_tile_id(dis, 0, n % UI_PAGE_COUNT, LV_ANIM_ON);
    rotate = true;
}

void ui_switch_page_down(void)
{
    n = n-1 >= 0 ? n - 1 : UI_PAGE_COUNT-1;
    lv_obj_set_tile_id(dis, 0, n, LV_ANIM_ON);
}

void ui_gotomain_page(uint8_t mp)
{
    lv_obj_set_tile_id(dis, 0, mp, LV_ANIM_ON);
    n = (uint8_t)mp;
}

void ui_toolbar_status(lv_obj_t* parent)
{
    lv_obj_t *batt_text = lv_label_create(parent);
    lv_obj_set_size(batt_text, elev_png.header.w, 50);
    lv_obj_align(batt_text, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_text_align(batt_text, LV_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(batt_text, &lv_font_montserrat_28, 0);
    lv_label_set_recolor(batt_text,true);
    lv_label_set_text_fmt(batt_text, "#ff00ff %s#",LV_SYMBOL_BATTERY_FULL);
    lv_obj_set_style_text_color(batt_text, UI_FONT_COLOR, 0);
    lv_obj_add_event_cb(batt_text,
                    update_text_subscriber_cb,
                    LV_EVENT_MSG_RECEIVED,
                    NULL);
    lv_msg_subsribe_obj(MSG_NEW_BATTERY, batt_text, (void *)"#ff00ff %s#");

    lv_obj_t *ble_text = lv_label_create(parent);
    lv_obj_set_size(ble_text, elev_png.header.w, 50);
    lv_obj_align(ble_text, LV_ALIGN_TOP_LEFT, 50, 0);
    lv_obj_set_style_text_align(ble_text, LV_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(ble_text, &lv_font_montserrat_28, 0);
    lv_label_set_recolor(ble_text,true);
    lv_label_set_text_fmt(ble_text, "#4f0a4f %s#",LV_SYMBOL_BLUETOOTH);
    lv_obj_set_style_text_color(ble_text, UI_FONT_COLOR, 0);
    lv_obj_add_event_cb(ble_text,
                    update_text_subscriber_cb,
                    LV_EVENT_MSG_RECEIVED,
                    NULL);
    lv_msg_subsribe_obj(MSG_NEW_BLE, ble_text, (void *)"%s");

    lv_obj_t *wifi_text = lv_label_create(parent);
    lv_obj_set_size(wifi_text, elev_png.header.w, 50);
    lv_obj_align(wifi_text, LV_ALIGN_TOP_LEFT, 90, 0);
    lv_obj_set_style_text_align(wifi_text, LV_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(wifi_text, &lv_font_montserrat_28, 0);
    lv_label_set_recolor(wifi_text,true);
    lv_label_set_text_fmt(wifi_text, "#4f0a4f %s#",LV_SYMBOL_WIFI);
    lv_obj_set_style_text_color(wifi_text, UI_FONT_COLOR, 0);
    lv_obj_add_event_cb(wifi_text,
                    update_text_subscriber_cb,
                    LV_EVENT_MSG_RECEIVED,
                    NULL);
    lv_msg_subsribe_obj(MSG_NEW_WIFI, wifi_text, (void *)"%s");

    lv_obj_t *audio_text = lv_label_create(parent);
    lv_obj_set_size(audio_text, elev_png.header.w, 50);
    lv_obj_align(audio_text, LV_ALIGN_TOP_RIGHT, 0, 0);
    lv_obj_set_style_text_align(audio_text, LV_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(audio_text, &lv_font_montserrat_28, 0);
    lv_label_set_recolor(audio_text,true);
    lv_label_set_text_fmt(audio_text, "#ff00ff %s#",LV_SYMBOL_VOLUME_MAX);
    lv_obj_set_style_text_color(audio_text, UI_FONT_COLOR, 0);
    lv_obj_add_event_cb(audio_text,
                        update_text_subscriber_cb,
                        LV_EVENT_MSG_RECEIVED,
                        NULL);
    lv_msg_subsribe_obj(MSG_NEW_VOLUME, audio_text, (void *)"%s");


    lv_obj_t *sat_text = lv_label_create(parent);
    lv_obj_set_size(sat_text, elev_png.header.w, 50);
    lv_obj_align(sat_text, LV_ALIGN_TOP_RIGHT, -50, 0);
    lv_obj_set_style_text_align(sat_text, LV_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(sat_text, &lv_font_montserrat_28, 0);
    lv_label_set_recolor(sat_text,true);
    lv_label_set_text_fmt(sat_text, "#4f0a4f %s#",LV_SYMBOL_GPS);
    lv_obj_set_style_text_color(sat_text, UI_FONT_COLOR, 0);
    lv_obj_add_event_cb(sat_text,
                    update_text_subscriber_cb,
                    LV_EVENT_MSG_RECEIVED,
                    NULL);
    lv_msg_subsribe_obj(MSG_NEW_GPSFIX, sat_text, (void *)"%s");

    lv_obj_t *baro_text = lv_label_create(parent);
    lv_obj_set_size(baro_text, elev_png.header.w, 50);
    lv_obj_align(baro_text, LV_ALIGN_TOP_RIGHT, -100, 0);
    lv_obj_set_style_text_align(baro_text, LV_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(baro_text, &lv_font_montserrat_28, 0);
    lv_label_set_recolor(baro_text,true);
    lv_label_set_text_fmt(baro_text, "#4f0a4f %s#",LV_SYMBOL_OK);
    lv_obj_set_style_text_color(baro_text, UI_FONT_COLOR, 0);   
    lv_obj_add_event_cb(baro_text,
                    update_text_subscriber_cb,
                    LV_EVENT_MSG_RECEIVED,
                    NULL);
    lv_msg_subsribe_obj(MSG_NEW_BARO, baro_text, (void *)"%s");
}

void ui_data_linetrough(lv_obj_t* parent){

    lv_obj_t *line;
    line = lv_line_create(parent);
    lv_line_set_points(line, line_points, 2);
    lv_obj_add_style(line, &style_line, 0);
    lv_obj_align(line, LV_ALIGN_CENTER, -10, 10);
}

void ui_begin()
{
    lv_style_init(&style_line);
    lv_style_set_line_width(&style_line, 4);
    lv_style_set_line_color(&style_line, UI_BG_COLOR);
    lv_style_set_line_rounded(&style_line, true);

    dis = lv_tileview_create(lv_scr_act());
    lv_obj_align(dis, LV_ALIGN_TOP_RIGHT, 0, 0);
    lv_obj_set_size(dis, LV_PCT(100), LV_PCT(100));
    lv_obj_remove_style(dis, 0, LV_PART_SCROLLBAR);

    lv_obj_t *tv1 = lv_tileview_add_tile(dis, 0, 0, LV_DIR_VER);
    lv_obj_t *tv2 = lv_tileview_add_tile(dis, 0, 1, LV_DIR_VER);
    lv_obj_t *tv3 = lv_tileview_add_tile(dis, 0, 2, LV_DIR_VER);
    lv_obj_t *tv4 = lv_tileview_add_tile(dis, 0, 3, LV_DIR_VER);


    /* page 1 */
    /// https://lvgl.io/tools/imageconverter
    /// LV_IMG_CF_RAW_CHROMA_KEYED
    lv_obj_t *logo_img = lv_gif_create(tv1);
    lv_obj_center(logo_img);
//    lv_gif_set_src(logo_img, &lilygo1_gif);
    lv_gif_set_src(logo_img, &tzi);
    ui_toolbar_status(tv1);

    lv_obj_t *bat_label = lv_label_create(tv1);
    lv_obj_align(bat_label, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_obj_set_style_text_font(bat_label, &lv_font_montserrat_28, 0);
    lv_obj_add_event_cb(bat_label,
                        update_text_subscriber_cb,
                        LV_EVENT_MSG_RECEIVED,
                        NULL);
    lv_msg_subsribe_obj(MSG_NEW_VOLT, bat_label, (void *)"%d mV");

    lv_obj_t *nsat_label = lv_label_create(tv1);
    lv_obj_align(nsat_label, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
    lv_obj_set_style_text_font(nsat_label, &lv_font_montserrat_28, 0);
    lv_obj_add_event_cb(nsat_label,
                        update_text_subscriber_cb,
                        LV_EVENT_MSG_RECEIVED,
                        NULL);
    lv_msg_subsribe_obj(MSG_NEW_NSAT, nsat_label, (void *)"%d sat");

    /* page 2 */
    // main container
    lv_obj_t *main_p2 = lv_obj_create(tv2);
    lv_obj_set_size(main_p2, LV_PCT(100), LV_PCT(100));
    lv_obj_clear_flag(main_p2, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(main_p2, 0, 0);
    lv_obj_set_style_bg_color(main_p2, UI_BG_COLOR, 0);

    lv_obj_t *p2_data = lv_obj_create(main_p2);
    lv_obj_set_size(p2_data, 460, 220);
    lv_obj_align(p2_data, LV_ALIGN_RIGHT_MID, 0, 10);
    lv_obj_set_style_bg_color(p2_data, UI_BG_COLOR, 0);
    lv_obj_set_style_border_color(p2_data, UI_BG_COLOR, 0);
    lv_obj_clear_flag(p2_data, LV_OBJ_FLAG_SCROLLABLE);

    // SMALL DATA
    lv_obj_t *p2_data_small = lv_obj_create(main_p2);
    lv_obj_set_size(p2_data_small, 200, 60);
    lv_obj_align(p2_data_small, LV_ALIGN_TOP_MID, 0, -20);
    lv_obj_set_style_bg_color(p2_data_small, UI_BG_COLOR, 0);
    lv_obj_set_style_border_color(p2_data_small, UI_BG_COLOR, 0);
    lv_obj_clear_flag(p2_data_small, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *c2_label_s = lv_label_create(main_p2);
    lv_obj_align(c2_label_s, LV_ALIGN_TOP_LEFT, 10, 30);
    lv_obj_set_style_text_align(c2_label_s, LV_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(c2_label_s, &lv_font_montserrat_42, 0);
    lv_label_set_text(c2_label_s, "NS");
    lv_obj_set_style_text_color(c2_label_s, UI_FONT_COLOR, 0);
    lv_obj_add_event_cb(c2_label_s,
                        update_text_subscriber_cb,
                        LV_EVENT_MSG_RECEIVED,
                        NULL);
    lv_msg_subsribe_obj(MSG_NEW_NESW, c2_label_s, (void *)"%s");

    lv_obj_t *ms_label_s = lv_label_create(p2_data_small);
    lv_obj_align(ms_label_s, LV_ALIGN_TOP_RIGHT, 0, -10);
    lv_obj_set_style_text_align(ms_label_s, LV_ALIGN_RIGHT_MID, 0);
    lv_obj_set_style_text_font(ms_label_s, &lv_font_montserrat_42, 0);
    lv_label_set_text(ms_label_s, "+1.2 m|s");
    lv_obj_set_style_text_color(ms_label_s, UI_FONT_COLOR, 0);
    lv_obj_add_event_cb(ms_label_s,
                        update_text_subscriber_cb,
                        LV_EVENT_MSG_RECEIVED,
                        NULL);
    lv_msg_subsribe_obj(MSG_NEW_VARIO_M, ms_label_s, (void *)"%s m|s");
    //END SMALL DATA

    lv_obj_t *elev_img = lv_img_create(main_p2);
    lv_obj_align(elev_img, LV_ALIGN_LEFT_MID, 0, 0);
    lv_img_set_src(elev_img,&elev_png);

    lv_obj_t *masl_text = lv_label_create(main_p2);
    lv_obj_set_size(masl_text, elev_png.header.w, 220);
    lv_obj_align(masl_text, LV_ALIGN_LEFT_MID, 10, 150);
    lv_obj_set_style_text_align(masl_text, LV_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(masl_text, &lv_font_montserrat_28, 0);
    lv_label_set_recolor(masl_text,true);
    lv_label_set_text(masl_text, "#55d400 m#");
    lv_obj_set_style_text_color(masl_text, UI_FONT_COLOR, 0);

    lv_obj_t *masl_label = lv_label_create(p2_data);
    lv_obj_center(masl_label);
    lv_obj_set_style_text_align(masl_label, LV_ALIGN_RIGHT_MID, 0);
    lv_obj_set_style_text_font(masl_label, &font_alibaba, 0);
    lv_label_set_text(masl_label, "0000");
    lv_obj_set_style_text_color(masl_label, UI_FONT_COLOR, 0);
    lv_obj_add_event_cb(masl_label,
                        update_text_subscriber_cb,
                        LV_EVENT_MSG_RECEIVED,
                        NULL);
    lv_msg_subsribe_obj(MSG_NEW_ELEV, masl_label, (void *)"%d");

    ui_data_linetrough(main_p2);
    ui_toolbar_status(tv2);

    /* page 3 */
    // Variometer
    lv_obj_t *main_p3 = lv_obj_create(tv3);
    lv_obj_set_size(main_p3, LV_PCT(100), LV_PCT(100));
    lv_obj_clear_flag(main_p3, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(main_p3, 0, 0);
    lv_obj_set_style_bg_color(main_p3, UI_BG_COLOR, 0);

    lv_obj_t *p3_data = lv_obj_create(main_p3);
    lv_obj_set_size(p3_data, 360, 220);
    lv_obj_align(p3_data, LV_ALIGN_RIGHT_MID, 0, 10);
    lv_obj_set_style_bg_color(p3_data, UI_BG_COLOR, 0);
    lv_obj_set_style_border_color(p3_data, UI_BG_COLOR, 0);
    lv_obj_clear_flag(p3_data, LV_OBJ_FLAG_SCROLLABLE);

    // SMALL DATA
    lv_obj_t *p3_data_small = lv_obj_create(main_p3);
    lv_obj_set_size(p3_data_small, 200, 60);
    lv_obj_align(p3_data_small, LV_ALIGN_TOP_MID, 0, -20);
    lv_obj_set_style_bg_color(p3_data_small, UI_BG_COLOR, 0);
    lv_obj_set_style_border_color(p3_data_small, UI_BG_COLOR, 0);
    lv_obj_clear_flag(p3_data_small, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *c3_label_s = lv_label_create(main_p3);
    lv_obj_align(c3_label_s, LV_ALIGN_TOP_LEFT, 10, 30);
    lv_obj_set_style_text_align(c3_label_s, LV_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(c3_label_s, &lv_font_montserrat_42, 0);
    lv_label_set_text(c3_label_s, "NS");
    lv_obj_set_style_text_color(c3_label_s, UI_FONT_COLOR, 0);
    lv_obj_add_event_cb(c3_label_s,
                        update_text_subscriber_cb,
                        LV_EVENT_MSG_RECEIVED,
                        NULL);
    lv_msg_subsribe_obj(MSG_NEW_NESW, c3_label_s, (void *)"%s");

    lv_obj_t *el_label_s = lv_label_create(p3_data_small);
    lv_obj_align(el_label_s, LV_ALIGN_TOP_RIGHT, 0, -10);
    lv_obj_set_style_text_align(el_label_s, LV_ALIGN_RIGHT_MID, 0);
    lv_obj_set_style_text_font(el_label_s, &lv_font_montserrat_42, 0);
    lv_label_set_text(el_label_s, "0000 m");
    lv_obj_set_style_text_color(el_label_s, UI_FONT_COLOR, 0);
    lv_obj_add_event_cb(el_label_s,
                        update_text_subscriber_cb,
                        LV_EVENT_MSG_RECEIVED,
                        NULL);
    lv_msg_subsribe_obj(MSG_NEW_ELEV, el_label_s, (void *)"%d m");
    //END SMALL DATA

    lv_obj_t *vario_img = lv_img_create(main_p3);
    lv_obj_align(vario_img, LV_ALIGN_LEFT_MID, 10, 0);
    lv_img_set_src(vario_img,&vario);

    lv_obj_t *ms_text = lv_label_create(main_p3);
    lv_obj_set_size(ms_text, 100, 220);
    lv_obj_align(ms_text, LV_ALIGN_LEFT_MID, 0, 150);
    lv_obj_set_style_text_align(ms_text, LV_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(ms_text, &lv_font_montserrat_28, 0);
    lv_label_set_recolor(ms_text,true);
    lv_label_set_text(ms_text, "#55d400 m|s#");
    lv_obj_set_style_text_color(ms_text, UI_FONT_COLOR, 0);

    lv_obj_t *dm_cout = lv_obj_create(p3_data);
    lv_obj_set_size(dm_cout, 115, 170);
    lv_obj_align(dm_cout, LV_ALIGN_RIGHT_MID, 10, 0);
    lv_obj_set_style_bg_color(dm_cout, UI_DM_COLOR, 0);
    lv_obj_clear_flag(dm_cout, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(dm_cout,
                        update_background_color_cb,
                        LV_EVENT_MSG_RECEIVED,
                        NULL);
    lv_msg_subsribe_obj(MSG_NEW_BG_COLOR, dm_cout, (void*)"%i");

    // lv_obj_t *dm_point = lv_obj_create(p3_data);
    // lv_obj_set_size(dm_point, 25, 25);
    // lv_obj_align(dm_point, LV_ALIGN_BOTTOM_RIGHT, -90, 0);
    // lv_obj_set_style_bg_color(dm_point, UI_FONT_COLOR, 0);
    // lv_obj_clear_flag(dm_point, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *ms_label = lv_label_create(p3_data);
    lv_obj_center(ms_label);
    lv_obj_set_style_text_align(ms_label, LV_ALIGN_RIGHT_MID, 0);
    lv_obj_set_style_text_font(ms_label, &font_alibaba, 0);
    lv_label_set_text(ms_label, "0000");
    lv_obj_set_style_text_color(ms_label, UI_FONT_COLOR, 0);
    lv_obj_add_event_cb(ms_label,
                        update_text_subscriber_cb,
                        LV_EVENT_MSG_RECEIVED,
                        NULL);
    lv_msg_subsribe_obj(MSG_NEW_VARIO, ms_label, (void *)"%03i");

    lv_obj_t *plusminus_label = lv_label_create(tv3);
    lv_obj_set_size(plusminus_label, 100, 50);
    lv_obj_align(plusminus_label, LV_ALIGN_CENTER, -130, 10);
    lv_obj_set_style_text_align(plusminus_label, LV_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(plusminus_label, &lv_font_montserrat_48, 0);
    lv_label_set_recolor(plusminus_label,true);
        lv_label_set_text_fmt(plusminus_label, "#d5ff03 %s#",LV_SYMBOL_BELL);
    lv_obj_set_style_text_color(plusminus_label, UI_FONT_COLOR, 0);
    lv_obj_add_event_cb(plusminus_label,
                        update_text_subscriber_cb,
                        LV_EVENT_MSG_RECEIVED,
                        NULL);
    lv_msg_subsribe_obj(MSG_NEW_VARIO_PM, plusminus_label, (void *)"%s");

    ui_data_linetrough(main_p3);
    ui_toolbar_status(tv3);

    /* page 4 */

    lv_obj_t *main_p4 = lv_obj_create(tv4);
    lv_obj_set_size(main_p4, LV_PCT(100), LV_PCT(100));
    lv_obj_clear_flag(main_p4, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(main_p4, 0, 0);
    lv_obj_set_style_bg_color(main_p4, UI_BG_COLOR, 0);

    lv_obj_t *p4_data = lv_obj_create(main_p4);
    lv_obj_set_size(p4_data, 460, 220);
    lv_obj_align(p4_data, LV_ALIGN_RIGHT_MID, 0, 10);
    lv_obj_set_style_bg_color(p4_data, UI_BG_COLOR, 0);
    lv_obj_set_style_border_color(p4_data, UI_BG_COLOR, 0);
    lv_obj_clear_flag(p4_data, LV_OBJ_FLAG_SCROLLABLE);

    // SMALL DATA
    lv_obj_t *p4_data_small = lv_obj_create(main_p4);
    lv_obj_set_size(p4_data_small, 200, 60);
    lv_obj_align(p4_data_small, LV_ALIGN_TOP_MID, 0, -20);
    lv_obj_set_style_bg_color(p4_data_small, UI_BG_COLOR, 0);
    lv_obj_set_style_border_color(p4_data_small, UI_BG_COLOR, 0);
    lv_obj_clear_flag(p4_data_small, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *ms4_label_s = lv_label_create(p4_data_small);
    lv_obj_align(ms4_label_s, LV_ALIGN_TOP_RIGHT, 0, -10);
    lv_obj_set_style_text_align(ms4_label_s, LV_ALIGN_RIGHT_MID, 0);
    lv_obj_set_style_text_font(ms4_label_s, &lv_font_montserrat_42, 0);
    lv_label_set_text(ms4_label_s, "+1.2 m|s");
    lv_obj_set_style_text_color(ms4_label_s, UI_FONT_COLOR, 0);
    lv_obj_add_event_cb(ms4_label_s,
                        update_text_subscriber_cb,
                        LV_EVENT_MSG_RECEIVED,
                        NULL);
    lv_msg_subsribe_obj(MSG_NEW_VARIO_M, ms4_label_s, (void *)"%s m|s");

    lv_obj_t *el4_label_s = lv_label_create(main_p4);
    lv_obj_align(el4_label_s, LV_ALIGN_BOTTOM_MID, 0, 20);
    lv_obj_set_style_text_align(el4_label_s, LV_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(el4_label_s, &lv_font_montserrat_42, 0);
    lv_label_set_text(el4_label_s, "0000 m");
    lv_obj_set_style_text_color(el4_label_s, UI_FONT_COLOR, 0);
    lv_obj_add_event_cb(el4_label_s,
                        update_text_subscriber_cb,
                        LV_EVENT_MSG_RECEIVED,
                        NULL);
    lv_msg_subsribe_obj(MSG_NEW_ELEV, el4_label_s, (void *)"%d m");
    //END SMALL DATA


    ui_data_linetrough(main_p4);

    lv_obj_t *buss_label = lv_label_create(main_p4);
    lv_obj_align(buss_label, LV_ALIGN_TOP_LEFT, 10, 30);
    lv_obj_set_style_text_align(buss_label, LV_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(buss_label, &lv_font_montserrat_42, 0);
    lv_label_set_text(buss_label, "NS");
    lv_obj_set_style_text_color(buss_label, UI_FONT_COLOR, 0);
    lv_obj_add_event_cb(buss_label,
                        update_text_subscriber_cb,
                        LV_EVENT_MSG_RECEIVED,
                        NULL);
    lv_msg_subsribe_obj(MSG_NEW_NESW, buss_label, (void *)"%s");

    lv_obj_t *buss_label_t = lv_label_create(p4_data);
    lv_obj_align(buss_label_t, LV_ALIGN_CENTER, 0, -5);
    lv_obj_set_style_text_align(buss_label_t, LV_ALIGN_RIGHT_MID, 0);
    lv_obj_set_style_text_font(buss_label_t, &font_alibaba, 0);
    lv_label_set_text(buss_label_t, "000");
    lv_obj_set_style_text_color(buss_label_t, UI_FONT_COLOR, 0);
    lv_obj_add_event_cb(buss_label_t,
                        update_text_subscriber_cb,
                        LV_EVENT_MSG_RECEIVED,
                        NULL);
    lv_msg_subsribe_obj(MSG_NEW_BUSS_D, buss_label_t, (void *)"%03i");

    lv_obj_t *buss_img = lv_img_create(main_p4);
    lv_obj_align(buss_img, LV_ALIGN_LEFT_MID, 0, 30);
    lv_img_set_src(buss_img,&bussola);
    lv_obj_add_event_cb(buss_img,
                        update_rotation_buss_cb,
                        LV_EVENT_MSG_RECEIVED,
                        NULL);
    lv_msg_subsribe_obj(MSG_NEW_BUSS, buss_img, (void *)"%i");

    lv_obj_t *buss_text = lv_label_create(main_p4);
    lv_obj_set_size(buss_text, elev_png.header.w, 220);
    lv_obj_align(buss_text, LV_ALIGN_LEFT_MID, 28, 122);
    lv_obj_set_style_text_align(buss_text, LV_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(buss_text, &lv_font_montserrat_28, 0);
    lv_label_set_recolor(buss_text,true);
    lv_label_set_text_fmt(buss_text, "#55d400 %s#", LV_SYMBOL_UP);
    lv_obj_set_style_text_color(buss_text, UI_FONT_COLOR, 0);

    ui_toolbar_status(tv4);

    // lv_obj_t *debug_label = lv_label_create(tv3);
    // String text;
    // esp_chip_info_t t;
    // esp_chip_info(&t);
    // text = "chip : ";
    // text += ESP.getChipModel();
    // text += "\n";
    // text += "psram size : ";
    // text += ESP.getPsramSize() / 1024;
    // text += " KB\n";
    // text += "flash size : ";
    // text += ESP.getFlashChipSize() / 1024;
    // text += " KB\n";

    // lv_obj_set_style_text_font(debug_label, &lv_font_montserrat_28, 0);
    // lv_label_set_text(debug_label, text.c_str());
    // lv_obj_align(debug_label, LV_ALIGN_TOP_LEFT, 0, 0);

    // lv_obj_t *bat_label = lv_label_create(tv3);
    // lv_obj_set_style_text_font(bat_label, &lv_font_montserrat_28, 0);
    // lv_obj_align_to(bat_label, debug_label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);
    // lv_obj_add_event_cb(bat_label,
    //                     update_text_subscriber_cb,
    //                     LV_EVENT_MSG_RECEIVED,
    //                     NULL);
    // lv_msg_subsribe_obj(MSG_NEW_VOLT, bat_label, (void *)"VOLT : %d mV");

    /* page 4 */
    /// https://lvgl.io/tools/imageconverter
    /// LV_IMG_CF_RAW_CHROMA_KEYED
    // lv_obj_t *logo_img_4 = lv_gif_create(tv4);
    // lv_obj_center(logo_img_4);
    // lv_gif_set_src(logo_img_4, &lilygo1_gif);

    //lv_timer_t *timer = lv_timer_create(timer_task, 500, seg_text);

}

static void timer_task(lv_timer_t *t)
{
    lv_obj_t *seg = (lv_obj_t *)t->user_data;
    static bool j;
    if (j)
        lv_obj_add_flag(seg, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_clear_flag(seg, LV_OBJ_FLAG_HIDDEN);
    j = !j;
}

static void update_text_subscriber_cb(lv_event_t *e)
{
    lv_obj_t *label = lv_event_get_target(e);
    lv_msg_t *m = lv_event_get_msg(e);
    
    const char *fmt = (const char *)lv_msg_get_user_data(m);
    const int32_t *v = (const int32_t *)lv_msg_get_payload(m);

    lv_label_set_text_fmt(label, fmt, *v);

}

static void update_rotation_buss_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_msg_t *m = lv_event_get_msg(e);
    const int16_t *v = (const int16_t *)lv_msg_get_payload(m);

    lv_img_set_angle(obj, *v);
}

static void update_background_color_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_msg_t *m = lv_event_get_msg(e);
    const int *v = (const int *)lv_msg_get_payload(m);
    const lv_color_t c = lv_color_hex(*v);
    lv_obj_set_style_bg_color(obj, c, 0);
}
