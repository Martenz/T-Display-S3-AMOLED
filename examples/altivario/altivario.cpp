#include <Arduino.h>
#include "altivario.h"
#include "OneButton.h" /* https://github.com/mathertel/OneButton.git */
#include "lvgl.h"      /* https://github.com/lvgl/lvgl.git */

#include "esp_log.h"
#include "rm67162.h"
#include "setup_img.h"
#include "tzi_gui.h"
#include "ble.h"

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
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
float voltages[VPARRAYSIZE] = {3.50, 3.60, 3.70, 3.80, 3.90, 4.00, 4.05, 4.20 };
uint8_t percentages[VPARRAYSIZE] = {  0,  20,  50,  80,  95,  98,   99, 100 };

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

static char* battery[5] = {
    LV_SYMBOL_BATTERY_EMPTY,
    LV_SYMBOL_BATTERY_1,
    LV_SYMBOL_BATTERY_2,
    LV_SYMBOL_BATTERY_3,
    LV_SYMBOL_BATTERY_FULL};

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

void setup()
{

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
    delay(50);

    pinMode(GPSRXPIN,INPUT);
    pinMode(GPSTXPIN,OUTPUT);
    ss.begin(GPSBAUD);
    delay(500);
    resetUBX();
    changeGpsHz();
    setSentences();
    delay(50);

    rm67162_init(); // amoled lcd initialization
    lcd_setRotation(1);
    xTaskCreatePinnedToCore(led_task, "led_task", 1024, NULL, 1, NULL, 0);
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
        // voltAdcPin_round = 3.4;
        char* nb = battery[4];
        if (volt_avg>uint32_t(4050)){
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

//        lv_msg_send(MSG_NEW_VOLT, &volt);

        last_tick = millis();
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

      // Serial.print("ms: ");
      // Serial.print(dt);
      // Serial.print(" alt: ");
      // Serial.print(altitude);
      // Serial.print(" est.alt: ");
      // Serial.print(estimated_alt);
      // Serial.print(" est.vario: ");
      // Serial.println(vario);

      status.altitude = estimated_alt;
      status.vario = vario;
      status.vario_avg = average_vario(vario);
      status.vario_dcm = int(status.vario_avg*10);
      status.vario_avg_b = average_vario_b(vario);

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
  
    // if (status.update_kalman){
    //   altKalmanFilter.setEstimateError(status.kalman_e_est);
    //   altKalmanFilter.setMeasurementError(status.kalman_e_mea);
    //   altKalmanFilter.setProcessNoise(status.kalman_q);      
    //   log_i("Updating kalman e_est:%f e_mea:%f q:%f",status.kalman_e_est,status.kalman_e_mea,status.kalman_q);  
    //   status.update_kalman = false;
    // }
    
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

        int32_t vd = status.vario_dcm;
        if (vd>=0){
            lv_msg_send(MSG_NEW_VARIO_PM, &pv);
            lv_msg_send(MSG_NEW_BG_COLOR, &pbg);
        }else{
            lv_msg_send(MSG_NEW_VARIO_PM, &mv);
            lv_msg_send(MSG_NEW_BG_COLOR, &mbg);
        }

        if (status.GPS_Fix){
          lv_msg_send(MSG_NEW_GPSFIX, &gpsfix);

          int32_t el = +status.GPS_alt;
          lv_msg_send(MSG_NEW_ELEV, &el);
        }else{
          lv_msg_send(MSG_NEW_GPSFIX, &gpsnofix);
          int32_t el = int(status.altitude);
          lv_msg_send(MSG_NEW_ELEV, &el);
        }

        if (status.baro_ok) lv_msg_send(MSG_NEW_BARO,&barook);

//        log_i("Elevation: %i - Vario: %i",el,vd);

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

    if (status.lowPower) break;
    delay(10);
  }

  ss.end();
  // log_i("stop task GPSU7");
  // Serial.println("stop task GPSU7");
  log_i("stop task GPSU7");
  vTaskDelete(xHandleGPSU7);
}