extern struct statusData status;

#ifndef __BLE_H__
#define __BLE_H__

#include <Arduino.h>
#include <esp_coexist.h>
#include <NimBLEDevice.h>


#define BLE_LOW_HEAP 10000
String ble_data="";
bool ble_mutex=false;
uint8_t bluetoothStat ;
bool refresh = true;

String gpsStatus = "No Fix";
uint8_t gpsSat = 0;

NimBLECharacteristic *pCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "cfe9db21-4691-42eb-93b9-ca259e8f6580"
#define CHARACTERISTIC_UUID "062cf149-f6df-4b28-b599-08bfb0b017f3"

class MyServerCallbacks : public NimBLEServerCallbacks {

  void onConnect(NimBLEServer* pServer) {
      //log_d("***************************** BLE CONNECTED *****************");
    bluetoothStat = 2; //we have a connected client
    NimBLEDevice::startAdvertising();
    status.BLE_connected = true;
    log_i("BLE connected (ble.h): %i",status.BLE_connected);
  };

  void onDisconnect(NimBLEServer* pServer) {
    //log_d("***************************** BLE DISCONNECTED *****************");
    bluetoothStat = 1; //client disconnected
    NimBLEDevice::startAdvertising();
    status.BLE_connected = false;
  }
};

void checkReceivedLine(char *ch_str){
  if(!strncmp(ch_str, "#FNT", 4)){
    delay(1);
  }else if(!strncmp(ch_str, "#FNG", 4)){
    delay(1);
  }else if (!strncmp(ch_str,"$G",2)){
    delay(1);
  }
};

class MyCallbacks : public NimBLECharacteristicCallbacks {

  void onWrite(NimBLECharacteristic *pCharacteristic) {
    static String sLine = "";
    std::string rxValue = pCharacteristic->getValue();
    int valueLength = rxValue.length();
    if (valueLength > 0) {
      sLine += rxValue.c_str();
      if (sLine.endsWith("\n")){
        checkReceivedLine((char *)sLine.c_str());
        sLine = "";
      }
      if (sLine.length() > 512){
        sLine = "";
      }
    }
  }
};

void BLESendChunks(String str)
{
//  str = "$GPRMC,135533.00,A,4550.53722,N,00843.22619,E,1.292,,110122,,,A*7F";
  String substr;
  if (bluetoothStat == 2) {
    for (int k = 0; k < str.length(); k += _min(str.length(), 20)) {
      substr = str.substring(k, k + _min(str.length() - k, 20));
      pCharacteristic->setValue(std::string(substr.c_str()));
      pCharacteristic->notify();            
      vTaskDelay(5);
    }
      pCharacteristic->setValue(std::string("\r\n"));
      pCharacteristic->notify();    
      vTaskDelay(5);

  }
};

void start_ble (String bleId){
    
    NimBLEDevice::init(bleId.c_str());
    
    NimBLEDevice::setMTU(256); //set MTU-Size to 256 Byte
    NimBLEServer *pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    NimBLEService *pService = pServer->createService(NimBLEUUID((uint16_t)0xFFE0));
    // Create a BLE Characteristic
    pCharacteristic = pService->createCharacteristic(NimBLEUUID((uint16_t)0xFFE1),
      NIMBLE_PROPERTY::NOTIFY| NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );

    pCharacteristic->setCallbacks(new MyCallbacks());

    // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
    // Create a BLE Descriptor
    /***************************************************   
     NOTE: DO NOT create a 2902 descriptor. 
    it will be created automatically if notifications 
    or indications are enabled on a characteristic.
    
    pCharacteristic->addDescriptor(new BLE2902());
    ****************************************************/
    // Start the service
    pService->start();

    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter

    BLEDevice::startAdvertising();
    log_i("Waiting a client connection to notify...");
    delay(1);
    bluetoothStat = 1;
};

void stop_ble (){
    NimBLEDevice::deinit(true);
    esp_bt_controller_disable();
    esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
};

#endif