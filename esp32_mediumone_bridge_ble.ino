/*
 * esp32_mediumone_bridge_ble.ino
 * 
 * Connect to Wi-Fi and BLE device, receive incoming Temperature
 * measurements via BLE, send sensor data to Medium One.
 * 
 * Copyright (c) 2020 Greg Toth. All rights reserved.
 * 
 * Dependencies:
 * 
 *    -- ESP32 Arduino Board Support Package
 *    -- PubSubClient Library
 */

//#include "WiFiClient.h" // Non-TLS
#include "WiFiClientSecure.h" // TLS
#include "BLEDevice.h"
#define MQTT_MAX_PACKET_SIZE 1024   /* must set this inside PubSubClient.h */
#include <PubSubClient.h>

#define WIFI_SSID       "YOUR_INFO"
#define WIFI_PASSWORD   "YOUR_INFO"

#define MQTT_BROKER     "mqtt.mediumone.com"
#define MQTT_PORT       61618           /* encrypted port */
#define MQTT_USERNAME   "xxxxxxxxxxx/xxxxxxxxxxx"
#define MQTT_PASSWORD   "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx/xxxxxxxxxxxxxxxxx"
#define MQTT_PUB_TOPIC  "0/xxxxxxxxxxx/xxxxxxxxxxx/mydevice"

#define PUB_INTERVAL_MS 3000
#define ENABLE_PUBLISH  1

// BLE service, characteristic and controls
BLEUUID serviceUUID(BLEUUID((uint16_t)0x1809));  // Health Thermometer Service
BLEUUID charUUID(BLEUUID((uint16_t)0x2A1c));     // Temperature Measurement Characteristic
BLERemoteCharacteristic* pRemoteCharacteristic;
BLEAdvertisedDevice* myDevice;
const uint8_t indicateOn[] = {0x2, 0x0};
const uint8_t indicateOff[] = {0x0, 0x0};
boolean foundDevice = false;
boolean foundService = false;
boolean foundCharacteristic = false;
boolean bleConnected = false;
float tempC, tempF;
uint32_t sensorUpdateCount = 0;
uint32_t sensorUpdateCountLast = 0;
uint32_t iterationCount = 0;
uint32_t timestampLast = 0;

// Wi-fi & Medium One
//WiFiClient          wifiClient;     // Non-TLS
WiFiClientSecure    wifiClient;     // TLS
PubSubClient mqttClient(wifiClient);
const char *mqttClientId = "esp32_wifi_ble";
boolean mqttConnectActive = false;

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      BLEDevice::getScan()->stop();  // stop scan
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      foundDevice = true;
      Serial.println("Found our BLE device");
    }
  }
};

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient *pClient) {
    Serial.println("Connected to BLE server");
  }
  void onDisconnect(BLEClient *pClient) {
    bleConnected = false;
    Serial.println("Disconnected from BLE server");
  }
};

void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
#if 0
  Serial.print("Notify for characteristic: ");
  Serial.println(pBLERemoteCharacteristic->getUUID().toString().c_str());
#endif
  int32_t mantissa = (((pData[3] << 16) | (pData[2] << 8) | pData[1]) << 8) >> 8;
  int8_t exponent = pData[4];
  tempC = (float)mantissa * pow(10, exponent);
  tempF = tempC * (9.0/5.0) + 32.0;
  sensorUpdateCount++;
#if 0
  Serial.print("tempF = "); Serial.print(tempF); Serial.print(", sensorUpdateCount = "); Serial.println(sensorUpdateCount);
#endif
}

void connectWifi(void)
{
  int countdown;
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("Connecting to Wi-Fi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    countdown = 10;
    while (countdown-- > 0) {
      if (WiFi.status() == WL_CONNECTED) {
        break;
      } else {
        Serial.print(".");
        delay(1000);
      }
    }
    if (countdown == 0) {
      Serial.println("Failed, retrying");
    } else {
      Serial.println("Connected");
    }
  }
}

void connectMqtt(void)
{
  Serial.print("Connecting to MQTT...");
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  //wifiClient.setCACert(root_ca);
  if (mqttClient.connect(mqttClientId, MQTT_USERNAME, MQTT_PASSWORD)) {
    mqttConnectActive = 1;
    Serial.println("Connected");
  } else {
    Serial.println("ERROR: MQTT connect failed");
  }
}

void connectBLE(boolean mqttLoop = false)
{
  // Scan for our BLE device

  BLEDevice::init("");
  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  boolean continueFlag = false;
  while (true) {
    if (mqttLoop) {
      mqttClient.loop();
    }
    Serial.println("Scanning for our BLE device...");
    pBLEScan->start(5, continueFlag);
    //continueFlag = true;  // if set true, previously found devices are skipped
    if (foundDevice) {
      break;
    }
  }

  // Connect to our BLE device and enable temperature indications

  Serial.print("Connecting to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  BLEClient *pClient = BLEDevice::createClient();
  Serial.println("Created BLE client");
  pClient->setClientCallbacks(new MyClientCallback());
  pClient->connect(myDevice);
  Serial.println("Connected to device");
  BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == NULL) {
    Serial.print("Unable to find service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
  } else {
    Serial.println("Found service");
    foundService = true;
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == NULL) {
      Serial.print("Unable to find characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
    } else {
      Serial.println("Found characteristic");
      foundCharacteristic = true;
      if (pRemoteCharacteristic->canIndicate()) {
        // Enable value indication
        pRemoteCharacteristic->registerForNotify(notifyCallback);
        pRemoteCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)indicateOn, 2, true);
        bleConnected = true;
      }
    }
  }
}

void setup() {

  Serial.begin(115200);
  Serial.println("Starting app");

  // Connect to Wi-Fi, MQTT, BLE in that order
  
  connectWifi();
  connectMqtt();
  connectBLE(true);

  timestampLast = millis();

  Serial.println("Done with setup()");
  
}

void loop() {

  uint32_t timenow = millis();
  boolean status;

  mqttClient.loop();
  
  if ((timenow - timestampLast) >= PUB_INTERVAL_MS) {
    if (WiFi.status() == WL_CONNECTED && bleConnected) {
      if (sensorUpdateCount != sensorUpdateCountLast) {
        if (mqttConnectActive && !mqttClient.connected()) {
          // Re-connect
          if (!mqttClient.connect(mqttClientId, MQTT_USERNAME, MQTT_PASSWORD)) {
            Serial.println("ERROR: MQTT re-connect failed");
          }
        }
        if (mqttClient.connected()) {
          // Publish sensor message
          char msg[256];
          sprintf(msg, "{\"event_data\":{\"iteration\":%u,\"timestamp\":%u,\"temp\":%.2f}}",
              iterationCount++, timenow, tempF);
          if (mqttClient.publish(MQTT_PUB_TOPIC, msg)) {
            Serial.print("Published to topic '");
            Serial.print(MQTT_PUB_TOPIC);
            Serial.print("' msg '");
            Serial.print(msg);
            Serial.println("'");
          } else {
            Serial.println("ERROR: MQTT publish failed");
          }
        }        
      }
    } else {
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("*** Wi-Fi is not connected ***");
      }
      if (!bleConnected) {
        Serial.println("*** BLE is not connected ***");
      }
    }
    timestampLast = timenow;
  }
}
