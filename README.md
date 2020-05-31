# esp32_mediumone_bridge_ble

Bridge application for interfacing between BLE and Medium One IoT Platform using an ESP32-based board.

Author: Greg Toth

### Target Boards

* Adafruit HUZZAH32
* ESP32 DevKitC and compatible variants

### Development Tools & Libraries

* Arduino IDE
* ESP32 Arduino Core Board Support Package for Arduino IDE
* PubSubClient Library for Arduino IDE

### Arduino Environment Setup

* Download or clone this repo into a directory on your computer.
* Install Arduino IDE on your computer.
* Launch Arduino IDE.
* Install the ESP Arduno Core Board Support Package by following instructions at https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md
* In Arduino IDE, from Tools > Manage Libraries... install these libraries:
    * PubSubClient by Nick O'Leary, version 2.7.0 or higher
* For the PubSubClient library, edit Arduino/libraries/PubSubClient/src/PubSubClient.h and change MQTT_MAX_PACKET_SIZE to be 1024 instead of 128.
* Open the application source code file esp32_mediumone_bridge_ble.ino from repository directory.

### Configuring Wi-Fi and Medium One Parameters

Set these #defines to your own values:

```
#define WIFI_SSID      ""
#define WIFI_PASSWORD  ""
#define MQTT_USERNAME  ""
#define MQTT_PASSWORD  ""
#define MQTT_PUB_TOPIC ""
```

### Programming the ESP32 Board

Connect the ESP32 board to your computer using a USB cable, then compile and upload the program to the board.

