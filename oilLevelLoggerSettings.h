/*
  oilLevelLoggerSettings.h header file includes settings for the Arduino-Oil-Level-Logger sketch.
  Header file needs to be stored within a same folder with the sketch.
*/
#ifndef oilLevelLoggerSettings_h
#define oilLevelLoggerSettings_h

  #define NETWORK_SSID "NetworkName" // SSID (name) of the WLAN network
  #define NETWORK_PASSWORD "NetworkPassword" // Password of the WLAN network
  #define MQTT_SERVER "192.168.1.40" // IP address of the MQTT server
  #define MQTT_TOPIC "domoticz/in" // MQTT topic where data is sent. Default incoming topic in Domoticz is domoticz/in
  #define DEVICE_ID "Uno"
  
#endif
