Arduino-Oil-Level-Logger
=================

Arduino based oli level logger is a measurement device for Domoticz home automation system which monitors
fill rate of oil tank. The Arduino oli level logger includes following features:

* Ultrasonic sensor for measuring fill rate of oil tank
* Temperature sensor DS18B20
* 1-way communication from the oil level logger to the Domoticz using MQTT protocol

Temperature readings are read from DS18B20 digital temperature sensor via OneWire bus and Dallas Temperature Control Library.
Time of flight data is read from the ultrasonic sensor using pulse width output. The temperature data is used in calculations
about actual distance from the sensor to level of oil. Thermal expansion of the oil is not taken into account.

All the data to the Domoticz is transmitted via WiFi in LAN using the MQTT protocol.

In practice all the "intelligence" is in the Domoticz and therefore for example decision about is oil tank empty enough for notification
of end user is done by the Domoticz.

Arduino-Oil-Level-Logger sketch will need following SW libraries and HW to work:

**HW**

* Arduino Uno and WiFi shield
* DS18B20 temperature sensor
* Ultrasonic sensor MB1010 from MaxBotix

**Libraries**

* WiFi for WiFi communication
* SPI for WiFi shield
* OneWire for communication with DS18B20 sensor
* DallasTemperature for DS18B20 sensor
* Time and TimeAlarms for alarm functionality
* PubSubClient for MQTT communication
* awr/wdt.h for watchdog timer handling

**HW connections**

Temperature readings are read from DS18B20 digital temperature sensor via OneWire bus connected to GPIO2. 
The DS18B20 is used in parasite mode. The DS18B20 is connected to the Arduino system via two wire cable and RCA connector.

The ultrasonic sensor is connected to GPIO8 and is used in pulse width mode. The ultrasonic sensor is placed in a small enclosure which
is fixed on top of manhole's hatch. The connection between the ultrasonic sensor and the Arduino is taken care with three wire cable and 3.5mm AV connector.

Resistor R7 (0R) is mounted on the WiFi shield in order to reset the Wifi shield by pulling ANALOG_INPUT5 down if needed. 

This repository includes also breadboard and schematics data for the oil level logger and ultrasonic subsystem.

**Functionality of the sketch**

The sketch is intended to work with rectangular tanks.

The WiFi shield is reset every time when setup() function runs. This is needed because if watchdog reset takes place (in case of WiFi shield is jammed), WiFi shield
needs to be reset as well in order to get it working again.

Volume of oil tank is calculated only in setup() based on dimensions of the oil tank. Timings of temperature and oil level measurements are taken care by timer. 
Both measurements will take place once the timer triggers. Once the timer triggers, temperature is measured and sent to the Domoticz using the MQTT protocol.

Next ToF (Time of Flight) is measured with the ultrasonic sensor. By default 5 measurements are done. Measured values are filtered (min and max values are excluded) and
average is calculated from rest of the values. Median or mode filtering could be used if filtering and averaging doesn't provide robust results.

Distance between the sensor and level of the oil is calculated using the measured temperature and filtered and averaged ToF data (speed of sound changes in a function of the temperature).
Once the distance is known, actual volume and fill rate of oil tank is calculated and sent to the Domoticz using the MQTT protocol. If tank has a manhole which has some offset to top of tank,
it can be taken into account in the volume and fill rate calculations.

IDX values of the sensors and measured values are added to the MQTT payload (payload differs depends on a type of a sensor (temperature vs percentage)). 
The created MQTT payload is sent to MQTT server. It's constantly checked in main loop is MQTT connection alive in order to maintain connection to MQTT server. 
MQTT callback function doesn't do anything in the sketch though.

If the MQTT connection has been disconnected, reconnection will take place. If the reconnection fails, WiFi connection is disconnected and initialized again.
If the MQTT connection has been disconnected 5 times altogether, the Arduino and WiFi shield is reset. Reset will take place also if WiFi connection
hasn't been established in 3 consecutive tries.

It's possible to print amount of free RAM memory of Arduino via serial port by uncommenting `#define RAM_DEBUG` line
It's possible to print more debug information via serial port by uncommenting `#define DEBUG` line
