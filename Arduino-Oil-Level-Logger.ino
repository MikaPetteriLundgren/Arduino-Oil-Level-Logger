/*
   Temperature readings from oil tank is measured with DS18B20 digital temperature sensor connected via OneWire.
   Ultrasonic sensor used in this application is MB1010 from MaxBotix. Temperature and fill rate values are sent to Domoticz server via Wifi shield using MQTT protocol.
   R7 is mounted (0R) in Wifi shield in order to reset the shield at startup. System reset will take place if MQTT connection has failed 5 times altogether or
   if Wifi connection hasn't been established in 3 consecutive tries. There is also possibility to take thermal expansion of oil into account in calculations.
   Sketch includes a possibility to check the amount of free RAM and print more information about ultrasonic measurements.

   An idea about how to sort arrays is based on the code found from Arduino forums http://forum.arduino.cc/index.php?topic=20920.0

   The sketch needs oilLevelLoggerSettings.h header file in order to work. The header file includes settings for the sketch.
   */
#include "oilLevelLoggerSettings.h" // Header file containing settings for the sketch included.
#include <WiFi.h>
#include <OneWire.h>
#include <SPI.h>
#include <DallasTemperature.h>
#include <Time.h>
#include <TimeAlarms.h>
#include <PubSubClient.h>
#include <avr/wdt.h> // Needed for watchdog reset

// Temperature measurement variables are initialised
float temperature = 0; // Measured temperature of oil tank is stored to this variable

// Variables for ultrasonic sensor
const int tofPin = 8; // Data pin of ultrasonic sensor
float tofDistance = 0; // Measured distance in centimetres
const int sampleAmount = 5; // Amount of samples to be measured. Median value is determined from the samples. sampleAmount needs to be an odd number.
unsigned int tofValues[sampleAmount]; // Measured ToF values are stored to this array

// Variables for oil tank
const long width = 120; // Internal width of oil tank in centimetres
const long depth = 284; // Internal depth/length of oil tank in centimetres
const long height = 122; // Internal heigth of oil tank in centimetres
const long heightOffset = 6; // Heigth offset in centimeters compared to top level of tank if ultrasonic sensor is attached for example on top of manhole's hatch etc.
long maxVolume = 0; // Calculated max. volume of oil tank in cm^3
long volume = 0; // Calculated current volume of oil tank in cm^3
long fillRate = 0; // Calculated fill rate of oil tank
const unsigned int measInterval = 3600; // Oil tank measurement interval in seconds. Default value is 3600s (1h)
const float expansionCoefficent = 0.0007; // Volumetric coefficient of expansion of oil (1/DegC)
const boolean expansionCalculationEnabled = true; // true = thermal expansion calculation of oil enabled; false = thermal expansion calculation of oil disabled
const int referenceOilTemperature = 3; // Reference temperature in degC used in thermal expansion calculation

// OneWire and Dallas temperature sensors library are initialised
#define ONE_WIRE_BUS 2 // OneWire data wire is connected to GPIO2 pin of the Arduino. Parasite powering scheme is used.
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature library

// Wifi network settings
char ssid[] = NETWORK_SSID; // Network SSID (name) is defined in oilLevelLoggerSettings.h header file
char pass[] = NETWORK_PASSWORD; // Network password is defined in oilLevelLoggerSettings.h header file
int status = WL_IDLE_STATUS; // Wifi status
IPAddress ip; // IP address
WiFiClient wifiClient; // Initialize Arduino Wifi Client

//MQTT configuration
char topic[] = MQTT_TOPIC; // Default incoming topic in Domoticz is domoticz/in
int mqttConnectionFails = 0; // If MQTT connection is disconnected for some reason, this variable is increment by 1

// MQTT callback function header
void mqttCallback(char* topic, byte* payload, unsigned int length);

//MQTT initialization
PubSubClient mqttClient(MQTT_SERVER, 1883, mqttCallback, wifiClient); // MQTT_SERVER constructor parameter is defined in oilLevelLoggerSettings.h header file
char clientID[50];
char msg[80];

//MQTT variables
const int temperatureSensordtype = 80; // dtype (device type of temperature sensor) is used to help creating MQTT payload
const int percentageSensordtype = 2; // dtype (device type of percentage sensor) is used to help creating MQTT payload
const int temperatureSensorIDX = 17; // IDX number of temperature sensor
const int percentageSensorIDX = 18; // IDX number of percentage sensor

//#define DEBUG // Uncomment this line if there is no need to print debug information via serial port
//#define RAM_DEBUG // Uncomment this line if there is no need to print RAM debug information via serial port

void setup()
{
  Serial.begin(9600); // Start serial port
  //while (!Serial) ; // Needed for Leonardo only

  // Reset WiFi shield (A5 connected to RESET pin of WiFi shield)
  Serial.println("\nArduino will reset WiFi shield in 2s...");
  delay(2000);
  pinMode(A5, OUTPUT);
  digitalWrite(A5, LOW); // Reset WiFi shield
  delay(500);
  pinMode(A5, INPUT);
  delay(2000);

  // SD Card SPI CS signal is configured to output and set to high state. SD Card and WiFi controller shares same SPI bus
  pinMode(4,OUTPUT);
  digitalWrite(4,HIGH);

  // Start up the OneWire and sensors library
  sensors.begin();

  //Find DS18B20 devices on the bus
  Serial.println(F("-------DS18B20 sensor setup started-------"));
  Serial.println(F("Locating devices..."));
  Serial.print(F("Found "));
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(F(" devices."));
  Serial.println(F("-------DS18B20 sensor setup finished-------\n"));

  // Data pin of ultrasonic sensor is set to Input mode
  pinMode(tofPin, INPUT);

  // Volume of oil tank is calculated
  maxVolume = width * depth * height;
  Serial.print(F("Volume of oil tank is: "));
  Serial.print(maxVolume / 1000); // maxVolume needs to be divided by 1000 in order to get max volume in litres
  Serial.println(F(" litres\n"));

  // Start Wifi on Arduino
  startWiFi();

  //Create MQTT client String
  String clientIDStr = "Arduino-";
  clientIDStr.concat(DEVICE_ID); // DEVICE_ID is defined in oilLevelLoggerSettings.h header file
  clientIDStr.toCharArray(clientID, clientIDStr.length()+1);

  // Timers are initialised
  Alarm.timerRepeat(measInterval, measFunction);

  Serial.println(F("Setup completed succesfully!\n"));
}


void loop()
{

    mqttClient.loop(); //This should be called regularly to allow the MQTT client to process incoming messages and maintain its connection to the server.

    Alarm.delay(1000); //Timers are only checks and their functions called when you use this delay function. 0 can be passed for minimal delay.


   #if defined RAM_DEBUG
     Serial.print(F("Amount of free RAM memory: "));
     Serial.print(memoryFree()); // Prints the amount of free RAM memory
     Serial.println(F(" / 2048 bytes")); //ATmega328 has 2kB of RAM memory
   #endif
}


// mqttCallback function handles message arrived on subscribed MQTT topic(s)
void mqttCallback(char* topic, byte* payload, unsigned int length) {
	// No processing here because MQTT callback functionality is not needed in this sketch
}

String createMQTTPayload(int idx, int deviceType) //Create MQTT message payload. Returns created message as a String.
{
	String dataMsg = "{\"idx\":";
	dataMsg.concat(idx);

	switch (deviceType)
	  {
		case 80: // Temperature sensor.
			char buffer[10]; // Needed with dtostrf function
	    	dataMsg.concat(F(",\"svalue\":\""));
	    	dataMsg.concat(dtostrf(temperature, 5, 1, buffer)); //Converts float temperature to String with 1 digit precision
	    	dataMsg.concat("\"}");
	    	break;

		case 2: // percentage sensor
	    	dataMsg.concat(F(",\"svalue\":\""));
	    	dataMsg.concat(fillRate);
	    	dataMsg.concat("\"}");
	    	break;

		default: // If dtype is unknown, then default case to be used. Default case is a temperature sensor.
			Serial.println(F("Unknown dtype received. Default procedure to be done."));
	    	dataMsg.concat(F(",\"svalue\":\""));
	    	dataMsg.concat(temperature);
	    	dataMsg.concat("\"}");
	    	break;
	  }

	return dataMsg;
}

void sendMQTTPayload(String payload) // Sends MQTT payload to the MQTT server running on a Raspberry Pi. MQTT server deliveres data to Domoticz server running on a same Raspberry Pi
{

	// Convert payload to char array
	payload.toCharArray(msg, payload.length()+1);

    //If connection to MQTT broker is disconnected, connect again
    if (!mqttClient.connected())
    {
      Serial.println(F("MQTT client disconnected, to be reconnected..."));

      // Wifi connection to be disconnected and initialized again if reconnecting to MQTT broker fails
      if (mqttClient.connect(clientID))
      {
        Serial.println(F("MQTT client reconnected"));
      }
      else
      {
        Serial.println(F("MQTT client reconnection failed. Wifi connection to be initialized again!"));
        mqttConnectionFails +=1; // // If MQTT connection is disconnected for some reason, this variable is increment by 1
        WiFi.disconnect();
        (WL_DISCONNECTED) ? Serial.println(F("Wifi disconnected")) : Serial.println(F("Wifi still connected for some reason...")); //condition ? valueIfTrue : valueIfFalse - This is a Ternary operator
        startWiFi();
      }

      // Arduino and Wifi shield to be reset if MQTT connection has been disconnected 5 times altogether
      if (mqttConnectionFails >= 5)
      {
        Serial.print(F("Resetting system in 2s...."));
        wdt_enable(WDTO_2S); // Watchdog will bite within 2s after it has been enabled
        delay(3000);
      }
    }

	//Publish payload to MQTT broker
	if (mqttClient.publish(topic, msg))
	{
		Serial.print(F("Following data published to MQTT broker: "));
		Serial.print(topic);
		Serial.print(F(" "));
		Serial.println(payload);
		Serial.println();
	}
	else
		Serial.println(F("Publishing to MQTT broker failed...\n"));
}

float tempReading() // Function TempReading reads temperature from DS18B20 temp sensor
{
  sensors.requestTemperatures(); // Send the command to get temperatures
  return (float) sensors.getTempCByIndex(0); // Return temperature value
}

void measFunction() /* Function measFunction reads temperature from DS18B20 sensor and measures fill rate of oil tank.
                    The temperature and fill rate is sent to Domoticz via MQTT protocol*/
{
	Serial.println(F("Requesting temperature from oil tank sensor..."));
  temperature = tempReading(); // TempReading function is called

  // Measured temperature is printed
  Serial.print(F("Temperature: "));
  Serial.print(temperature);
  Serial.println(F("DegC"));

  sendMQTTPayload(createMQTTPayload(temperatureSensorIDX, temperatureSensordtype)); //Send measured temperature value to MQTT broker running on a Raspberry Pi
  delay(1000); // Delay added in order to provide some time for Domoticz to process incoming data

	// Fill level of oil tank is measured and sent to Domoticz
  Serial.println(F("Fill rate of oil tank to be measured..."));
  readToFSensor();
  calculateVolume();
  sendMQTTPayload(createMQTTPayload(percentageSensorIDX, percentageSensordtype)); //Send calculated fill rate to MQTT broker running on a Raspberry Pi
}

// Function calculates temperature compensated distance from top of oil tank to level of oil
void readToFSensor()
{
  tofDistance = (measureToFValues() * ((20.05 * sqrt(temperature + 273.15)) / 2)) / 10000; // This formula can be found via following link: http://www.maxbotix.com/documents/Temperature_Compensation.pdf
  Serial.print(F("Measured and temperature compensated distance is "));
  Serial.print(tofDistance);
  Serial.println(F("cm"));
}

// Function measures as many ToF values as defined in sampleAmount variable. Function determines and returns median value of the measurements.
unsigned int measureToFValues()
{
  unsigned int tofValue = 0; // Variable for calculated ToF value

  for (int i = 0; i < sampleAmount; i = i + 1)
  {
    tofValues[i] = pulseIn(tofPin, HIGH);
    delay(100); // Minimum delay 50ms because readings of MB1010 sensor can occur up to every 50ms
  }

  #if defined DEBUG
    Serial.print(F("Measured unsorted values are: "));
    printArray(tofValues, sampleAmount);
  #endif

  sortAscending(tofValues, sampleAmount);

  #if defined DEBUG
    Serial.print(F("Measured sorted values are: "));
    printArray(tofValues, sampleAmount);
  #endif

  int medianValue = sampleAmount / 2; // Determine middle index of an array
  tofValue = tofValues[medianValue]; // Median value is the middle value of an array
  Serial.print("Median value is: ");
  Serial.println(tofValue);

  return tofValue;
}

// Function sortAscending sorts values of the array to ascending order
void sortAscending(unsigned int *a, int n) // *a is an array pointer function
{
 for (int i = 1; i < n; ++i)
 {
   unsigned int j = a[i];
   unsigned int k;
   for (k = i - 1; (k >= 0) && (j < a[k]); k--)
   {
     a[k + 1] = a[k];
   }
   a[k + 1] = j;
 }
}

// Function printArray prints values of the array
void printArray(unsigned int *a, int n) // *a is an array pointer function
{

 for (int i = 0; i < n; i++)
 {
   Serial.print(a[i], DEC);
   Serial.print(' ');
 }

 Serial.println();
}

// Function calculates current amount of oil in litres and prints the volume and fill rate
void calculateVolume()
{
  volume = width * depth * (height + heightOffset - tofDistance);

  Serial.print(F("Current amount of oil without thermal expansion correction is "));
  Serial.print(volume/1000); // volume needs to be divided by 1000 in order to get volume in litres
  Serial.println(F(" litres"));

  if (expansionCalculationEnabled)
  {
	  volume = volume + (volume * expansionCoefficent * (referenceOilTemperature - temperature)); // Formula can be found from http://www.engineeringtoolbox.com/volumetric-temperature-expansion-d_315.html
	  Serial.print(F("Current amount of oil with thermal expansion correction is "));
	  Serial.print(volume/1000); // volume needs to be divided by 1000 in order to get volume in litres
	  Serial.println(F(" litres"));
  }

  fillRate = 100 * volume / maxVolume; // Fill rate needs to be multiplied by 100 in order to get fill rate as a percentage
  Serial.print(F("Fill rate is "));
  Serial.print(fillRate);
  Serial.println(F("%"));
}

void startWiFi()
{
  int resetCounter = 0;

  // Attempt to connect to Wifi network:
  while (status != WL_CONNECTED)
  {

    // If Wifi connection hasn't been established in 3 consecutive tries, watchdog reset to be applied
    if(resetCounter > 2)
    {
      Serial.print(F("Wifi connection failed, resetting in 2s...."));
      wdt_enable(WDTO_2S); // Watchdog will bite within 2s after it has been enabled
      delay(3000);
    }
     // Connect to WPA/WPA2 network
    Serial.print(F("Attempting to connect to WiFi network: "));
    Serial.println(ssid);
    Serial.println(F("Wait 10s for connection"));
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
    resetCounter++;
  }

  // You're connected now, so print out the local ip address of the WiFi shield:
  Serial.println(F("Connected to the network using DHCP"));
  Serial.print(F("IP address: "));
  ip = WiFi.localIP();
  Serial.println(ip);

}

// variables created by the build process when compiling the sketch. Used in memoryFree function
extern int __bss_end;
extern void *__brkval;

int memoryFree() //Function to return the amount of free RAM
{
  int freeValue;
  if((int)__brkval == 0)
  {
    freeValue = ((int)&freeValue) - ((int)&__bss_end);
  }
  else
  {
    freeValue = ((int)&freeValue) - ((int)__brkval);
  }
  return freeValue;
}
