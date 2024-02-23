/**********************************************************
  
  Arduino sketch to drive an air quality sensor for the studio
  
  Inputs:
  * various sensors
  * 

  Outputs:
  * Publish stats serial output for diagnostics
  * Publish to MQTT topics for integration with home assistant
  

  Developed for:
  * Feather Huzzah
    * https://www.adafruit.com/product/2821  ESP8266
    * https://www.adafruit.com/product/3405  - or - ESP32
  * https://www.mouser.co.uk/ProductDetail/Sensirion/SEN55-SDN-T - SEN55 air quality sensor
  * https://www.adafruit.com/product/4162  veml7700 light sensor

  * Both sensors run on I2C, so for each connect +V, Ground, and SDA + SDL.  The lux sensor
    is a lot easier to interface with, so is included in here as part of getting the system
    operable before dealing with the nuances of the SEN5x data set.
    

  * The SEN55 needs to connect to +5v (not +3.3v).  Without this the PM measurements are bogus

  
  TODO:
  * Handle millis() rollover.  Perhaps NTP time instead?
  * Split .ino into multiple files for management - http://www.gammon.com.au/forum/?id=12625
  * OTA updates
  * ...
  * Because luminance can change far more rapidly than it's polled:
    * report spot-luminance -- i.e. absolute last value
    * report high-lum -- highest it's been in the past minute/15m/30m/...
    * report low-lum  -- lowest it's been in the same period
  *

     // todo -- if lux changes more than 50%, then send a mqtt update immediately, else wait for the timer


**********************************************************/


#define VERSION "0.1.0"

// Comment this line out for the Feather Huzzah ESP8266
#define ESP32


// Private config held in separate file
// const char* wifiName = "SSID";
// const char* wifiPass = "password";
#include "private-config.h"


#ifdef ESP32
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif


// save macaddr during setup()
char macaddr[15] = "";



// TODO - capture and push the autodiscovery json payload




#include <PubSubClient.h>


// * these must be defined in private-config.h 
//const char* mqttServer = "mqtt.example.com";
//const int   mqttPort = 1883;
//const char* mqttUser = "username";
//const char* mqttPass = "password";

// generate and save during setup()
char mqttIdent[64] = "";


int mqttBufferSize = MQTT_MAX_PACKET_SIZE;   // default, from headers








// SEN5x - https://github.com/Sensirion/arduino-i2c-sen5x/blob/master/examples/exampleUsage/exampleUsage.ino
#include <SensirionI2CSen5x.h>
#include <Wire.h>

// SEN5x docs talk about I2C_BUFFER_LENGTH being bigger than some Arduino implementation; not a concern for our ESP32 or ESP8266

SensirionI2CSen5x sen5x;


boolean aqPresent = 0;            // Air Quality sensor present; set true if it's detected

uint32_t delayEnviron;       // delay between re-reading value


// Read Measurements
float massConcentrationPm1p0;
float massConcentrationPm2p5;
float massConcentrationPm4p0;
float massConcentrationPm10p0;
float ambientHumidity;
float ambientTemperature;
float vocIndex;
float noxIndex;

float lastLux = 0;
float lastPM1 = 0;
float lastPM25 = 0;
float lastPM4 = 0;
float lastPM10 = 0;
float lastHumidity = 0;
float lastTemperature = 0;
float lastVoc = 0;
float lastNox = 0;

boolean temperatureChanged = 0;
boolean humidityChanged = 0;
boolean vocChanged = 0;
boolean noxChanged = 0;
boolean pm1Changed = 0;
boolean pm25Changed = 0;
boolean pm4Changed = 0;
boolean pm10Changed = 0;


//
boolean luxChanged = 0;

    

// not used any more -- todo, why are they marked volatile?
volatile float temp = 0.0;
volatile boolean tempGood = 0;    // value is not good before first reading
volatile float humidity = 0.0;
volatile boolean humidityGood = 0;








// VEML7700 produces actual lux value, as well as luminosity equivalent
// if we fail to initialize, we disable the sensor and stop taking readings
#include "Adafruit_VEML7700.h"

Adafruit_VEML7700 veml = Adafruit_VEML7700();
volatile boolean luxPresent = 0;   // hardware is present?
volatile boolean luxGood = 0;      // value read is good?
volatile float lux = 0.0;
uint32_t delayLuminance = 1000;    // read value every second


// Update environmental stats this many ms, unless prompted to update sooner
uint32_t envFreq = 2500;          // when environmentals are changing, send this often
uint32_t envForceFreq = 300000;   // force an update this often, even if not changing

unsigned long lastEnvTime = 0;    // timestamp of last env update
int sendStateNow = 0;             // env state changed, and should be sent asap




// Timer to capture last poll of sensors
// Set to 30s to avoid polling for 30s after booting to help data settle before reporting

// TODO: sanity check for millis() rolling over
unsigned long lastEnviron = 30000;
unsigned long lastLuminance = 30000;




// callback function is needed for the MQTT client connection string, but only used when subscribing to
// topics, so in our case we leave it empty.
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // empty
}



// Create MQTT Client
WiFiClient espClient;
PubSubClient mqttclient(mqttServer, mqttPort, mqttCallback, espClient);






// processor function is called for our HTML file and
// replaces tags (e.g. %STATE%) with the value returned here
//
// TODO: Update template to remove quotes
//       and fill in literal "null" in place of -
// ^- but would break same format inserted into json and text/plain

String processor(const String& var){
	char buffer[50];
	
	if(var == "MILLIS") {
		sprintf(buffer, "%d", millis());
	}
 
	// TODO: millis() will rollover every ~50 days.  Use NTP here.
	else if (var == "UPTIME") {
		unsigned long m = millis();
		
		// > 1 day
		if (m > (1000 * 60 * 60 * 24)) {
			sprintf(buffer, "%0d day%s, %02d:%02dm",
				(int) (m / (1000 * 60 * 60 * 24)),
				(m >= (2 * 1000 * 60 * 60 * 24) ? "s" : ""),  // day vs days
				(int) (m / (1000 * 60 * 60) % 24),
				(int) (m / (1000 * 60) % 60));
		}
		// > 1 hour
		else if (m > (1000 * 60 * 60)) {
			sprintf(buffer, "%02d:%02d:%02ds",
				(int) (m / (1000 * 60 * 60) % 24),
				(int) (m / (1000 * 60) % 60),
				(int) (m / 1000 % 60));
		}
		else {
			sprintf(buffer, "%02d:%02ds",
				(int) (m / (1000 * 60) % 60),
				(int) (m / 1000 % 60));
		}
	}
 
	// TODO: since these are String(), can they be in a case statement for easier reading?
	else if (var == "LUMINANCE" && luxGood) {
		sprintf(buffer, "%0.2f", lux);
	}
 
	else if (var == "TEMPERATURE" && tempGood) {
		sprintf(buffer, "%0.2f", temp);
	}
 
	else if (var == "HUMIDITY" && humidityGood) {
		sprintf(buffer, "%0.2f", humidity);
	}

	else if (var == "MACADDR") {
		sprintf(buffer, macaddr);
	}
	
	// future use for a config.json type entry.
	// also want to capture build time (USEMQTT etc) vars
	else if (var == "VERSION") {
		sprintf(buffer, VERSION);
	}
	else if (var == "BUILDDATE") {
		sprintf(buffer, __DATE__);
	}
	else if (var == "BUILDTIME") {
		sprintf(buffer, __TIME__);
	}
	else if (var == "BUILDFILE") {
		sprintf(buffer, __FILE__);
	}
 
	// Catch all for any unrecognized variable name
	else {
		sprintf(buffer, "-");
	}  
	
	return buffer;
  
}






// mqtt publish update
//
// We always send "retain: true" with messages; the last seen state
// will be an appropriate value to use if a system restarts
//


boolean mqttConnect() {
  // are we already connected?
  //
  // note - this attempts to connect each time we send state; if we get more nuanced here
  // we may want a lastReconnectAttempt = millis() structure to not try to reconnect too often
  if (! mqttclient.connected() ) {
    Serial.print("Connecting to MQTT broker ... ");


  ///// TODO: don't mark values as unavailable just if the bridge goes unavailable
   ///// mark based on the AGE of the value.  we can have a timestamp on the data to age it out.

   
    // if (! mqttclient.connect(mqttIdent, mqttUser, mqttPass) ) {
  
    if (! mqttclient.connect(mqttIdent, mqttUser, mqttPass, "aq2mqtt/246F28035110/availability", 0, true, "offline") ) {  // todo: hardcoded
      Serial.println("failed.");
      return false;
    }
    else {
      mqttclient.publish("aq2mqtt/246F28035110/availability", "online", true); // no error checking on this one.
      Serial.println("ok.");


  // try to increase size of MQTT buffer.  

  // TODO: capture return code on failure and send shorter messages.
  // -- we would not be able to send autoconfig messages; also not able to send full JSON state messages with all data
      if ( mqttclient.setBufferSize(512) ) {
        mqttBufferSize = 512;
      }
      else {
        Serial.println("  Failed to set MQTT buffer size; we are unable to send JSON autoconfig entries.");
      }
      
      char msg[32];
      sprintf(msg, "%d", mqttBufferSize);
      mqttclient.publish("aq2mqtt/246F28035110/buffersize", msg, false);
      
    
    }
    
  }

    
  return true;

  
  
}


// General function to send autoconfig strings
void mqttSendAutoconfig() {

  // connect / reconnect if needed
  if (! mqttConnect()) {
    return;
  }

 
  char buff[64];
  char msg[32];

  Serial.print("Sending MQTT autoconfiguration strings ... ");

  if (mqttBufferSize < 512) {
    Serial.println("  MQTT Buffer not big enough for JSON autoconfig strings.  Not sent.");
    return;
  }

  Serial.println("  Not yet supported.  Please autoconfig manually.");   // TODO, of course
  
}





// Generic function to send all environment values
// - temp, humidity, lux
//
boolean mqttSendState() {

  char topic[64];  // topic
  char msg[512];   // message

  // connect / reconnect if needed
  if (! mqttConnect()) {
    return false;
  }
  

  
// - literal json string
  const char* json = R"({
  "lux": "%0.2f",
  "temp": "%0.2f",
  "humidity": "%0.2f",
  "particulate": { "pm1": "%0.2f", "pm25": "%0.2f", "pm4": "%0.2f", "pm10": "%0.2f" },
  "vocIndex": "%d",
  "noxIndex": "%d",
  "hardware": {
    "ipaddr": "%d.%d.%d.%d",
    "macaddr": "%s"
  }
})";
// end string


 // TODO: if isnan() then do not insert a number at all - blank, or literal "false" for invalid reading
 
    sprintf(msg, json, 
      luxGood ? lux : -1,
      isnan(ambientTemperature) ? -1 : ambientTemperature,
      isnan(ambientHumidity) ? -1 : ambientHumidity,

      isnan(massConcentrationPm1p0) ? -1 : massConcentrationPm1p0,
      isnan(massConcentrationPm2p5) ? -1 : massConcentrationPm2p5,
      isnan(massConcentrationPm4p0) ? -1 : massConcentrationPm4p0,
      isnan(massConcentrationPm10p0) ? -1 : massConcentrationPm10p0,
      
      isnan(vocIndex) ? -1 : (int)vocIndex,
      isnan(noxIndex) ? -1 : (int)noxIndex,
      WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3],
      macaddr
    );


    sprintf(topic, "aq2mqtt/%s/state", macaddr);

    //const char* shortj = R"({ "lux": "%0.2f", "temp": "%0.2f" })";
    //sprintf(msg, shortj, lux, ambientTemperature);
    
		Serial.print("Sending MQTT state ... ");

            
    if ( ! millis() > lastEnviron ) {
      Serial.println("  Waiting for sensors to settle... skipped.");
      return 0;
    }


	  if (mqttclient.publish(topic, msg, true)) {
		  
			Serial.println(" sent.");


      // save the "last SENT values"
      lastLux = lux;
      lastTemperature = ambientTemperature;
      lastHumidity = ambientHumidity;
      lastPM1 = massConcentrationPm1p0;
      lastPM25 = massConcentrationPm2p5;
      lastPM4 = massConcentrationPm4p0;
      lastPM10 = massConcentrationPm10p0;
      lastVoc = vocIndex;
      lastNox = noxIndex;

      temperatureChanged = 0; humidityChanged = 0;
      vocChanged = 0; noxChanged = 0;
      pm1Changed = 0; pm25Changed = 0; pm4Changed = 0; pm10Changed = 0;
      luxChanged = 0;

      sendStateNow = 0;
      
		}
		else {
		  Serial.println("failed to send.");
      return 0;
		}
			

    return 1;
	
	

}




///
void printModuleVersions() {
    uint16_t error;
    char errorMessage[256];

    unsigned char productName[32];
    uint8_t productNameSize = 32;

    error = sen5x.getProductName(productName, productNameSize);

    if (error) {
        Serial.print("Error trying to execute getProductName(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("  ProductName: ");
        Serial.println((char*)productName);
    }

    uint8_t firmwareMajor; uint8_t firmwareMinor;
    bool firmwareDebug;
    uint8_t hardwareMajor; uint8_t hardwareMinor;
    uint8_t protocolMajor; uint8_t protocolMinor;

    error = sen5x.getVersion(firmwareMajor, firmwareMinor, firmwareDebug,
                             hardwareMajor, hardwareMinor, protocolMajor,
                             protocolMinor);
    if (error) {
        Serial.print("Error trying to execute getVersion(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("  Firmware: ");
        Serial.print(firmwareMajor);
        Serial.print(".");
        Serial.print(firmwareMinor);
        
        Serial.print("  Hardware: ");
        Serial.print(hardwareMajor);
        Serial.print(".");
        Serial.print(hardwareMinor);

        Serial.print("  Protocol: ");
        Serial.print(protocolMajor);
        Serial.print(".");
        Serial.println(protocolMinor);
    }
}

// TODO: capture serial number from SEN5x and inject into our json data
void printSerialNumber() {
    uint16_t error;
    char errorMessage[256];
    unsigned char serialNumber[32];
    uint8_t serialNumberSize = 32;

    error = sen5x.getSerialNumber(serialNumber, serialNumberSize);
    if (error) {
        Serial.print("Error trying to execute getSerialNumber(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("  SerialNumber: ");
        Serial.println((char*)serialNumber);
    }
}


void pollsen() {
    uint16_t error;
    char errorMessage[256];


    error = sen5x.readMeasuredValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
        massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
        noxIndex);

    if (error) {
        Serial.print("Error trying to execute readMeasuredValues(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }
    else {

    // Did our environment state change significantly compared to the last time we sent state to MQTT?
    // Amount of change depends on the measurement.
    // Save specifically which value changed so we can mark it in our diagnostic output

    // TODO: some of these look like they don't work correctly

    // TODO: if we disconnect from mqtt server, we COULD force a resend to reconnect rather 
    if (abs(lastTemperature - ambientTemperature) >= 0.4) {
      sendStateNow = 1;
      temperatureChanged = 1;
    }

    if (abs(lastHumidity - ambientHumidity) >= 5) {
      sendStateNow = 1;
      humidityChanged = 1;
    }

    if (abs(lastVoc - vocIndex) > 5) {
      sendStateNow = 1;
      vocChanged = 1;
    }

    if (abs(lastNox - noxIndex) > 5) {
      sendStateNow = 1;
      noxChanged = 1;
    }

    if ( (lastPM1  != 0 && massConcentrationPm1p0  != 0) && ( ( (lastPM1  / massConcentrationPm1p0) > 1.5) || (lastPM1  / massConcentrationPm1p0) < 0.6) ) {
      sendStateNow = 1;
      pm1Changed = 1;
    }
    
    if ( (lastPM25 != 0 && massConcentrationPm2p5  != 0) && ( ( (lastPM25 / massConcentrationPm2p5) > 1.5) || (lastPM25 / massConcentrationPm2p5) < 0.6) ) {
      sendStateNow = 1;
      pm25Changed = 1;
    }

    if ( (lastPM4  != 0 && massConcentrationPm4p0  != 0) && ( ( (lastPM4  / massConcentrationPm4p0) > 1.5) || (lastPM4  / massConcentrationPm4p0) < 0.6) ) {
      sendStateNow = 1;
      pm4Changed = 1;
    }

    if ( (lastPM10 != 0 && massConcentrationPm10p0 != 0) && ( ( (lastPM10 / massConcentrationPm1p0) > 1.5) || (lastPM10  / massConcentrationPm10p0) < 0.6) ) {
      sendStateNow = 1;
      pm10Changed = 1;
    }
    



      // Print measurements once they have stabilized (== once they are marked to send via MQTT)
      // TODO: move this to loop() and print Lux as well for diags
      if ( millis() > lastEnviron ) {
        Serial.print("  Temp: ");    Serial.print(temperatureChanged ? "*" : ""); Serial.print(ambientTemperature);
        Serial.print("\t Humd: ");   Serial.print(humidityChanged ? "*" : "");    Serial.print(ambientHumidity);

        Serial.print("\t Voc: ");    Serial.print(vocChanged ? "*" : "");         Serial.print(vocIndex);
        Serial.print("\t Nox: ");    Serial.print(noxChanged ? "*" : "");         Serial.print(noxIndex);
        
        Serial.print("\t PM1: ");    Serial.print(pm1Changed ? "*" : "");         Serial.print(massConcentrationPm1p0);
        Serial.print("\t PM2.5: ");  Serial.print(pm25Changed ? "*" : "");        Serial.print(massConcentrationPm2p5);
        Serial.print("\t PM4: ");    Serial.print(pm4Changed ? "*" : "");         Serial.print(massConcentrationPm4p0);
        Serial.print("\t PM10: ");   Serial.print(pm10Changed ? "*" : "");        Serial.print(massConcentrationPm10p0);

        Serial.println("");
      }  
    }


    
}


///


void setup() {
	// Setup
	Serial.begin(115200);
  Serial.println("");
  Serial.println("");
	Serial.println("Air Quality monitoring system starting.");
  Serial.println("");


  // TODO - set up a button on a GPIO that when pressed does an immediate send.

  
  // Look for VEML7700 lux sensor

  // Setup VEML7700 lux sensor
  // TODO: in testing, if I unplug the lux sensor it starts
  // giving unreasonably high readings.  If we see this, we can
  // just disable it - flag as "unavailable".
  if (!veml.begin()) {
    Serial.println("Lux sensor not found");
    luxPresent = 0;
  }
  else {
    luxPresent = 1;
    Serial.println("Lux sensor found - initializing");
    veml.setGain(VEML7700_GAIN_1);
    veml.setIntegrationTime(VEML7700_IT_800MS);
    
    Serial.print(F("  Gain: "));
    switch (veml.getGain()) {
      case VEML7700_GAIN_1: Serial.println("1"); break;
      case VEML7700_GAIN_2: Serial.println("2"); break;
      case VEML7700_GAIN_1_4: Serial.println("1/4"); break;
      case VEML7700_GAIN_1_8: Serial.println("1/8"); break;
    }
    
    Serial.print(F("  Integration Time (ms): "));
    switch (veml.getIntegrationTime()) {
      case VEML7700_IT_25MS: Serial.println("25"); break;
      case VEML7700_IT_50MS: Serial.println("50"); break;
      case VEML7700_IT_100MS: Serial.println("100"); break;
      case VEML7700_IT_200MS: Serial.println("200"); break;
      case VEML7700_IT_400MS: Serial.println("400"); break;
      case VEML7700_IT_800MS: Serial.println("800"); break;
    }
    
    //veml.powerSaveEnable(true);
    //veml.setPowerSaveMode(VEML7700_POWERSAVE_MODE4);
    
    // Disabling these - we just poll; don't need interrupts on thresholds
    //veml.setLowThreshold(10000);
    //veml.setHighThreshold(20000);
    //veml.interruptEnable(true);

    Serial.println("");
  } // setup lux



  // SEN5x setup -- poll every 30 seconds
  delayEnviron = 30000;

  // note -- sen5x uses the Wire to talk I2C, but veml7700 has something else internal
  Wire.begin();

  sen5x.begin(Wire);
    uint16_t error;
    char errorMessage[256];
    error = sen5x.deviceReset();
    if (error) {
        Serial.println("Environment sensor not found");
        Serial.print("  Error trying to execute deviceReset(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
        aqPresent = 0;
    }
    else {
      aqPresent = 1;
    }

  if (aqPresent) {
    // Print SEN55 module information (i2c buffers are always large enough on our boards)
    Serial.println("Environmental sensor found - initializing");
    printModuleVersions();
    printSerialNumber();

  }


    // Start Measurement
    error = sen5x.startMeasurement();
    if (error) {
        Serial.print("Error trying to execute startMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);

        // disable
        aqPresent = 0;
    }

  
  Serial.println("");
  Serial.println("");

  
	


  
  


	Serial.println("Connecting to wifi ....");
	WiFi.mode(WIFI_STA);
	WiFi.begin(wifiName, wifiPass);
	Serial.println("");

	// Wait for connection to finish and print details.
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}
	Serial.println("");
	Serial.print("Connected to ");
	Serial.println(wifiName);
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());

	// Read and save mac address one time since it won't change
	char longmac[32];  // will contain colons, etc.
	sprintf(longmac, WiFi.macAddress().c_str());
	int i = 0; int j = 0; char c;
	while ((c = longmac[i++]) != '\0' && j <= 12)
		if (isalnum(c)) 
			macaddr[j++] = c;
	macaddr[j] = '\0';
	
	
	Serial.print("MAC address: ");
	Serial.println(macaddr);

#ifdef ESP32
	sprintf(mqttIdent, "esp32-%s", macaddr);
#else
  sprintf(mqttIdent, "esp8266-%s", macaddr);
#endif

	Serial.print("MQTT Client Ident: ");
	Serial.println(mqttIdent);
	Serial.println("");

	// Connect to MQTT for initial updates 
	Serial.println("Connecting to MQTT server for initial updates: ");



  // we don't have measurements yet, but have confirmed whether hardware is present and
  // can send our autoconfig JSON
  mqttSendAutoconfig();
  

  // todo -- print value 
  Serial.println("Waiting for sensors to settle ...");

}








// Run indefinitely.
void loop()
{

	
  
  
    // how often to take Temp and Humidity measurements ... this is every 5 seconds.
    
  

  
/*
	// refresh sensors, avoiding refreshing "too often"
	if (millis() - lastTempHumidity > delayTempHumidity && dhtPresent) {
		lastTempHumidity = millis();
		
		sensors_event_t event;
		dht.temperature().getEvent(&event);
		if (isnan(event.temperature)) {
			// TODO: on too many failures, disable future reads (dhtPresent = 0)
			// Serial.println("Reading temp failed; disabling");
			tempGood = 0;
		}
		else {
			float lastT = temp;
			temp = (event.temperature * 1.8) + 32;
			tempGood = 1;
			
			// more than 1 degree change?
			if (abs(temp - lastT) > 1.0) {
				sendStateNow = 1;
			}
		}
		
		// repeat for humidity
		dht.humidity().getEvent(&event);
		if (isnan(event.relative_humidity)) {
			// Serial.println("Reading humidity failed; disabling");
			humidityGood = 0;
		}
		else {
			float lastH = humidity;
			
			humidity = event.relative_humidity;
			humidityGood = 1;
			
			// more than 5 percent change?
			if (abs(humidity - lastH) > 5.0) {
				sendStateNow = 1;
			}

		}
	}

*/

  if (millis() - lastEnviron > delayEnviron && aqPresent) {

    pollsen();
    lastEnviron = millis();


    

 // TODO -- mark pmGood when they stabilise
   
// TODO -- mark "tempGood" when temp stabilizes ... appears to generate temp=0 on first run




  }

    
	// Note: trying to readLux() with the board absent will cause
	// the entire platform to crash.  So only poll if it's there
	if (millis() - lastLuminance > delayLuminance && luxPresent) {
		float lastL = lux;
		
		lastLuminance = millis();
		lux = veml.readLux();
		
		if (lux == 989560448.00) {   // from observation .... todo replace with a > for "randomly large number"
			Serial.println("Lux sensor out of range - disabling");
			luxPresent = 0;
			luxGood = 0;
		} else {
			// more than 5 point change?
			if (abs(lux - lastL) > 5.0) {
				sendStateNow = 1;
			}
			
			luxGood = 1;
		}

	}
	
	
	
	
	// if we are not connected, try to send now
	if (! mqttclient.connected() ) {
    Serial.println("MQTT became disconnected.");
    sendStateNow = 1;
	}
 
    
	
	// TODO: deprecate serial output.

	
	// Update stats when prompted, or every N milliseconds
	//
	// TODO: separate flags to update stats for different measurements
	// -- just because door opened doesn't mean we should push a temp update
	// -- ^ this is done; now need to split out temp/humid/lux to separate updates
	//
	if ( ( sendStateNow && millis() - lastEnvTime > envFreq && millis() > lastEnviron ) ||
	     ( millis() - lastEnvTime > envForceFreq) ) {
		
		sendStateNow = 0;
		
		if (mqttSendState()) {
		  lastEnvTime = millis();
     
		}
		
	}
	

	// TODO: do we need mqttclient.loop() when we're not subscribed to anything?
	mqttclient.loop();

	yield();  // or delay(0);
  
}
