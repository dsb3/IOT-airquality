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



**********************************************************/


#define VERSION "0.2.0"



// Private config held in separate file
// const char* wifiName = "SSID";
// const char* wifiPass = "password";
#include "private-config.h"


// determine board, for the specific boards I'm developing against

// code will fail to compile if neither of these is true, as no Wifi include is made

#ifdef ARDUINO_ESP8266_ADAFRUIT_HUZZAH

#define boardident "esp8266"
#define BOARDNAME "Adafruit Huzzah ESP8266"
#include <ESP8266WiFi.h>

#endif

#ifdef ARDUINO_FEATHER_ESP32

#define boardident "esp32"
#define BOARDNAME "Adafruit Feather ESP32"
#include <WiFi.h>

#endif


// save macaddr during setup()
char macaddr[15] = "";



#include <PubSubClient.h>


// * these must be defined in private-config.h 
//const char* mqttServer = "mqtt.example.com";
//const int   mqttPort = 1883;
//const char* mqttUser = "username";
//const char* mqttPass = "password";

// generate and save during setup()
char mqttIdent[64] = "";


// default value from headers; we need to increase this from default 256 bytes for oversize JSON data
int mqttBufferSize = MQTT_MAX_PACKET_SIZE;








// SEN5x - https://github.com/Sensirion/arduino-i2c-sen5x/blob/master/examples/exampleUsage/exampleUsage.ino
#include <SensirionI2CSen5x.h>
#include <Wire.h>

// SEN5x docs talk about I2C_BUFFER_LENGTH being bigger than some Arduino implementation; not a concern for
// the boards I'm using so I've stripped from the code sample I used
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
float lastLux = 0;
boolean luxChanged = 0;

    





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


// timestamp of when we can next connect -- on mqtt failure we push it into the future
unsigned long nextMqttConnect = 0;


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

 
	// Catch all for any unrecognized variable name
	else {
		sprintf(buffer, "-");
	}  
	
	return buffer;
  
}






// mqtt publish updates
//
// We always send "retain: true" with messages; the last seen state
// will be an appropriate value to use if a system restarts
//


boolean mqttConnect() {
  // if we are not already connected ...
  if (! mqttclient.connected() ) {

    // ... and we didn't try to connect (and fail) too recently ...
    if (millis() > nextMqttConnect) {
    
      // ... then try to connect
      Serial.print("Connecting to MQTT broker ... ");
  
      if (! mqttclient.connect(mqttIdent, mqttUser, mqttPass, "aq2mqtt/246F28035110/availability", 0, true, "offline") ) {  // todo: hardcoded - update to serial of sen5x
        Serial.println("failed.");
        nextMqttConnect = millis() + 60000;   // next allowed in 60 seconds
        return false;
      }
      else {
        mqttclient.publish("aq2mqtt/246F28035110/availability", "online", true); // no error checking on this one.
        Serial.println("ok.");

        // try to set; and then read it back; if it wasn't able to be set we report when we try to use it
        mqttclient.setBufferSize(512);
        mqttBufferSize = mqttclient.getBufferSize();
      
        char msg[32];
        sprintf(msg, "%d", mqttBufferSize);
        mqttclient.publish("aq2mqtt/246F28035110/buffersize", msg, false);
    
      }
    
    }


  }

  nextMqttConnect = 0;  // success; we can reconnect if needed at any time
  return true;

  
  
}


// General function to send hardware configuration
//
// TODO - some of these values may change; if that happens, detect and resend
// -- sensors may go unavailable, so disable in info
//    wifi might reconnect with a different IP

boolean mqttSendHardwareInfo() {

  // connect / reconnect if needed
  if (! mqttConnect()) {
    return 0;
  }
 
  char topic[64];
  char msg[512];

  if (mqttBufferSize < 512) {
    Serial.println("MQTT Buffer not big enough for hardware information string.  Not sent.");
    return 0;
  }

// - literal json string
  const char* json = R"({
    "board": { "model": "%s" },
    "connection": { "ipaddr": "%d.%d.%d.%d", "macaddr": "%s" },
    "software": { "version": "%s", "compiledate": "%s", "compiletime": "%s" },
    "sensors": {
       "lux": { "model": "veml7700" },
       "env": { "model": "sen55", serial: "%s", firmware: "%s" }
    }
})";
// end string

    sprintf(msg, json, 
      BOARDNAME,
      WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3],
      macaddr,
      __VERSION__, __DATE__, __TIME__,
      "senserial",
      "firm1.x"
    );

    sprintf(topic, "aq2mqtt/%s/hardware", macaddr);   // TODO: data should be tagged with serial number of the SEN5x, not the mac of the board.

    Serial.print("Sending MQTT hardware configuration info ... ");    

    if (! mqttclient.publish(topic, msg, true)) {
      Serial.println("failed to send.");
      return 0;
    }
    else {
      Serial.println("ok.");
    }

  return 1;
  
}



// General function to send autoconfig strings
boolean mqttSendAutoconfig() {

  // connect / reconnect if needed
  if (! mqttConnect()) {
    return 0;
  }

 
  char buff[64];
  char msg[32];

  Serial.print("Sending MQTT autoconfiguration strings ... ");

  if (mqttBufferSize < 512) {
    Serial.println("  MQTT Buffer not big enough for JSON autoconfig strings.  Not sent.");
    return 0;
  }

  Serial.println("  Not yet supported.  Please autoconfig manually.");   // TODO, of course


  return 1;
  
}





// Generic function to send all environment values
// - temp, humidity, lux
//
boolean mqttSendState() {

  char topic[64];  // topic
  char msg[512];   // message

  if ( millis() <  30000 ) {  // meterSendFreq ) {  //todo rename settleTime
    
    Serial.println("Waiting for sensors to settle...");
    return 0;
  }


  // connect / reconnect if needed
  if (! mqttConnect()) {
    return 0;
  }
  

  
// - literal json string
  const char* json = R"({
   "env": { "lux": "%0.2f", "temp": "%0.2f", "humidity": "%0.2f" },
   "particulate": { "pm1": "%0.2f", "pm25": "%0.2f", "pm4": "%0.2f", "pm10": "%0.2f" },
   "index": { "voc": "%d", "nox": "%d" } 
})";
// end string


 // TODO: if isnan() then do not insert a number at all - blank, or literal "false" for unavailable reading
 
 // TODO: different sensors take different times to "settle" after startup
 //       we should omit (so they go unavailable) those rather than send bad values
 //
 // lux             = 5 seconds, but can delay to 30 to fit with the other "env" set
 // temp / humidity = 30 seconds
 // voc / nox       = 60 seconds
 // pmX             = unknown ... seems fairly quick.  perhaps 30 is enough?
 //
 
    sprintf(msg, json, 
      luxGood ? lux : -1,
      isnan(ambientTemperature) ? -1 : ambientTemperature,
      isnan(ambientHumidity) ? -1 : ambientHumidity,

      isnan(massConcentrationPm1p0) ? -1 : massConcentrationPm1p0,
      isnan(massConcentrationPm2p5) ? -1 : massConcentrationPm2p5,
      isnan(massConcentrationPm4p0) ? -1 : massConcentrationPm4p0,
      isnan(massConcentrationPm10p0) ? -1 : massConcentrationPm10p0,
      
      isnan(vocIndex) ? -1 : (int)vocIndex,
      isnan(noxIndex) ? -1 : (int)noxIndex

    );


    sprintf(topic, "aq2mqtt/%s/state", macaddr);   // TODO: data should be tagged with serial number of the SEN5x, not the mac of the board.

    
		Serial.print("Sending MQTT state ... ");
	  if (! mqttclient.publish(topic, msg, true)) {
      Serial.println("failed to send.");
      return 0;
    }
    else {
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

      // changes are based on the last values that were actually sent; not the last that were measured
      temperatureChanged = 0; humidityChanged = 0;
      vocChanged = 0; noxChanged = 0;
      pm1Changed = 0; pm25Changed = 0; pm4Changed = 0; pm10Changed = 0;
      luxChanged = 0;

      sendStateNow = 0;

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

    
    
    


    
}


///


void setup() {
	// Setup
	Serial.begin(115200);
  Serial.println("");
  Serial.println("");
	Serial.println("Air Quality monitoring system starting.");
  Serial.print("Compiled for "); Serial.println(BOARDNAME);
  Serial.println(__FILE__);
  Serial.print("Version "); Serial.print(VERSION);
    Serial.print(" - "); Serial.print(__DATE__); Serial.print(" "); Serial.println(__TIME__);
  
  Serial.println("");


  // TODO - set up a button on a GPIO that when pressed does an immediate send.

  
  // Look for VEML7700 lux sensor

  // Setup VEML7700 lux sensor
  // TODO: in testing, if I unplug the lux sensor after the board boots, 
  // it starts giving unreasonably high readings.  If we see this, we can
  // just disable it and flag as "unavailable".
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
    
    // Disabling these - we poll frequently enough to not need interrupts
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

    // Start Measurement
    error = sen5x.startMeasurement();
    if (error) {
      Serial.print("Error trying to execute startMeasurement(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
      // disable
      aqPresent = 0;
    }
    
  }

  
  Serial.println("");
  Serial.println("");

  
	


	Serial.print("Searching for wifi SSID ");
  Serial.print(wifiName);
  
	WiFi.mode(WIFI_STA);
	WiFi.begin(wifiName, wifiPass);
	Serial.println("");

	// Wait for connection to finish and print details.
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}
	Serial.println("");
	Serial.print("Connected - ");
	Serial.println(WiFi.localIP());

	// Read and save mac address one time since it won't change
	char longmac[32];  // will contain colons, etc.
	sprintf(longmac, WiFi.macAddress().c_str());
	int i = 0; int j = 0; char c;
	while ((c = longmac[i++]) != '\0' && j <= 12)
		if (isalnum(c)) 
			macaddr[j++] = c;
	macaddr[j] = '\0';
	

  // MQTT identifier is esp32-(mac), or esp8266-(mac) for later use
	sprintf(mqttIdent, "%s-%s", boardident, macaddr);


  // we don't have measurements yet, but have confirmed whether hardware is present and
  // can send our autoconfig JSON
  mqttSendAutoconfig();

  // Send board / hardware info
  mqttSendHardwareInfo();
  

}








// Run indefinitely.
void loop()
{


  bool readLux = 0;
  bool readEnv = 0;

  // todo - we can read lux more often.  perhaps every 2.5 seconds versus 5.0 for envs.
    if (millis() - lastEnviron > delayEnviron && luxPresent) {

    // poll lux sensor
    lux = veml.readLux();

    if (lux > 500000000) {  // from observation errors == 989560448.00
      Serial.println("Lux sensor out of range - disabling");
      luxPresent = 0;
      luxGood = 0;
    } else {
      
      luxGood = 1;
    }

    readLux = 1;
  }


  if (millis() - lastEnviron > delayEnviron && aqPresent) {
    // poll environmental sensor
    pollsen();

    readEnv = 1;
  }




  // did we take a reading from either sensor ...
  if (readLux || readEnv) {
    
    // ... and if so, did any values change "significantly" from the last time they were sent via MQTT?
    // Amount of change depends on the measurement - absolute, or percentage.
    //
    // We also save which value changed to flag it in the diagnostic output.
    //
    
    if (abs(lastTemperature - ambientTemperature) >= 0.4) {
      sendStateNow = 1;
      temperatureChanged = 1;
    }

    if (abs(lastHumidity - ambientHumidity) >= 4) {
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

    // be careful of divide by zero on comparisons
    if ( (lastPM1  > 0 && massConcentrationPm1p0  > 0) && ( ( (lastPM1  / massConcentrationPm1p0) > 1.5)  || (lastPM1  / massConcentrationPm1p0) < 0.6) ) {
      sendStateNow = 1;
      pm1Changed = 1;
    }
    
    if ( (lastPM25 > 0 && massConcentrationPm2p5  > 0) && ( ( (lastPM25 / massConcentrationPm2p5) > 1.5)  || (lastPM25 / massConcentrationPm2p5) < 0.6) ) {
      sendStateNow = 1;
      pm25Changed = 1;
    }

    if ( (lastPM4  > 0 && massConcentrationPm4p0  > 0) && ( ( (lastPM4  / massConcentrationPm4p0) > 1.5)  || (lastPM4  / massConcentrationPm4p0) < 0.6) ) {
      sendStateNow = 1;
      pm4Changed = 1;
    }

    if ( (lastPM10 > 0 && massConcentrationPm10p0 > 0) && ( ( (lastPM10 / massConcentrationPm10p0) > 1.5) || (lastPM10  / massConcentrationPm10p0) < 0.6) ) {
      sendStateNow = 1;
      pm10Changed = 1;
    }
    

    if ( (lastLux > 0 && lux > 0) && ( ( (lastLux / lux) > 1.5) || (lastLux  / lux) < 0.6) ) {
      sendStateNow = 1;
      luxChanged = 1;
    }



    // Print measurements once they have stabilized (== once they are marked to send via MQTT)
    // TODO: move this to loop()
    // TODO: rename lastEnviron to "settle time" or similar
    if ( millis() > lastEnviron ) {
      Serial.print("  T: ");       Serial.print(temperatureChanged ? "*" : ""); Serial.print(ambientTemperature);
      Serial.print("\t RH: ");     Serial.print(humidityChanged ? "*" : "");    Serial.print(ambientHumidity);
      
      Serial.print("\t Voc: ");    Serial.print(vocChanged ? "*" : "");         Serial.print(vocIndex);
      Serial.print("\t Nox: ");    Serial.print(noxChanged ? "*" : "");         Serial.print(noxIndex);
      
      Serial.print("\t PM1: ");    Serial.print(pm1Changed ? "*" : "");         Serial.print(massConcentrationPm1p0);
      Serial.print("\t PM2.5: ");  Serial.print(pm25Changed ? "*" : "");        Serial.print(massConcentrationPm2p5);
      Serial.print("\t PM4: ");    Serial.print(pm4Changed ? "*" : "");         Serial.print(massConcentrationPm4p0);
      Serial.print("\t PM10: ");   Serial.print(pm10Changed ? "*" : "");        Serial.print(massConcentrationPm10p0);
      
      Serial.print("\t Lux: ");    Serial.print(luxChanged ? "*" : "");         Serial.print(lux);
      Serial.println("");
    }  

    lastEnviron = millis();


  }


  
	
	
	
	// if we became disconnected, flag to resend
	if (! mqttclient.connected() ) {
    Serial.println("MQTT became disconnected.");
    sendStateNow = 1;
	}
 
    
			
	// Update stats when prompted, or every N milliseconds
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
