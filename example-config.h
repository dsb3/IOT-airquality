
// Development flags
#define USEMYQTTHUB    // bogus: remove this soon
#define FASTTIMERS

// Uncomment/comment to enable or disable different options for data upload
#define ENABLEMQTT


// Use this file to store all of the private credentials 
// and connection details

const char* wifiName = "xxx";
const char* wifiPass = "yyy";




// MQTT - e.g. free low volume accounts available here
// the "ifdef USEMYQTTHUB" is just here to let me quickly switch between
// public mqtt broker (where I have usage limits), and my development
// internal one (where I don't).  It'll be removed at some time.
#ifdef USEMYQTTHUB
const char* mqttServer = "node02.myqtthub.com";
const int   mqttPort   = 1883;
#define mqttFixedIdent   "required for myqtthub!"
const char* mqttUser   = "yyy";
const char* mqttPass   = "zzz";
#else
const char* mqttServer = "other.example.com";
const int   mqttPort = 1883;
const char* mqttUser = "aaa";
const char* mqttPass = "bbb";
#endif




// Timers - all are based on millis()
//
const int tempReadFreq =     5000;  // refresh temp sensors every 5 seconds



#ifdef FASTTIMERS

// Very short timers for debugging  - upload stats much faster
const int tempSendFreq =    10000;  // 10s
const int boardSendFreq =   60000;  // 60s


#else

const int tempSendFreq =   300000;  // upload temp sensors: 5 minutes
const int boardSendFreq = 1800000;  // upload board stats: 30 minutes


#endif


