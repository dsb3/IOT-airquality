// This is only suitable for very small files;
//   literal R (raw string) is C++11 only
//
// This is the JSON payload for all measurements
//

const char* all_json = R"(

{
  "status": {
    "wifiLevel": "medium",
    "lastUpdate": 1708614620000
  },

  "diags": {
    "board": "%BOARD%",
    "macaddr": "%MACADDR%",
    "ipaddr": "%IPADDR%",
    "mqttTx": {
      "successful": "%mqttSendSuccess%",
      "failed": "%mqttSendFail%"
    },
    "uptime": "%UPTIME%",
    "millis": "%MILLIS%"
  },

  "lux": {
    "enabled": "1"

  },

  "environ": {
    "enabled": "1"

  }
    
}

  
}
)";

