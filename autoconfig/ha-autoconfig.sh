#!/bin/sh

# mac address of ESP32 board
#  // TODO - replace with serial of SEN5x sensor
SERIAL=246F28035110


for autoconfig in sensor-*json; do
  autotype=$( echo $autoconfig | cut -d- -f1 )                 # type of entry: sensor, binary_sensor, etc
  autoname=$( echo $autoconfig | cut -d- -f2 | cut -d. -f1 )   # name of entry

  mosquitto_pub -r -h $MQTTHOST -u $MQTTUSER -P $MQTTPASS -t "homeassistant/$autotype/$SERIAL/$autoname/config" -f $autoconfig


done


