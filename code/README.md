# WiFi Septic Controller Sketch
The WiFi Septic Controller Sketch is written in C++. The sketch is primarily two state machines one for effluent pump control and one for alarm state. The pump state machine can detect when the override float has applied power to the pump thus providing some indication of over usage or rain seepage into the tanks. The sketch is a discoverable set of sensors for [Home Assistant](https://home-assistant.io/), an open-source home automation platform running on Python. [MQTT](http://mqtt.org/), a machine-to-machine (M2M)/"Internet of Things" connectivity protocol, is the basis of communication with Home Assistant.

## Status
This software is only partially complete. The state machines have been tested on a lab bench but not connected to an actual septic controller. Home Assistant detects all MQTT sensors: Status Sensor, Pump Sensor, Alarm Sensor, Temperature Sensor, and RSSI Sensor.

# Setup
## Sketch Setup
The arduino_secrets.h file is not included on Github. You must create and edit it to match your configuration.

```c
/******************************************************************
 * arduino_secrets.h
 ******************************************************************/
#define SECRET_SSID             "SSID of your WiFi network"
#define SECRET_PASSWORD         "Password for your WIFi network"
// IP address of MQTT server, host name should work but I've never tried
#define MQTT_SERVER             "192.168.0.230"
#define MQTT_SERVERPORT         1883
// MQTT user name and password leave as empty string if not used
#define MQTT_USERNAME           ""
#define MQTT_PASSWORD           ""
// Over-The-Air Update password if used
#define OTA_PASSWORD            "password"
```

The rest of the sketch settings are C defines in the [WiFi_Septic_Controller.h](WiFi Septic_Controller/WiFi_Septic_Controller.h) file. There are a few build defines that control how the sketch is built.

```c
/******************************************************************
 * Build defines
 ******************************************************************/
// Enable Watchdog Timer
#define ENABLE_WATCHDOG
// Enable OTA updates
//#define ENABLE_OTA_UPDATES
// Enable Serial on USB
//#define ENABLE_SERIAL
// Current Version
#define VERSION                   "0.1"
```

* When "ENABLE_WATCHDOG" is defined the Watchdog Timer is enabled with a 16 second timeout. This will reset the ARM processor is something goes bad.
* When "ENABLE_OTA_UPDATES" is defined the sketch can be updated using Arduino's Over-The-Air Update capability.
* When "ENABLE_SERIAL" is defined then status information is sent out the serial connection which in this case is the USB port. Otherwise the serial port won't even be enabled.

```c
/******************************************************************
 * Application defines
 ******************************************************************/
// Name of this board, each board should be unique
// Used as MQTT Client ID, HASS Name, and OTA Name
#define BOARD_NAME                "Front Gate"
// input debounce time in milliseconds
#define DEBOUNCE_TIME             25
// output contact time in milliseconds
#define CONTACT_TIME              500
// output contact dead time between contact closures in milliseconds
#define CONTACT_DEADBAND_TIME     100
// temperature measurement time in milliseconds, Must be greater than 5 seconds
#define TEMP_RATE                 5*60*1000
/******************************************************************
 * Application defines
 ******************************************************************/
// Name of this board, each board should be unique
// Used as MQTT Client ID, HASS Name, and OTA Name
#define BOARD_NAME                "Septic"
// input debounce time in milliseconds
#define DEBOUNCE_TIME             25
// pump toggle switch input debounce time in milliseconds
#define PUMP_TOGGLE_DEBOUNCE_TIME 500
// pump toggle switch input debounce time in milliseconds (100 is DEBOUNCE_TIME = 25)
#define ALARM_DEBOUNCE_TIME       250
// pump state machine dead-band time in milliseconds
#define PUMP_DEADBAND_TIME        2000
// alarm state machine dead-band time in milliseconds
#define ALARM_DEADBAND_TIME       2000
// temperature publish time in milliseconds
#define TEMP_PUBLISH_RATE         5 * 60 * 1000
// timezone difference from GMT in hours (Standard Time difference)
#define TZDIFF                    -6

/******************************************************************
 * Home Assistant MQTT Defines
 ******************************************************************/
#define HASS_PREFIX               "hass"
#define HASS_NODE_NAME            "septic"
```
* "BOARD_NAME" names the board. It is used is several places including Home Assistant Name, OTA Name, and MQTT Client ID. This is a string.
* "DEBOUNCE_TIME" is how long to ignore contact changes on the input before accepting them as valid. This is an integer and the units are milliseconds.
* "PUMP_TOGGLE_DEBOUNCE_TIME" is how long to ignore contact changes on Pump Toggle switch. This is an integer and the units are milliseconds.
* "ALARM_DEBOUNCE_TIME" is how long to ignore contact changes on Alarm switch input. This is an integer and the units are milliseconds. Should be significantly larger than DEBOUNCE_TIME.
* "PUMP_DEADBAND_TIME" is how long after the Pump State Machine changes states before allowing another state change. This is an integer and the units are milliseconds.
* "ALARM_DEADBAND_TIME" is how long after the Alarm State Machine changes states before allowing another state change. This is an integer and the units are milliseconds.
* "TEMP_RATE" is how often the temperature should be sampled and updated via MQTT.  This is an integer and the units are milliseconds. The time must be greater than 5000 milliseconds.
* "TEMP_PUBLISH_RATE" is how often the temperature should be sampled and published via MQTT.  This is an integer and the units are milliseconds. The time must be greater than 5000 milliseconds.
* "HASS_PREFIX" is the Home Assistant MQTT Discovery Prefix as defined in your system. This is a string.
* "HASS_NODE_NAME" is used in the MQTT topics to identify the cover and sensor to Home Assistant. Home Assistant calls this the node id. It is a string and must not contain special characters including a space.

## Home Assistant Setup
The sketch is setup to enable MQTT Discovery on Home Assistant. If you don't want to use discovery here is the configuration of the WiFi Septic Controller in Home Assistant. Note the 'Septic' in the name: definitions you see in the example yaml is the "BOARD_NAME" and 'septic' in topics is "HASS_NODE_NAME". The WiFi Septic Controller uses the [MQTT Sensor](https://home-assistant.io/components/sensor.mqtt/) platform.

```yaml
# Example configuration.yaml entry
sensor:
  # Septic Controller State
  - platform: mqtt
    name: "Septic Status"
    state_topic: "hass/cover/septic/status/state"
  # Septic Controller Pump State
  - platform: mqtt
    name: "Septic Pump State"
    state_topic: "hass/cover/septic/pump/state"
  # Septic Controller Alarm State
  - platform: mqtt
    name: "Septic Alarm State"
    state_topic: "hass/cover/septic/alarm/state"
  # Septic Controller temperature
  - platform: mqtt
    name: "Septic Temperature"
    state_topic: "hass/cover/septic/temperature/state"
    unit_of_measurement: "°C"
  # Septic Controller RSSI
  - platform: mqtt
    name: "Septic RSSI"
    state_topic: "hass/cover/septic/rssi/state"
    unit_of_measurement: "dBm"
```

Once the board is running Home Assistant should automatically pick up the cover and sensors with MQTT Discovery. Debugging Home Assistant problems is a bit of a stretch for this guide but here are a couple of hints.

* Make sure you have MQTT installed. If you use HASS.IO goto the HASS.IO configuration and install the Mosquitto Broker.
* Make sure you have MQTT discovery enabled. See [MQTT Discovery](https://home-assistant.io/docs/mqtt/discovery/).
* Make sure your MQTT discovery prefix matches the HASS_PREFIX in the [WiFi_Septic_Controller.h](WiFi Septic_Controller/WiFi_Septic_Controller.h) file.

I use HASS.IO with the Mosquitto Broker add-on installed and my configuration for MQTT is as follows...

```yaml
mqtt:
  broker: core-mosquitto
  discovery: true
  discovery_prefix: hass

```
