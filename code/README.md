# WiFi Septic Controller Sketch
The WiFi Septic Controller Sketch is written in C++. The sketch is primarily two state machines: one for effluent pump control and one for alarm state. The pump state machine can detect when the override float has applied power to the pump thus providing some indication of over usage or rain seepage into the tanks. The sketch is a discoverable set of sensors for [Home Assistant](https://home-assistant.io/), an open-source home automation platform running on Python. [MQTT](http://mqtt.org/), a machine-to-machine (M2M)/"Internet of Things" connectivity protocol, is the basis of communication with Home Assistant.

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
#define ENABLE_OTA_UPDATES
// Enable Serial on USB
//#define ENABLE_SERIAL
// Enable Low Power Mode on WiFi
#define ENABLE_WIFI_LOW_POWER
// Current Version
#define VERSION                   "0.1"
```

* "ENABLE_WATCHDOG" - When defined, the Watchdog Timer is enabled with a 16 second timeout. This will reset the ARM processor is something goes bad.
* "ENABLE_OTA_UPDATES" - When defined, the sketch can be updated using Arduino's Over-The-Air Update capability.
* "ENABLE_SERIAL" - When defined, status information is sent out the serial connection which in this case is the USB port. Otherwise the serial port won't even be enabled.
* When "ENABLE_WIFI_LOW_POWER" is defined the WINC1500 module is set to Low Power Mode. This can drop the current requirements of the module to a third or even less. Low Power Mode reduces the transmit frequency to the beacon interval and may also cause the module to hang occasionally.

```c
/******************************************************************
 * Application defines
 ******************************************************************/
// Name of this board, each board should be unique
// Used as MQTT Client ID, HASS Name, and OTA Name
// Used as MQTT Client ID, HASS Name, and OTA Name
#define BOARD_NAME                "Septic"
// input debounce time in milliseconds
#define DEBOUNCE_TIME             100
// pump toggle switch input debounce time in milliseconds
//   Should be larger than DEBOUNCE_TIME
#define PUMP_TOGGLE_DEBOUNCE_TIME 500
// pump toggle switch input debounce time in milliseconds
//   Should be larger than DEBOUNCE_TIME
#define ALARM_DEBOUNCE_TIME       500
// pump state machine deadband time in milliseconds
#define PUMP_DEADBAND_TIME        2000
// alarm state machine deadband time in milliseconds
#define ALARM_DEADBAND_TIME       2000
// temperature publish time in milliseconds when pump is off
#define TEMP_PUBLISH_RATE         5 * 60 * 1000
// timezone difference from GMT in hours (Standard Time difference)
#define TZDIFF                    -6
// Solid State Relay shutoff temperature in Celsius
#define SSR_SHUTOFF_TEMP          65.0

/******************************************************************
 * Home Assistant MQTT Defines
 ******************************************************************/
#define HASS_PREFIX               "hass"
#define HASS_NODE_NAME            "septic"
```
* "BOARD_NAME" - Names the board. It is used is several places including Home Assistant Name, OTA Name, and MQTT Client ID. This is a string.
* "DEBOUNCE_TIME" - How long to ignore sense changes on the input before accepting them as valid. This is an integer and the units are milliseconds.
* "PUMP_TOGGLE_DEBOUNCE_TIME" - How long to ignore contact changes on Pump Toggle switch. This is an integer and the units are milliseconds.
* "ALARM_DEBOUNCE_TIME" - How long to ignore contact changes on Alarm switch inputs. This is an integer and the units are milliseconds. Should be significantly larger than DEBOUNCE_TIME.
* "PUMP_DEADBAND_TIME" - How long after the Pump State Machine changes states before allowing another state change. This is an integer and the units are milliseconds.
* "ALARM_DEADBAND_TIME" - How long after the Alarm State Machine changes states before allowing another state change. This is an integer and the units are milliseconds.
* "TEMP_PUBLISH_RATE" - How often the temperature should be sampled and published via MQTT. This is an integer and the units are milliseconds. The time should not be less than 5000 milliseconds.
** "TZDIFF" - Used to correct internet time for your given timezone. This should be the Standard Time timezone difference to GMT. It is a integer and units are hours.
* "SSR_SHUTOFF_TEMP" - Specifies the temperature is which the pump state machine will shut off the SSR. It is a float.
* "HASS_PREFIX" - The Home Assistant MQTT Discovery Prefix as defined in your system. This is a string.
* "HASS_NODE_NAME" - Used in the MQTT topics to identify the cover and sensor to Home Assistant. Home Assistant calls this the node id. It is a string and should not contain special characters including spaces.

## Home Assistant Setup
The sketch is setup to enable MQTT Discovery on Home Assistant. If you don't want to use discovery here is the configuration of the WiFi Septic Controller in Home Assistant. Note the 'Septic' in the name: definitions you see in the example yaml is the "BOARD_NAME" and 'septic' in topics is "HASS_NODE_NAME". The WiFi Septic Controller uses the [MQTT Sensor](https://home-assistant.io/components/sensor.mqtt/) and [MQTT Binary Sensor](https://www.home-assistant.io/components/binary_sensor.mqtt/) platforms.

```yaml
# Example configuration.yaml entry
sensor:
  # Septic Controller State
  - platform: mqtt
    name: "Septic Status"
    state_topic: "hass/sensor/septic/status/state"
  # Septic Controller Pump State
  - platform: mqtt
    name: "Septic Pump State"
    state_topic: "hass/binary_sensor/septic/pump/state"
  # Septic Controller Alarm State
  - platform: mqtt
    name: "Septic Alarm State"
    state_topic: "hass/binary_sensor/septic/alarm/state"
  # Septic Controller temperature
  - platform: mqtt
    name: "Septic Temperature"
    state_topic: "hass/sensor/septic/temperature/state"
    unit_of_measurement: "°C"
  # Septic Controller RSSI
  - platform: mqtt
    name: "Septic RSSI"
    state_topic: "hass/sensor/septic/rssi/state"
    unit_of_measurement: "dBm"
```

Septic Status can be one of the following:
* "Overtemp Alarm On" - The SSR temperature is too high and the effluent pump has been turned off. The pump will come back on when the temperature has dropped 15 C. Most likely the effluent pump is drawing too much current or the SSR is failing.
* "Tank High Alarm On" - The top float in the tank is on. Most likely the effluent pump is not working.
* "Air Pump Alarm On" - The air compressor is not producing any air pressure.
* "Bleach Alarm On" -  Bleach level is low.
* "Manual Pump On" - Effluent pump was manually turned on.
* "Auto Pump On" - Effluent pump was automatically turned on at 1:00AM.
* "Override Pump On" - The middle/override float is on which also turned on the effluent pump. This usually means too much water has entered the septic system.
* "Tank Empty" - This a short state that appears for a few seconds after pumping indicating the bottom float is off thus stopping any pumping state.
* "Idle" - Nothing to do except look pretty.

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
