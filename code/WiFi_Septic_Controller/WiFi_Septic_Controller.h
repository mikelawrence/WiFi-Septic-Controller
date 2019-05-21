/*
  Septic WiFi Controller Project wide defines

  Copyright (c) 2019 Mike Lawrence

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#ifndef WIFI_SEPTIC_CONTROLLER_H
#define WIFI_SEPTIC_CONTROLLER_H

/******************************************************************
 * Build defines
 ******************************************************************/
// Enable Watchdog Timer
#define ENABLE_WATCHDOG
// Enable OTA updates
#define ENABLE_OTA_UPDATES
// Enable Serial on USB
#define ENABLE_SERIAL
// Enable Low Power Mode on WiFi
//#define ENABLE_WIFI_LOW_POWER
// Current Version
#define VERSION                   "0.5"

/******************************************************************
 * Application defines
 ******************************************************************/
// Name of this board, each board should be unique
// Used as MQTT Client ID, HASS Name, and OTA Name
#define BOARD_NAME                "Septic"
// input debounce time in milliseconds
#define DEBOUNCE_TIME             100
// pump toggle switch input debounce time in milliseconds
//   Should be larger than DEBOUNCE_TIME
#define PUMP_TOGGLE_DEBOUNCE_TIME 500
// pump toggle switch input debounce time in milliseconds
//   Should be larger than DEBOUNCE_TIME
#define ALARM_DEBOUNCE_TIME       1000
// pump state machine deadband time in milliseconds
#define PUMP_DEADBAND_TIME        2000
// alarm state machine deadband time in milliseconds
#define ALARM_DEADBAND_TIME       2000
// temperature publish time in milliseconds when pump is off
#define TEMP_PUBLISH_RATE         1*60*1000
// NTP server used to update local time, can be IP Address or name
#define NTP_SERVER                "ntp.home"
//#define NTP_SERVER                "time.nist.gov"
// timezone difference from GMT in hours (Standard Time difference)
#define TZDIFF                    -6
// Solid State Relay shutoff temperature in Celsius
#define SSR_SHUTOFF_TEMP          65.0

/******************************************************************
 * Home Assistant MQTT Defines
 ******************************************************************/
#define HASS_PREFIX               "hass"
#define HASS_NODE_NAME            "septic"
// HASS defines below here should not be modified
#define HASS_TEMP_CONFIG_TOPIC    HASS_PREFIX "/sensor/" HASS_NODE_NAME "/temperature/config"
#define HASS_TEMP_STATE_TOPIC     HASS_PREFIX "/sensor/" HASS_NODE_NAME "/temperature/state"
#define HASS_TEMP_CONFIG          "{ \"name\": \"" BOARD_NAME " Temperature\", \"state_topic\": \"" HASS_TEMP_STATE_TOPIC \
                                  "\", \"unit_of_measurement\": \"\\u00b0C\" }"
#define HASS_RSSI_CONFIG_TOPIC    HASS_PREFIX "/sensor/" HASS_NODE_NAME "/rssi/config"
#define HASS_RSSI_STATE_TOPIC     HASS_PREFIX "/sensor/" HASS_NODE_NAME "/rssi/state"
#define HASS_RSSI_CONFIG          "{ \"name\": \"" BOARD_NAME " RSSI\", \"state_topic\": \"" HASS_RSSI_STATE_TOPIC \
                                  "\", \"unit_of_measurement\": \"dBm\" }"
#define HASS_PUMP_CONFIG_TOPIC    HASS_PREFIX "/binary_sensor/" HASS_NODE_NAME "/pump/config"
#define HASS_PUMP_STATE_TOPIC     HASS_PREFIX "/binary_sensor/" HASS_NODE_NAME "/pump/state"
#define HASS_PUMP_CONFIG          "{ \"name\": \"" BOARD_NAME " Pump\", \"state_topic\": \"" HASS_PUMP_STATE_TOPIC "\" }"

#define HASS_ALARM_CONFIG_TOPIC   HASS_PREFIX "/binary_sensor/" HASS_NODE_NAME "/alarm/config"
#define HASS_ALARM_STATE_TOPIC    HASS_PREFIX "/binary_sensor/" HASS_NODE_NAME "/alarm/state"
#define HASS_ALARM_CONFIG         "{ \"name\": \"" BOARD_NAME " Alarm\", \"state_topic\": \"" HASS_ALARM_STATE_TOPIC "\" }"

#define HASS_STATUS_CONFIG_TOPIC  HASS_PREFIX "/sensor/" HASS_NODE_NAME "/status/config"
#define HASS_STATUS_STATE_TOPIC   HASS_PREFIX "/sensor/" HASS_NODE_NAME "/status/state"
#define HASS_STATUS_CONFIG        "{ \"name\": \"" BOARD_NAME " Status\", \"state_topic\": \"" HASS_STATUS_STATE_TOPIC "\" }"

/******************************************************************
 * Board Defines
 ******************************************************************/
// Output pin defines
#define EFFLUENT_PUMP_RELAY       0
#define LCD_RS                    PIN_A1
#define LCD_E                     PIN_A2
#define LCD_D4                    PIN_A0
#define LCD_D5                    25
#define LCD_D6                    PIN_A5
#define LCD_D7                    PIN_A6
#define LCD_BACKLIGHT_R           PIN_A3
#define LCD_BACKLIGHT_G           PIN_A4
#define LCD_BACKLIGHT_B           11
// Output State defines
#define PUMP_RELAY_ACTIVE         HIGH
#define PUMP_RELAY_INACTIVE       LOW
// Input array defines
#define NUMBER_INPUTS             6
#define EFFLUENT_PUMP_SENSE       0
#define SPARE_SENSE               1
#define FLOAT_ALARM               2
#define AIR_ALARM                 3
#define BLEACH_ALARM              4
#define PUMP_TOGGLE_SWITCH        5
// Input State defines
#define SENSE_ACTIVE              LOW
#define SENSE_INACTIVE            HIGH
#define ALARM_ACTIVE              HIGH
#define ALARM_INACTIVE            LOW
#define PUMP_TOGGLE_ACTIVE        HIGH
#define PUMP_TOGGLE_INACTIVE      LOW
// LED
#define BOARD_LED                 6
#define ALARM_LED                 BOARD_LED
// 1-Wire pin
#define OWIRE                     2
// pin number array for the eight inputs on this board (pins are D7, MISO, D1, D4, SCK, MOSI)
const uint8_t input_pins[NUMBER_INPUTS] = {7, 10, 1, 4, 9, 8};

// Logging/Printing defines
#ifdef ENABLE_SERIAL
#define Print(...)                Serial.print(__VA_ARGS__)
#define Println(...)              Serial.println(__VA_ARGS__)
#define Log(...)                  {WiFiRTC.printTimeHMS24Hr();Serial.print(' ');Serial.print(__VA_ARGS__);}
#define Logln(...)                {WiFiRTC.printTimeHMS24Hr();Serial.print(' ');Serial.println(__VA_ARGS__);}
#else
#define Print(...)
#define Println(...)
#define Log(...)
#define Logln(...)
#endif

#endif // WIFI_SEPTIC_CONTROLLER_H
