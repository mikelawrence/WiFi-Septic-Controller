/*
  Septic WiFi Controller 
  
  This code interfaces a remote gate opener (US Automation) using a custom 
  SAMD/ATWINC1500C board. The board is compatible with an Arduino MKR1000.

  The following libraries must be installed using Library Manager:
  
    WiFi101 by Arduino
    WiFiOTA by Arduino
    RTCZero by Arduino
    LiquidCrystal by Arduino
    MQTT by Joel Gaehwiler
    OneWire by Paul Stoffregen and many others
    DallasTemperature by Miles Burton and others
  
  Copyright (c) 2018 Mike Lawrence

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
// Modify ../WiFi101/src/bsp/include/nm_bsp_internal.h library file
// so that CONF_PERIPH is defined. This will enabled LEDS from WiFi Module
#include <WiFi101.h>
#include <WiFi101OTA.h>
#include <MQTT.h>
#include <avr/dtostrf.h>
#include "WiFi_Septic_Controller.h"
#include "SepticLCD.h"
#include "WiFiRTC.h"
#include "arduino_secrets.h"

/******************************************************************
 * Definitions in arduino_secrets.h
 ******************************************************************/
/*
#define SECRET_SSID             "SSID of your WiFi network"
#define SECRET_PASSWORD         "Password for your WIFi network"
// IP address of MQTT server, host name may work but I've never tried
#define MQTT_SERVER             "192.168.0.230"
#define MQTT_SERVERPORT         1883
// MQTT user name and password leave as empty string if not used
#define MQTT_USERNAME           ""
#define MQTT_PASSWORD           ""
// Over-The-Air Update password if used
#define OTA_PASSWORD            "password"
*/

/******************************************************************
 * Defines
 ******************************************************************/
// Enumeration for pump state
enum PumpStateEnum {PS_RESET, PS_OFF, PS_OFF_A, PS_MANUAL_ON, PS_MANUAL_ON_A, PS_AUTO_ON, PS_OVER_ON, PS_ALARM_ON}; 
// Enumeration for alarm state
enum AlarmStateEnum {AS_RESET, AS_OFF, AS_TANK_HIGH, AS_AIR_PUMP, AS_BLEACH_LEVEL}; 

/******************************************************************
 * Global Variables
 ******************************************************************/
// Network
WiFiClient      net;
// MQTT CLient
MQTTClient      mqtt(1024);
// current input state, arranged in array by input_pins
uint8_t         inputState[NUMBER_INPUTS] = {-1, -1, -1, -1, -1, -1};
// when true a hardware reset just occurred
bool            resetOccurred = true;
// when true WiFi was recently disconnected from the network
bool            wifiDisconnectOccurred = true;
// when true WiFi was recently connected to the network
bool            wifiConnectOccurred = false;
// current state for Pump State machine
PumpStateEnum   curPumpState = PS_RESET;
// last state for Pump State machine
PumpStateEnum   lastPumpState = PS_RESET;
// current state for alarm state machine
AlarmStateEnum  curAlarmState = AS_RESET;
// last state for Alarm State machine
AlarmStateEnum  lastAlarmState = AS_RESET;

/******************************************************************
 * Is Alarm On?
 ******************************************************************/
bool isAlarmOn() {
  return curAlarmState == AS_TANK_HIGH ||
         curAlarmState == AS_AIR_PUMP ||
         curAlarmState == AS_BLEACH_LEVEL;
}

/******************************************************************
 * Is Pump On?
 ******************************************************************/
bool isPumpOn() {
  return curPumpState == PS_MANUAL_ON ||
         curPumpState == PS_MANUAL_ON_A ||
         curPumpState == PS_AUTO_ON ||
         curPumpState == PS_OVER_ON ||
         curPumpState == PS_ALARM_ON;
}

/******************************************************************
 * Display Pump State on LCD
 * 
 * force - Force Display Update
 *   When true forces a display update even on states normally ignored
 ******************************************************************/
void displayPumpState(bool force) {
  // display current pump state
  switch (curPumpState) {
    case PS_OFF:
      if (!isAlarmOn()) {
        SepticLCD.setDisplayMessage("Idle");
        SepticLCD.setBacklightColor(COLOR_IDLE);
      }
      break;
    case PS_OFF_A:
      Println("Entered PS_OFF_A");
      if (!isAlarmOn() && force) {
        SepticLCD.setDisplayMessage("Idle");
        SepticLCD.setBacklightColor(COLOR_IDLE);
      }
      break;
    case PS_MANUAL_ON:
      Println("Entered PS_MANUAL_ON");
      if (!isAlarmOn()) {
        SepticLCD.setDisplayMessage("Manual Pump On");
        SepticLCD.setBacklightColor(COLOR_PUMPING);
      }
      break;
    case PS_MANUAL_ON_A:
      Println("Entered PS_MANUAL_ON_A");
      if (!isAlarmOn() && force) {
        SepticLCD.setDisplayMessage("Manual Pump On");
        SepticLCD.setBacklightColor(COLOR_PUMPING);
      }
      break;
    case PS_AUTO_ON:
      Println("Entered PS_AUTO_ON");
      if (!isAlarmOn()) {
        SepticLCD.setDisplayMessage("Auto Pump On");
        SepticLCD.setBacklightColor(COLOR_PUMPING);
      }
      break;
    case PS_OVER_ON:
      Println("Entered PS_OVER_ON");
      if (!isAlarmOn()) {
        SepticLCD.setDisplayMessage("Override Pump On");
        SepticLCD.setBacklightColor(COLOR_PUMPING);
      }
      break;
    case PS_ALARM_ON:
      Println("Entered PS_ALARM_ON");
      if (!isAlarmOn()) {
        SepticLCD.setDisplayMessage("Alarm Pump On");
        SepticLCD.setBacklightColor(COLOR_PUMPING);
      }
      break;
    default:
      break;
  }
}

/******************************************************************
 * Standard Arduino setup function
 ******************************************************************/
void setup() {
  // configure effluent pump as an output and not on
  pinMode(EFFLUENT_PUMP_RELAY, OUTPUT);
  digitalWrite(EFFLUENT_PUMP_RELAY, LOW);

  // configure ALARM LED output
  pinMode(ALARM_LED, OUTPUT);
  digitalWrite(ALARM_LED, LOW);
  
  // configure inputs (all active high)
  for (int i = 0; i < NUMBER_INPUTS; i++) {
    pinMode(input_pins[i], INPUT);                    // set mode to input
  }
  
  // Start the WiFi Real Time Clock
  WiFiRTC.begin(TZDIFF, true);

  // Start the Septic LCD
  SepticLCD.begin();

  // Serial setup
  #ifdef ENABLE_SERIAL
  //while (!Serial);                                  // wait until serial monitor is open
  Serial.begin(115200);
  delay(2000);                                        // takes a bit for the USB to come up
  #endif
  
  // Announce who we are and software
  Println("\nWiFi Septic Controller: " BOARD_NAME);
  Println("  Software Version: " VERSION);
  
//Configure pins for Adafruit ATWINC1500 Feather
//WiFi.setPins(8,7,4,2);

  // WiFi setup
  if (WiFi.status() == WL_NO_SHIELD) {                // check for the presence of the WINC1500
    Println("WINC1500 not present! Nothing can be done!");
    // don't continue:
    while (true);
  }
  
  Println("WINC1500 Detected");
  
  // Display Firmware Version
  Print("  Firmware Version: ");
  Println(WiFi.firmwareVersion());
  Print("  Library Version: ");
  Println(WIFI_FIRMWARE_LATEST_MODEL_B);
        
  // Turn on WINC1500 WiFi module and connect to network now
  WiFi.begin(SECRET_SSID, SECRET_PASSWORD);

  // MQTT setup
  mqtt.setOptions(15*60, true, 5000);                 // keep Alive, Clean Session, Timeout
  mqtt.begin(MQTT_SERVER, MQTT_SERVERPORT, net);
  mqtt.onMessageAdvanced(messageReceived);
  
  #ifdef ENABLE_WATCHDOG
  // Set up the generic clock (GCLK2) used to clock the watchdog timer at 1.024kHz
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(4) |              // Divide the 32.768kHz clock source by divisor 32
                                                      //   where 2^(4 + 1): 32.768kHz/32=1.024kHz
                    GCLK_GENDIV_ID(2);                // Select Generic Clock (GCLK) 2
  while (GCLK->STATUS.bit.SYNCBUSY);                  // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_DIVSEL |            // Set to divide by 2^(GCLK_GENDIV_DIV(4) + 1)
                     GCLK_GENCTRL_IDC |               // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |             // Enable GCLK2
                     GCLK_GENCTRL_SRC_OSCULP32K |     // Set the clock source to the ultra low power oscillator (OSCULP32K)
                     GCLK_GENCTRL_ID(2);              // Select GCLK2         
  while (GCLK->STATUS.bit.SYNCBUSY);                  // Wait for synchronization

  // Feed GCLK2 to WDT (Watchdog Timer)
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |             // Enable GCLK2 to the WDT
                     GCLK_CLKCTRL_GEN_GCLK2 |         // Select GCLK2
                     GCLK_CLKCTRL_ID_WDT;             // Feed the GCLK2 to the WDT
  while (GCLK->STATUS.bit.SYNCBUSY);                  // Wait for synchronization

  REG_WDT_CONFIG = 0xBu;                              // Set the WDT reset timeout to 16384 clock cycles or 16s seconds
  while(WDT->STATUS.bit.SYNCBUSY);                    // Wait for synchronization
  REG_WDT_CTRL = WDT_CTRL_ENABLE;                     // Enable the WDT in normal mode
  while(WDT->STATUS.bit.SYNCBUSY);                    // Wait for synchronization
  #endif

  // check connections
  connect();
}

// Arduino loop function
void loop() {
  static uint32_t lastTempPublish = 0 - TEMP_PUBLISH_RATE;  // used to publish the temp at a defined rate
  static uint32_t lastRSSIPublish = 0 - TEMP_PUBLISH_RATE;  // used to publish the RSSI at a defined rate
  static uint32_t lastDebounceTime[NUMBER_INPUTS] = {0, 0, 0, 0, 0, 0};
  static uint8_t  lastDebounceInput[NUMBER_INPUTS] = {-1, -1, -1, -1, -1, -1};
  static uint32_t lastPSDeadbandTime;                 // add a deadband between pump state changes
  static uint32_t lastASDeadbandTime;                 // add a deadband between alarm state changes
  static uint32_t lastPumpOnTime = 0 - 4 * 60 * 60 * 1000;  // last pump time was 4 hours ago.
  static uint32_t startManualOnTime = 0;              // start time of Manual Pump On 
  static bool     publishPumpStatus = false;          // when true pump status should be published to MQTT server
  static bool     publishAlarmStatus = false;         // when true alarm status should be published to MQTT server
  PumpStateEnum   nextPumpState = curPumpState;       // next state for pump state machine, default is no change
  AlarmStateEnum  nextAlarmState = curAlarmState;     // next state for alarm state machine, default is no change
  uint32_t        lastPumpOnDiff;                     // how long has it been since pump was last on
  uint16_t        curHour = WiFiRTC.getHours();       // last hour this loop had
  uint8_t         curMinute = WiFiRTC.getMinutes();   // last minute this loop had
  float           temperature;                        // temperature reading
  char            tempStr[16];                        // temporary string
  uint8_t         inputReading;                       // last input read
  bool            inputStabilized = true;             // when true inputs are stabilized and are ready to be read
  
  // Reset the watchdog with every loop to make sure the sketch keeps running.
  watchdogReset();
  
  // keep track of time appropriately
  WiFiRTC.loop();

  #ifdef ENABLE_OTA_UPDATES
  // check for WiFi OTA updates
  WiFiOTA.poll();
  #endif

  // MQTT client loop call
  mqtt.loop();
  
  // debounce input
  for (int i = 0; i < NUMBER_INPUTS; i++) {
    // get current inut debounce time
    // read the current input state
    inputReading = digitalRead(input_pins[i]);
    // update last debounce time if the state has changed
    if (inputReading != lastDebounceInput[i]) {
      // reset the debounce timer
      lastDebounceTime[i] = millis();
    }
    uint32_t debounceTime = millis() - lastDebounceTime[i];
    // if debounce time has elapsed then the input is stable
    if (debounceTime > DEBOUNCE_TIME) {
      // debounce finished 
      if (i == PUMP_TOGGLE_SWITCH) {
        // were are working the pump toggle input and rising edge
        if (debounceTime > PUMP_TOGGLE_DEBOUNCE_TIME) {
          // debounce was long enough
          inputState[i] = inputReading;               // update input state
        }
      } else if (i == FLOAT_ALARM ||
                 i == AIR_ALARM ||
                 i == BLEACH_ALARM) {
        // were are working alarm input
        if (debounceTime > ALARM_DEBOUNCE_TIME) {
          // debounce was long enough
          inputState[i] = inputReading;               // update input state
        }
      } else {
        // we are working on a normal input
        inputState[i] = inputReading;                 // update input state
      }
    }
    // prevent debounce time from getting too big
    if (debounceTime > 60 * 60 * 1000) {
      lastDebounceTime[i] = millis() - 60 * 1000;
    }
    // last reading is now current
    lastDebounceInput[i] = inputReading;
  }

  // inputs must have stabilized before they are read for the first time
  for(int i = 0; i < NUMBER_INPUTS; i++) {
    // check input to see if it is still default
    if (inputState[i] == -1) {
      inputStabilized = false;
      break;
    }
  }
  
  // if inputs are stabilized then check inputs as normal
  if (inputStabilized) {
    // handle Alarm State changing states
    if (lastAlarmState != curAlarmState) {
      switch (curAlarmState) {
        case AS_OFF:
          // just entered Alarm Off state
          Println("Entered AS_OFF");
          displayPumpState(true);
          digitalWrite(ALARM_LED, LOW);
          break;
        case AS_TANK_HIGH:
          // just entered Float Alarm state
          Println("Entered AS_TANK_HIGH");
          SepticLCD.setDisplayMessage("Tank High Alarm");
          SepticLCD.setBacklightColor(COLOR_ALARM);
          digitalWrite(ALARM_LED, HIGH);
          break;
        case AS_AIR_PUMP:
          // just entered Air Pump Alarm state
          Println("Entered AS_AIR_PUMP");
          SepticLCD.setDisplayMessage("Air Pump Alarm");
          SepticLCD.setBacklightColor(COLOR_ALARM);
          digitalWrite(ALARM_LED, HIGH);
          break;
        case AS_BLEACH_LEVEL:
          // just entered Bleach Alarm state
          Println("Entered AS_BLEACH_LEVEL");
          SepticLCD.setDisplayMessage("Bleach Alarm");
          SepticLCD.setBacklightColor(COLOR_ALARM);
          digitalWrite(ALARM_LED, HIGH);
          break;
      }
    
      // Alarm State changed so time to publish
      publishAlarmStatus = true;

      // last Alarm State is now current Alarm State
      lastAlarmState = curAlarmState;
    }

    // handle Alarm State Machine
    switch (curAlarmState) {
      case AS_RESET:
        // we are in reset state determine what state we should be in
        if (inputState[FLOAT_ALARM] == ALARM_ACTIVE) {
          // Float Alarm in progress
          nextAlarmState = AS_TANK_HIGH;              // switch to Float Alarm state
          lastASDeadbandTime = millis();              // start a new deadband time
        } else if (inputState[AIR_ALARM] == ALARM_ACTIVE) {
          // Air Alarm in progress
          nextAlarmState = AS_AIR_PUMP;               // switch to Air Alarm state
          lastASDeadbandTime = millis();              // start a new deadband time
        } else if (inputState[BLEACH_ALARM] == ALARM_ACTIVE) {
          // Bleach Alarm in progress
          nextAlarmState = AS_BLEACH_LEVEL;           // switch to Bleach Alarm state
          lastASDeadbandTime = millis();              // start a new deadband time
        } else {
          // No alarm in progress
          nextAlarmState = AS_OFF;                    // switch to Alarm Off state
          lastASDeadbandTime = millis();              // start a new deadband time
        }
        break;
      case AS_OFF:
        // Alarm Off state
        if (millis() - lastASDeadbandTime >= ALARM_DEADBAND_TIME) {
          // deadband time has elapsed so we can change states now
          if (inputState[FLOAT_ALARM] == ALARM_ACTIVE) {
            // Float Alarm just occurred
            nextAlarmState = AS_TANK_HIGH;            // switch to Tank High Alarm state
            lastASDeadbandTime = millis();            // start a new deadband time
          } else if (inputState[AIR_ALARM] == ALARM_ACTIVE) {
            // Air Alarm just occurred
            nextAlarmState = AS_AIR_PUMP;             // switch to Air Alarm state
            lastASDeadbandTime = millis();            // start a new deadband time
          } else if (inputState[BLEACH_ALARM] == ALARM_ACTIVE) {
            // Bleach Alarm just occurred
            nextAlarmState = AS_BLEACH_LEVEL;         // switch to Bleach Alarm state
            lastASDeadbandTime = millis();            // start a new deadband time
          }
        }
        break;
      case AS_TANK_HIGH:
        // Float Alarm state
        if (millis() - lastASDeadbandTime >= ALARM_DEADBAND_TIME) {
          // deadband time has elapsed so we can change states now
          if (inputState[AIR_ALARM] == ALARM_ACTIVE) {
            // Air Alarm in progress
            nextAlarmState = AS_AIR_PUMP;             // switch to Air Alarm state
            lastASDeadbandTime = millis();            // start a new deadband time
          } else if (inputState[BLEACH_ALARM] == ALARM_ACTIVE) {
            // Bleach Alarm in progress
            nextAlarmState = AS_BLEACH_LEVEL;         // switch to Bleach Alarm state
            lastASDeadbandTime = millis();            // start a new deadband time
          } else if (inputState[FLOAT_ALARM] == ALARM_INACTIVE) {
            // Float Alarm went inactive
            nextAlarmState = AS_OFF;                  // switch to Alarm Off state
            lastASDeadbandTime = millis();            // start a new deadband time
          }
        }
        break;
      case AS_AIR_PUMP:
        // Float Alarm state
        if (millis() - lastASDeadbandTime >= ALARM_DEADBAND_TIME) {
          // deadband time has elapsed so we can change states now
          if (inputState[FLOAT_ALARM] == ALARM_ACTIVE) {
            // Float Alarm just occurred
            nextAlarmState = AS_TANK_HIGH;            // switch to Float Alarm state
            lastASDeadbandTime = millis();            // start a new deadband time
          } else if (inputState[BLEACH_ALARM] == ALARM_ACTIVE) {
            // Bleach Alarm just occurred
            nextAlarmState = AS_BLEACH_LEVEL;         // switch to Bleach Alarm state
            lastASDeadbandTime = millis();            // start a new deadband time
          } else if (inputState[AIR_ALARM] == ALARM_INACTIVE) {
            // Air Alarm went inactive
            nextAlarmState = AS_OFF;                  // switch to Alarm Off state
            lastASDeadbandTime = millis();            // start a new deadband time
          }
        }
        break;
      case AS_BLEACH_LEVEL:
        // Float Alarm state
        if (millis() - lastASDeadbandTime >= ALARM_DEADBAND_TIME) {
          // deadband time has elapsed so we can change states now
          if (inputState[FLOAT_ALARM] == ALARM_ACTIVE) {
            // Float Alarm just occurred
            nextAlarmState = AS_TANK_HIGH;            // switch to Float Alarm state
            lastASDeadbandTime = millis();            // start a new deadband time
          } else if (inputState[AIR_ALARM] == ALARM_ACTIVE) {
            // Air Alarm just occurred
            nextAlarmState = AS_AIR_PUMP;             // switch to Air Alarm state
            lastASDeadbandTime = millis();            // start a new deadband time
          } else if (inputState[BLEACH_ALARM] == ALARM_INACTIVE) {
            // Bleach Alarm went inactive
            nextAlarmState = AS_OFF;                  // switch to Alarm Off state
            lastASDeadbandTime = millis();            // start a new deadband time
          }
        }
        break;
      default:
        break;
        // unknown state, force back to reset state
        nextAlarmState = AS_RESET;
        break;
    }

    // update current Alarm State with next Alarm State
    curAlarmState = nextAlarmState;
    
    // handle Pump State Machine changing states
    if (lastPumpState != curPumpState) {
      // update display based on Pump State
      displayPumpState(false);
    
      // Pump State changed so time to publish
      if (curPumpState != PS_OFF_A &&
          curPumpState != PS_MANUAL_ON_A) {
        publishPumpStatus = true;
      }
      
      // last Pump State is now current Pump State
      lastPumpState = curPumpState;
    }

    // handle Pump State Machine
    switch (curPumpState) {
      case PS_RESET:
        // we are in reset state and effluent pump relay is off
        if (inputState[FLOAT_ALARM] == ALARM_ACTIVE) {
          // the high level alarm occurred while pumping
          nextPumpState = PS_ALARM_ON;                // switch to Alarm Pump State
          lastPSDeadbandTime = millis();              // start a new deadband time
        } else if (inputState[EFFLUENT_PUMP_SENSE] == SENSE_ACTIVE) {
          // we are in override pump on state
          nextPumpState = PS_OVER_ON;                 // switch to Override Pump State
          lastPSDeadbandTime = millis();              // start a new deadband time
        } else {
          // we are in pump off state
          nextPumpState = PS_OFF;                     // switch to Pump Off State
          lastPSDeadbandTime = millis();              // start a new deadband time
        }
        break;
     case PS_OFF:
       // Effluent Pump is in Pump Off state, waiting for Pump Toggle Switch to release
        digitalWrite(EFFLUENT_PUMP_RELAY, PUMP_RELAY_INACTIVE); // force effluent pump off
        if (millis() - lastPSDeadbandTime >= PUMP_DEADBAND_TIME) {
          if (inputState[FLOAT_ALARM] == ALARM_ACTIVE) {
            // the high level alarm occurred while pumping
            nextPumpState = PS_ALARM_ON;              // switch to Alarm Pump State
            lastPSDeadbandTime = millis();            // start a new deadband time
          } else if (inputState[EFFLUENT_PUMP_SENSE] == SENSE_ACTIVE) {
            // override pump on just occurred
            nextPumpState = PS_OVER_ON;               // switch to Override Pump State
            lastPSDeadbandTime = millis();            // start a new deadband time
          } else if (inputState[PUMP_TOGGLE_SWITCH] == PUMP_TOGGLE_INACTIVE) {
            // the pump toggle switch was released
            nextPumpState = PS_OFF_A;                 // switch to Pump Off A State
          }
        }
        break;
      case PS_OFF_A:
        // Effluent Pump is in Pump Off state
        digitalWrite(EFFLUENT_PUMP_RELAY, PUMP_RELAY_INACTIVE); // force effluent pump off
        // get how long it has been since effluent pump was turned on
        lastPumpOnDiff = millis() - lastPumpOnTime;
        // deadband time has elapsed so we can change states now
        if (inputState[FLOAT_ALARM] == ALARM_ACTIVE) {
          // the high level alarm occurred while pumping
          nextPumpState = PS_ALARM_ON;                // switch to Alarm Pump State
          lastPSDeadbandTime = millis();              // start a new deadband time
        } else if (inputState[PUMP_TOGGLE_SWITCH] == PUMP_TOGGLE_ACTIVE) {
          // manual turn on of pump
          nextPumpState = PS_MANUAL_ON;               // switch to Manual Pump State
          lastPSDeadbandTime = millis();              // start a new deadband time
          startManualOnTime = lastPSDeadbandTime;     // Pump Manual On start time
        } else if (inputState[EFFLUENT_PUMP_SENSE] == SENSE_ACTIVE) {
          // override pump on just occurred
          nextPumpState = PS_OVER_ON;                 // switch to Override Pump State
          lastPSDeadbandTime = millis();              // start a new deadband time
        } else if ((lastPumpOnDiff > 3 * 60 * 60 * 1000 &&
                    curHour > 1 && curHour < 4) || 
                    lastPumpOnDiff > 24 * 60 * 60 * 1000) {
          // last time the pump was on was more than 3 hours ago and
          // time is between 1:00AM and 4:00AM or
          // last time the pump was on was more than 24 hours ago
          nextPumpState = PS_AUTO_ON;                 // switch to Automatic Pump On State
          lastPSDeadbandTime = millis();              // start a new deadband time
        }
        break;
      case PS_MANUAL_ON:
        // Effluent Pump is in Manual Pump state
        digitalWrite(EFFLUENT_PUMP_RELAY, PUMP_RELAY_ACTIVE); // force effluent pump on
        lastPumpOnTime = millis();                    // pump is still on
        if (millis() - lastPSDeadbandTime >= PUMP_DEADBAND_TIME) {
          if (inputState[FLOAT_ALARM] == ALARM_ACTIVE) {
            // the high level alarm occurred while pumping
            nextPumpState = PS_ALARM_ON;              // switch to Alarm Pump State
            lastPSDeadbandTime = millis();            // start a new deadband time
          } else if (inputState[PUMP_TOGGLE_SWITCH] == PUMP_TOGGLE_INACTIVE) {
            // the pump toggle switch was released
            nextPumpState = PS_MANUAL_ON_A;           // switch to Pump Manual ON A
          }
        }
        break;
      case PS_MANUAL_ON_A:
        // Effluent Pump is in Manual Pump state and Pump Toggle Switch was released
        digitalWrite(EFFLUENT_PUMP_RELAY, PUMP_RELAY_ACTIVE); // force effluent pump on
        lastPumpOnTime = millis();                    // pump is still on
        // deadband time has elapsed so we can change states now
        if (inputState[FLOAT_ALARM] == ALARM_ACTIVE) {
          // the high level alarm occurred while pumping
          nextPumpState = PS_ALARM_ON;                // switch to Alarm Pump State
          lastPSDeadbandTime = millis();              // start a new deadband time
        } else if (inputState[EFFLUENT_PUMP_SENSE] == SENSE_INACTIVE) {
          // the pump shut itself off due to low level float
          nextPumpState = PS_OFF;                     // switch to Pump Off State
          lastPSDeadbandTime = millis();              // start a new deadband time
        } else if (inputState[PUMP_TOGGLE_SWITCH] == PUMP_TOGGLE_ACTIVE) {
          // Pump Toggle Switch we pressed again, time to switch off
          nextPumpState = PS_OFF;                     // switch to Pump Off State
          lastPSDeadbandTime = millis();              // start a new deadband time
        } else if (millis() - startManualOnTime > 30 * 60 * 1000) {
          // Manual On time too long
          nextPumpState = PS_OFF;                     // switch to Pump Off State
          lastPSDeadbandTime = millis();              // start a new deadband time
        }
        break;
      case PS_AUTO_ON:
        // Effluent Pump is in Automatic Pump state
        digitalWrite(EFFLUENT_PUMP_RELAY, PUMP_RELAY_ACTIVE); // force effluent pump on
        lastPumpOnTime = millis();                    // pump is still on
        if (millis() - lastPSDeadbandTime >= PUMP_DEADBAND_TIME) {
          // deadband time has elapsed so we can change states now
          if (inputState[FLOAT_ALARM] == ALARM_ACTIVE) {
            // the high level alarm occurred while pumping
            nextPumpState = PS_ALARM_ON;              // switch to Alarm Pump State
            lastPSDeadbandTime = millis();            // start a new deadband time
          } else if (inputState[EFFLUENT_PUMP_SENSE] == SENSE_INACTIVE) {
            // the pump shut itself off due to low level float
            nextPumpState = PS_OFF;                   // switch to Pump Off State
            lastPSDeadbandTime = millis();            // start a new deadband time
          }
        }
        break;
      case PS_OVER_ON:
        // Effluent Pump is in Override Pump state
        digitalWrite(EFFLUENT_PUMP_RELAY, PUMP_RELAY_ACTIVE); // force effluent pump on
        lastPumpOnTime = millis();                    // pump is still on
        if (millis() - lastPSDeadbandTime >= PUMP_DEADBAND_TIME) {
          // deadband time has elapsed so we can change states now
          if (inputState[FLOAT_ALARM] == ALARM_ACTIVE) {
            // the tank high level alarm occurred while pumping
            nextPumpState = PS_ALARM_ON;              // switch to Alarm Pump State
            lastPSDeadbandTime = millis();            // start a new deadband time
          } else if (inputState[EFFLUENT_PUMP_SENSE] == SENSE_INACTIVE) {
            // the pump shut itself off due to low level float
            nextPumpState = PS_OFF;                   // switch to Pump Off State
            lastPSDeadbandTime = millis();            // start a new deadband time
          }
        }
        break;
      case PS_ALARM_ON:
        // Effluent Pump is in Alarm Pump state
        digitalWrite(EFFLUENT_PUMP_RELAY, PUMP_RELAY_ACTIVE); // force effluent pump on
        lastPumpOnTime = millis();                    // pump is still on
        if (millis() - lastPSDeadbandTime >= PUMP_DEADBAND_TIME) {
          // deadband time has elapsed so we can change states now
          if (inputState[FLOAT_ALARM] == ALARM_INACTIVE) {
            // the pump shut itself off due to low level float
            nextPumpState = PS_OFF;
            lastPSDeadbandTime = millis();            // start a new deadband time
          } else if (inputState[EFFLUENT_PUMP_SENSE] == SENSE_INACTIVE) {
            // the pump shut itself off due to low level float
            nextPumpState = PS_OFF;
            lastPSDeadbandTime = millis();            // start a new deadband time
          }
        }
        break;
      default:
        // unknown state, force back to reset state
        nextPumpState = PS_RESET;
        digitalWrite(EFFLUENT_PUMP_RELAY, PUMP_RELAY_INACTIVE); // force effluent pump off
        Print("Unknown Pump State: "); Println(curPumpState);
        break;
    }
    
    // update current Pump State with next Pump State
    curPumpState = nextPumpState;
  }

  // update LCD
  SepticLCD.loop();
  
  // Check connections
  if (!connect()) {
    // we are not currently connected, ignore rest of loop to prevent MQTT publishing
    return;
  }

  if (publishAlarmStatus) {
    // update display based on alarm state
    switch (curAlarmState) {
      case AS_OFF:
        if (mqtt.publish(HASS_ALARM_STATE_TOPIC, "Off", true, 1)) {
          Println("Published '" HASS_ALARM_STATE_TOPIC "' MQTT topic, 'Off' topic value");
          publishAlarmStatus = false;                 // won't need to publish again
        } else {
          Println("Failed to publish '" HASS_ALARM_STATE_TOPIC "' MQTT topic, 'Off' topic value");
        }
        break;
      case AS_TANK_HIGH:
        if (mqtt.publish(HASS_ALARM_STATE_TOPIC, "Tank High Alarm", true, 1)) {
          Println("Published '" HASS_ALARM_STATE_TOPIC "' MQTT topic, 'Tank High Alarm' topic value");
          publishAlarmStatus = false;                 // won't need to publish again
        } else {
          Println("Failed to publish '" HASS_ALARM_STATE_TOPIC "' MQTT topic, 'Tank High Alarm' topic value");
        }
        break;
      case AS_AIR_PUMP:
        if (mqtt.publish(HASS_ALARM_STATE_TOPIC, "Air Pump Alarm", true, 1)) {
          Println("Published '" HASS_ALARM_STATE_TOPIC "' MQTT topic, 'Air Pump Alarm' topic value");
          publishAlarmStatus = false;                 // won't need to publish again
        } else {
          Println("Failed to publish '" HASS_ALARM_STATE_TOPIC "' MQTT topic, 'Air Pump Alarm' topic value");
        }
        break;
      case AS_BLEACH_LEVEL:
        if (mqtt.publish(HASS_ALARM_STATE_TOPIC, "Bleach Alarm", true, 1)) {
          Println("Published '" HASS_ALARM_STATE_TOPIC "' MQTT topic, 'Bleach Alarm' topic value");
          publishAlarmStatus = false;                 // won't need to publish again
        } else {
          Println("Failed to publish '" HASS_ALARM_STATE_TOPIC "' MQTT topic, 'Bleach Alarm' topic value");
        }
        break;
      default:
        break;
    }
  }
  
  // if inputs are stabilized handle pump state publish
  if (inputStabilized && publishPumpStatus) {
    // publish with no force update
    switch (curPumpState) {
      case PS_OFF:
        if (mqtt.publish(HASS_PUMP_STATE_TOPIC, "Off", false, 1)) {
          Println("Published '" HASS_PUMP_STATE_TOPIC "' MQTT topic, 'Off' topic value");
          publishPumpStatus = false;
        } else {
          Println("Failed to publish '" HASS_PUMP_STATE_TOPIC "' MQTT topic, 'Off' topic value");
        }
        break;
      case PS_MANUAL_ON:
        if (mqtt.publish(HASS_PUMP_STATE_TOPIC, "Manual On", false, 1)) {
          Println("Published '" HASS_PUMP_STATE_TOPIC "' MQTT topic, 'Manual On' topic value");
          publishPumpStatus = false;
        } else {
          Println("Failed to publish '" HASS_PUMP_STATE_TOPIC "' MQTT topic, 'Manual On' topic value");
        }
        break;
      case PS_AUTO_ON:
        if (mqtt.publish(HASS_PUMP_STATE_TOPIC, "Auto On", false, 1)) {
          Println("Published '" HASS_PUMP_STATE_TOPIC "' MQTT topic, 'Auto On' topic value");
          publishPumpStatus = false;
        } else {
          Println("Failed to publish '" HASS_PUMP_STATE_TOPIC "' MQTT topic, 'Auto On' topic value");
        }
        break;
      case PS_OVER_ON:
        if (mqtt.publish(HASS_PUMP_STATE_TOPIC, "Override On", false, 1)) {
          Println("Published '" HASS_PUMP_STATE_TOPIC "' MQTT topic, 'Override On' topic value");
          publishPumpStatus = false;
        } else {
          Println("Failed to publish '" HASS_PUMP_STATE_TOPIC "' MQTT topic, 'Override On' topic value");
        }
        break;
      case PS_ALARM_ON:
        if (mqtt.publish(HASS_PUMP_STATE_TOPIC, "Alarm On", false, 1)) {
          Println("Published '" HASS_PUMP_STATE_TOPIC "' MQTT topic, 'Alarm On' topic value");
          publishPumpStatus = false;
        } else {
          Println("Failed to publish '" HASS_PUMP_STATE_TOPIC "' MQTT topic, 'Alarm On' topic value");
        }
        break;
    }
  }

  // is it time to temperature sensor values?
  temperature = SepticLCD.getTemperatureC();          // retrieve the temperature
  if (temperature > -100.0) {
    // there is a valid temperature
    if ((millis() - lastTempPublish) > TEMP_PUBLISH_RATE) {
      // it has been long enough since last publish
      dtostrf(temperature, 1, 1, tempStr);            // convert temperature to string
      // publish the most recent temperture
      if (mqtt.publish(HASS_TEMP_STATE_TOPIC, tempStr, false, 1)) {
/*        Print("Published '" HASS_TEMP_STATE_TOPIC "' MQTT topic, '");
        Print(tempStr);
        Println("' topic value");
*/      } else {
        Print("Failed to publish '" HASS_TEMP_STATE_TOPIC "' MQTT topic, '");
        Print(tempStr);
        Println("' topic value");
      }
      lastTempPublish += TEMP_PUBLISH_RATE;           // prepare for next publish time
    }
  }
  // is it time to RSSI sensor values?
  if ((millis() - lastRSSIPublish) > TEMP_PUBLISH_RATE) {
    itoa(WiFi.RSSI(), tempStr, 10);
    // publish the RSSI too
    if (mqtt.publish(HASS_RSSI_STATE_TOPIC, tempStr, false, 1)) {
/*      Print("Published '" HASS_RSSI_STATE_TOPIC "' MQTT topic, '");
      Print(tempStr);
      Println("' topic value");
*/    } else {
      Print("Failed to publish '" HASS_RSSI_STATE_TOPIC "' MQTT topic, '");
      Print(tempStr);
      Println("' topic value");
    }
    lastRSSIPublish += TEMP_PUBLISH_RATE;             // prepare for next publish time
  }
}

// Verify/Make connections
bool connect() {
  byte mac[6];
  IPAddress ip;

  if (WiFi.status() != WL_CONNECTED) {
    // Wifi is disconnected
    wifiDisconnectOccurred = true;                    // keep track of the fact we were disconnected
    return false;                                     // return with not connected
  }
  if (wifiDisconnectOccurred && (WiFi.status() == WL_CONNECTED)) {
    // WiFi is connected and previously we were disconnected
    Println("Connected to SSID: " SECRET_SSID);
    
    // we have detected that we just connected
    wifiDisconnectOccurred = false;                   // so we won't print network stats until next reconnect
    wifiConnectOccurred = true;                       // so MQTT publishing will know that Wifi just connected

    // Enable WiFi Low Power Mode
    //WiFi.lowPowerMode();
    //Println("  Low Power Mode enabled");
    
    #ifdef ENABLE_SERIAL
    // Display MAC Address
    WiFi.macAddress(mac);
    Print("  MAC Address: ");
    for (int i = 5; i != 0; i--) {
      if (mac[i] < 16) Print("0");
      Print(mac[i], HEX);
      Print(":");
    }
    if (mac[0] < 16) Print("0");
    Println(mac[0], HEX);
    
    // Display IP Address
    ip = WiFi.localIP();
    Print("  IP Address: ");
    Println(ip);
    #endif

    #ifdef ENABLE_OTA_UPDATES
    // start the WiFi OTA library with internal based storage
    WiFiOTA.begin(BOARD_NAME, OTA_PASSWORD, InternalStorage);
    #ifdef ENABLE_SERIAL
    Println("WiFi OTA updates enabled");
    #endif
    #endif
  } 
  
  if (!mqtt.connected()) {
    // we are not currently connected to MQTT Server
    Print("Connecting to MQTT Server:" MQTT_SERVER "...");
    if (mqtt.connect(BOARD_NAME, MQTT_USERNAME, MQTT_PASSWORD)) {
      // successfully connected to MQTT server
      Println("Success");
    } else {
      // failed to connect to MQTT server
      Println("Failed");
      printWiFiStatus(true);
      return false;
    }
    
    if (!mqtt.publish(HASS_TEMP_CONFIG_TOPIC, HASS_TEMP_CONFIG, true, 1)) {
      Println("  Failed to publish '" HASS_TEMP_CONFIG_TOPIC "' MQTT topic");
    } else {
      Println("  Published '" HASS_TEMP_CONFIG_TOPIC "' MQTT topic, '" HASS_TEMP_CONFIG "' topic value");
    }
    
    // Publish Home Assistant RSSI config topic
    if (!mqtt.publish(HASS_RSSI_CONFIG_TOPIC, HASS_RSSI_CONFIG, true, 1)) {
      Println("  Failed to publish '" HASS_RSSI_CONFIG_TOPIC "' MQTT topic");
    } else {
      Println("  Published '" HASS_RSSI_CONFIG_TOPIC "' MQTT topic, '" HASS_RSSI_CONFIG "' topic value");
    }
    
    // Publish Home Assistant pump config topic
    if (!mqtt.publish(HASS_PUMP_CONFIG_TOPIC, HASS_PUMP_CONFIG, true, 1)) {
      Println("  Failed to publish '" HASS_PUMP_CONFIG_TOPIC "' MQTT topic");
    } else {
      Println("  Published '" HASS_PUMP_CONFIG_TOPIC "' MQTT topic, '" HASS_PUMP_CONFIG "' topic value");
    }
    
    // Publish Home Assistant alarm config topic
    if (!mqtt.publish(HASS_ALARM_CONFIG_TOPIC, HASS_ALARM_CONFIG, true, 1)) {
      Println("  Failed to publish '" HASS_ALARM_CONFIG_TOPIC "' MQTT topic");
    } else {
      Println("  Published '" HASS_ALARM_CONFIG_TOPIC "' MQTT topic, '" HASS_ALARM_CONFIG "' topic value");
    }
    
    // Publish Home Assistant pump status config topic
    if (!mqtt.publish(HASS_STATUS_CONFIG_TOPIC, HASS_STATUS_CONFIG, true, 1)) {
      Println("  Failed to publish '" HASS_STATUS_CONFIG_TOPIC "' MQTT topic");
    } else {
      Println("  Published '" HASS_STATUS_CONFIG_TOPIC "' MQTT topic, '" HASS_STATUS_CONFIG "' topic value");
    }
    
    if (resetOccurred) {                              // Hardware reset occurred
      // reset just recently occurred
      if (!mqtt.publish(HASS_STATUS_STATE_TOPIC, "Reset Hardware", true, 1)) {
        Println("  Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Reset Hardware' topic value");
      } else {
        Println("  Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Reset Hardware' topic value");
      }
    } else if (wifiConnectOccurred) {                 // WiFi Connected
      // Wifi just connected
      if (!mqtt.publish(HASS_STATUS_STATE_TOPIC, "Reset WiFi Connect", true, 1)) {
        Println("  Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Reset WiFi Connect' topic value");
      } else {
        Println("  Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Reset WiFi Connect' topic value");
      }
    } else if (!mqtt.publish(HASS_STATUS_STATE_TOPIC, "Reset MQTT Connect", true, 1)) {
      // since no reset or WiFi connect occurred then it was a MQTT Connect
      Println("  Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'ResetMQTT Connect' topic value");
    } else {
      Println("  Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Reset MQTT Connect' topic value");
    }

    // wait a bit before updating HASS_STATUS_STATE_TOPIC again
    delay(100);
    
    // clear the connect reason flags
    resetOccurred = false;
    wifiConnectOccurred = false;
    if (!mqtt.publish(HASS_STATUS_STATE_TOPIC, "Idle", true, 1)) {
      // since no reset or WiFi connect occurred then it was a MQTT Connect
      Println("  Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Idle' topic value");
    } else {
      Println("  Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Idle' topic value");
    }
    Println("WiFi Septic Controller is ready!\n");
  }

  // if we got here then we were successfull
  return true;
}

// MQTT subscribed message received
void messageReceived(MQTTClient *client, char topic[], char payload[], int payload_length) {
/*  if (strcmp(topic, HASS_GATE_COMMAND_TOPIC) == 0) {
    // gate command has been received
    if (outputDeadband == LOW) {
      // no other output is in progress
      if (strcmp(payload, "OPEN") == 0) {             // time to OPEN the gate
        digitalWrite(output_pins[OPEN_BUTTON], HIGH); // make OPEN Button active
        outputState[OPEN_BUTTON] = HIGH;              // indicate to loop() that OPEN Button output is active
        lastOutputTime[OPEN_BUTTON] = millis();       // start the timer used to cancel output
        outputDeadband = HIGH;                        // contact closure in progress no other contact closures can occur
        lastOutputDeadbandTime = lastOutputTime[OPEN_BUTTON]; // start a timer for deadband timeout
        digitalWrite(BOARD_LED, HIGH);                // visual indication that an output is active
        Println("Accepted 'OPEN' MQTT Command");
      } else if (strcmp(payload, "CLOSE") == 0) {     // time to CLOSE the gate
        digitalWrite(output_pins[CLOSE_BUTTON], HIGH);// make CLOSE Button active
        outputState[CLOSE_BUTTON] = HIGH;             // indicate to loop() that CLOSE Button output is active
        lastOutputTime[CLOSE_BUTTON] = millis();      // start the timer used to cancel output
        outputDeadband = HIGH;                        // contact closure in progress no other contact closures can occur
        lastOutputDeadbandTime = lastOutputTime[CLOSE_BUTTON]; // start a timer for deadband timeout
        digitalWrite(BOARD_LED, HIGH);                // visual indication that an output is active
        Println("Accepted 'CLOSE' MQTT Command");
     } else if (strcmp(payload, "STOP") == 0) {      // time to TOGGLE the gate
        digitalWrite(output_pins[TOGGLE_BUTTON], HIGH); // make TOGGLE Button active
        outputState[TOGGLE_BUTTON] = HIGH;            // indicate to loop() that TOGGLE Button output is active
        lastOutputTime[TOGGLE_BUTTON] = millis();     // start the timer used to cancel output
        outputDeadband = HIGH;                        // contact closure in progress no other contact closures can occur
        lastOutputDeadbandTime = lastOutputTime[TOGGLE_BUTTON]; // start a timer for deadband timeout
        digitalWrite(BOARD_LED, HIGH);                // visual indication that an output is active
        Println("Accepted 'STOP' MQTT Command");
      }
    } else {
      // another output is in progress so we ignore this command
      if (!mqtt.publish(HASS_STATUS_STATE_TOPIC, "Ignored MQTT Command", true, 1)) {
        Println("Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic");
      } else {
        Println("  Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Ignored MQTT Command' topic value");
      }
      Print("Ignored '");
      Print(payload);
      Println("' MQTT Command");
    }
  }*/
}

// Reset the watchdog timer
inline void watchdogReset(void) {
  #ifdef ENABLE_WATCHDOG
  if (!WDT->STATUS.bit.SYNCBUSY)                // Check if the WDT registers are synchronized
    REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;        // Clear the watchdog timer
  #endif
}

// Prints current WiFi Status and RSSI
void printWiFiStatus(bool withLF) {
  #ifdef ENABLE_SERIAL
  Print("Status: ");
  switch (WiFi.status()) {
    case WL_NO_SHIELD: Print("No Shield"); break;
    case WL_IDLE_STATUS: Print("Idle Status"); break;
    case WL_NO_SSID_AVAIL: Print("No SSID Available"); break;
    case WL_SCAN_COMPLETED: Print("Scan Completed"); break;
    case WL_CONNECTED: Print("Connected"); break;
    case WL_CONNECT_FAILED: Print("Connect Failed"); break;
    case WL_CONNECTION_LOST: Print("Connection Lost"); break;
    case WL_DISCONNECTED: Print("Disconnected"); break;
  }
  Print(", RSSI: ");
  Print(WiFi.RSSI());
  Print("dBm");
  if (withLF)
    Println(".");
  else
    Print(", ");
  #endif
}
