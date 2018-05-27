/*
  Septic WiFi Controller 
  
  This code acts as an Aerobic Septic Controller using a custom 
  SAMD/ATWINC1500C board. The board is compatible with an Arduino MKR1000.
  The sketch is primarily two state machines: one for effluent pump control
  and one for alarm state. The pump state machine can detect when the 
  override float has applied power to the pump thus providing some indication 
  of over usage or rain seepage into the tanks. The sketch is a discoverable 
  set of sensors for Home Assistant, an open-source home automation platform 
  running on Python.
   
  MQTT, a machine-to-machine (M2M)/"Internet of Things" connectivity 
  protocol, is the basis of communication with Home Assistant.
  
  The following libraries must be installed using Library Manager:
  
    WiFi101 by Arduino
    WiFiOTA by Arduino
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
enum PumpStateEnum {PS_RESET, PS_TANK_EMPTY, PS_OFF, PS_OFF_A, PS_MANUAL_ON, PS_MANUAL_ON_A, PS_AUTO_ON, PS_OVER_ON}; 
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
int8_t         inputState[NUMBER_INPUTS] = {-1, -1, -1, -1, -1, -1};
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
         curPumpState == PS_OVER_ON;
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
    case PS_TANK_EMPTY:
      Logln("Entered PS_TANK_EMPTY State.");
      if (!isAlarmOn()) {
        SepticLCD.setDisplayMessage("Tank Empty");
      }
      break;
    case PS_OFF:
      Logln("Entered PS_OFF State.");
      if (!isAlarmOn()) {
        SepticLCD.setDisplayMessage("Idle");
      }
      break;
    case PS_OFF_A:
      Logln("Entered PS_OFF_A State.");
      if (!isAlarmOn() && force) {
        SepticLCD.setDisplayMessage("Idle");
      }
      break;
    case PS_MANUAL_ON:
      Logln("Entered PS_MANUAL_ON State.");
      if (!isAlarmOn()) {
        SepticLCD.setDisplayMessage("Manual Pump On");
      }
      break;
    case PS_MANUAL_ON_A:
      Logln("Entered PS_MANUAL_ON_A State.");
      if (!isAlarmOn() && force) {
        SepticLCD.setDisplayMessage("Manual Pump On");
      }
      break;
    case PS_AUTO_ON:
      Logln("Entered PS_AUTO_ON State.");
      if (!isAlarmOn()) {
        SepticLCD.setDisplayMessage("Auto Pump On");
      }
      break;
    case PS_OVER_ON:
      Logln("Entered PS_OVER_ON State.");
      if (!isAlarmOn()) {
        SepticLCD.setDisplayMessage("Override Pump On");
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
  static uint32_t nextTempPublish = 0;                // used to publish the temp at a defined rate
  static uint32_t nextRSSIPublish = 0;                // used to publish the RSSI at a defined rate
  static uint32_t lastDebounceTime[NUMBER_INPUTS] = {0, 0, 0, 0, 0, 0};
  static int8_t   lastDebounceInput[NUMBER_INPUTS] = {-1, -1, -1, -1, -1, -1};
  static uint32_t lastPSDeadbandTime;                 // add a deadband between pump state changes
  static uint32_t lastASDeadbandTime;                 // add a deadband between alarm state changes
  static uint8_t  lastEffluentPumpSense = -1;         // last sensed effuent pump state
  static uint32_t lastPumpOnStartTime = 0;                 // last pump on time was now.
  static uint32_t lastPumpOffStartTime = 0 - 4*60*60*1000; // last pump off start time was 4 hours ago.
  static bool     publishPumpState = true;            // when true pump status should be published to MQTT server
  static bool     publishAlarmState = true;           // when true alarm status should be published to MQTT server
  static bool     publishSepticStatus = true;         // when true septic status should be published to MQTT server
  PumpStateEnum   nextPumpState = curPumpState;       // next state for pump state machine, default is no change
  AlarmStateEnum  nextAlarmState = curAlarmState;     // next state for alarm state machine, default is no change
  uint32_t        timeDiff;                           // how long has it been since pump was last on
  float           temperature;                        // temperature reading
  char            tempStr[16];                        // temporary string
  uint8_t         inputReading;                       // last input read
  bool            inputStabilized = true;             // when true inputs are stabilized and are ready to be read
  
  // Reset the watchdog with every loop to make sure the sketch keeps running.
  watchdogReset();
  
  // keep track of time appropriately
  WiFiRTC.loop();

  //Print('.');
  
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
  
  // handle LCD background color
  if (curAlarmState != AS_OFF) {
    // alarm is on
    SepticLCD.setBacklightColor(COLOR_ALARM);
  } else if (inputState[EFFLUENT_PUMP_SENSE] == SENSE_ACTIVE) {
    // pump is on
    SepticLCD.setBacklightColor(COLOR_PUMPING);
  } else {
    // idle
    SepticLCD.setBacklightColor(COLOR_IDLE);
  }

  // if inputs are stabilized then check inputs as normal
  if (inputStabilized) {
    // handle Alarm State changing states
    if (lastAlarmState != curAlarmState) {
      switch (curAlarmState) {
        case AS_OFF:
          // just entered Alarm Off state
          Logln("Entered AS_OFF State.");
          displayPumpState(true);
          digitalWrite(ALARM_LED, LOW);
          break;
        case AS_TANK_HIGH:
          // just entered Float Alarm state
          Logln("Entered AS_TANK_HIGH State.");
          SepticLCD.setDisplayMessage("Tank High Alarm");
          SepticLCD.setBacklightColor(COLOR_ALARM);
          digitalWrite(ALARM_LED, HIGH);
          break;
        case AS_AIR_PUMP:
          // just entered Air Pump Alarm state
          Logln("Entered AS_AIR_PUMP State.");
          SepticLCD.setDisplayMessage("Air Pump Alarm");
          SepticLCD.setBacklightColor(COLOR_ALARM);
          digitalWrite(ALARM_LED, HIGH);
          break;
        case AS_BLEACH_LEVEL:
          // just entered Bleach Alarm state
          Logln("Entered AS_BLEACH_LEVEL State.");
          SepticLCD.setDisplayMessage("Bleach Alarm");
          SepticLCD.setBacklightColor(COLOR_ALARM);
          digitalWrite(ALARM_LED, HIGH);
          break;
      }
    
      // Alarm State changed so time to publish
      publishSepticStatus = true;
      publishAlarmState = true;

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
    
      // Pump State changed so time to publish septic status
      if (curPumpState != PS_OFF_A &&
          curPumpState != PS_MANUAL_ON_A) {
        publishSepticStatus = true;
      }
      
      // last Pump State is now current Pump State
      lastPumpState = curPumpState;
    }

    // handle Pump State Machine
    switch (curPumpState) {
      case PS_RESET:
        // we are in reset state
        if (inputState[EFFLUENT_PUMP_SENSE] == SENSE_ACTIVE) {
          // override float/pump came on
          lastPSDeadbandTime = millis();              // start a new deadband time
          nextPumpState = PS_OVER_ON;                 // switch to Override Pump State
        } else {
          // we are in pump off state
          lastPSDeadbandTime = millis();              // start a new deadband time
          nextPumpState = PS_OFF;                     // switch to Pump Off State
        }
        break;
      case PS_TANK_EMPTY:
        // we are in the tank went empty while pumping
        digitalWrite(EFFLUENT_PUMP_RELAY, PUMP_RELAY_INACTIVE); // force effluent pump off
        if (millis() - lastPSDeadbandTime >= 5 * 1000) {
          // delay time has elapsed
          // do not reset deadband time so Pump Off state won't wait for a new deadband time
          lastPumpOffStartTime = millis();            // new beginning of off time
          nextPumpState = PS_OFF;                     // switch to Pump Off State
        } else if (millis() - lastPSDeadbandTime >= PUMP_DEADBAND_TIME) {
          // normal deadband time has elapsed, look for pump toggle
          if (inputState[PUMP_TOGGLE_SWITCH] == PUMP_TOGGLE_ACTIVE) {
            // the pump toggle switch was pressed
            lastPSDeadbandTime = millis();              // start a new deadband time
            lastPumpOnStartTime = millis();             // new beginning of pump on time
            nextTempPublish = millis();                 // force temperature publish now
            nextPumpState = PS_MANUAL_ON;               // switch to Manual On State
          }
        }
        break;
      case PS_OFF:
        // Effluent Pump is in Pump Off state, waiting for Pump Toggle Switch to release
        digitalWrite(EFFLUENT_PUMP_RELAY, PUMP_RELAY_INACTIVE); // force effluent pump off
        if (millis() - lastPSDeadbandTime >= PUMP_DEADBAND_TIME) {
          if (inputState[EFFLUENT_PUMP_SENSE] == SENSE_ACTIVE) {
            // override float/pump came on
            lastPSDeadbandTime = millis();            // start a new deadband time
            lastPumpOnStartTime = millis();           // new beginning of pump on time
            nextTempPublish = millis();               // force temperature publish now
            nextPumpState = PS_OVER_ON;               // switch to Override Pump State
          } else if (inputState[PUMP_TOGGLE_SWITCH] == PUMP_TOGGLE_INACTIVE) {
            // the pump toggle switch was released
            nextPumpState = PS_OFF_A;                 // switch to Pump Off A State
          }
        }
        break;
      case PS_OFF_A:
        // Effluent Pump is in Pump Off state
        digitalWrite(EFFLUENT_PUMP_RELAY, PUMP_RELAY_INACTIVE); // force effluent pump off
        // get how long has effluent pump been off
        timeDiff = millis() - lastPumpOffStartTime;
        // deadband time has elapsed so we can change states now
        if (inputState[PUMP_TOGGLE_SWITCH] == PUMP_TOGGLE_ACTIVE) {
          // manual pump on because the pump toggle switch was pressed
          lastPSDeadbandTime = millis();              // start a new deadband time
          lastPumpOnStartTime = millis();             // new beginning of pump on time
          nextTempPublish = millis();                 // force temperature publish now
          nextPumpState = PS_MANUAL_ON;               // switch to Manual Pump State
        } else if (inputState[EFFLUENT_PUMP_SENSE] == SENSE_ACTIVE) {
          // override float/pump came on
          lastPSDeadbandTime = millis();              // start a new deadband time
          lastPumpOnStartTime = millis();             // new beginning of pump on time
          nextTempPublish = millis();                 // force temperature publish now
          nextPumpState = PS_OVER_ON;                 // switch to Override Pump State
        } else if ((WiFiRTC.getHour() >= 1 && WiFiRTC.getHour() < 4 &&
                    timeDiff > 4 * 60 * 60 * 1000) || 
                    timeDiff > 24 * 60 * 60 * 1000) {
          // time is between 1:00AM and 4:00AM and last time pump was on > 4 hours or
          // last time the pump was on > 24 hours
          lastPSDeadbandTime = millis();              // start a new deadband time
          lastPumpOnStartTime = millis();             // new beginning of pump on time
          nextTempPublish = millis();                 // force temperature publish now
          nextPumpState = PS_AUTO_ON;                 // switch to Automatic Pump On State
        }
        break;
      case PS_MANUAL_ON:
        // Effluent Pump is in Manual Pump state
        digitalWrite(EFFLUENT_PUMP_RELAY, PUMP_RELAY_ACTIVE); // force effluent pump on
        if (millis() - lastPSDeadbandTime >= PUMP_DEADBAND_TIME) {
          if (inputState[PUMP_TOGGLE_SWITCH] == PUMP_TOGGLE_INACTIVE) {
            // the pump toggle switch was released
            nextPumpState = PS_MANUAL_ON_A;           // switch to Pump Manual ON A
          }
        }
        break;
      case PS_MANUAL_ON_A:
        // Effluent Pump is in Manual Pump state and Pump Toggle Switch was released
        digitalWrite(EFFLUENT_PUMP_RELAY, PUMP_RELAY_ACTIVE); // force effluent pump on
        // get how long has effluent pump been on
        timeDiff = millis() - lastPumpOnStartTime;
        // deadband time has elapsed so we can change states now
        if (inputState[EFFLUENT_PUMP_SENSE] == SENSE_INACTIVE) {
          // the effluent pump shut itself off due to low level float
          lastPSDeadbandTime = millis();              // start a new deadband time
          lastPumpOffStartTime = millis();            // new beginning of pump off time
          nextPumpState = PS_TANK_EMPTY;              // switch to Tank Empty State
        } else if (inputState[PUMP_TOGGLE_SWITCH] == PUMP_TOGGLE_ACTIVE) {
          // Pump Toggle Switch was pressed again, time to switch off
          lastPSDeadbandTime = millis();              // start a new deadband time
          lastPumpOffStartTime = millis();            // new beginning of pump off time
          nextPumpState = PS_OFF;                     // switch to Pump Off State
        } else if (timeDiff > 30 * 60 * 1000) {
          // Manual On time long enough (30 minutes)
          lastPSDeadbandTime = millis();              // start a new deadband time
          lastPumpOffStartTime = millis();            // new beginning of pump off time
          nextPumpState = PS_OFF;                     // switch to Pump Off State
        }
        break;
      case PS_AUTO_ON:
        // Effluent Pump is in Automatic Pump state
        digitalWrite(EFFLUENT_PUMP_RELAY, PUMP_RELAY_ACTIVE); // force effluent pump on
        // get how long has effluent pump been on
        timeDiff = millis() - lastPumpOnStartTime;
        if (millis() - lastPSDeadbandTime >= PUMP_DEADBAND_TIME) {
          // deadband time has elapsed so we can change states now
          if (inputState[EFFLUENT_PUMP_SENSE] == SENSE_INACTIVE) {
            // the effluent pump shut itself off due to low level float
            lastPSDeadbandTime = millis();            // start a new deadband time
            lastPumpOffStartTime = millis();          // new beginning of off time
            nextPumpState = PS_TANK_EMPTY;            // switch to Tank Empty State
          } else if (timeDiff > 3 * 60 * 60 * 1000) {
            // Auto On time too long (more than 3 hours)
            lastPSDeadbandTime = millis();            // start a new deadband time
            lastPumpOffStartTime = millis();          // new beginning of off time
            nextPumpState = PS_OFF;                   // switch to Pump Off State
          }
        }
        break;
      case PS_OVER_ON:
        // Effluent Pump is in Override Pump state
        digitalWrite(EFFLUENT_PUMP_RELAY, PUMP_RELAY_ACTIVE); // force effluent pump on
        if (millis() - lastPSDeadbandTime >= PUMP_DEADBAND_TIME) {
          // deadband time has elapsed so we can change states now
          if (inputState[EFFLUENT_PUMP_SENSE] == SENSE_INACTIVE) {
            // the pump shut itself off due to low level float
            lastPSDeadbandTime = millis();            // start a new deadband time
            lastPumpOffStartTime = millis();          // new beginning of pump off time
            nextPumpState = PS_TANK_EMPTY;            // switch to Tank Empty State
          }
        }
        break;
      default:
        // unknown state, force back to reset state
        nextPumpState = PS_RESET;
        digitalWrite(EFFLUENT_PUMP_RELAY, PUMP_RELAY_INACTIVE); // force effluent pump off
        Log("Unknown Pump State: "); Println(curPumpState);
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
    publishSepticStatus = true;                         // force septic state publish when MQTT comes back on
    publishPumpState = true;                            // force pump state publish when MQTT comes back on
    publishAlarmState = true;                           // force alarm state publish when MQTT comes back on
    return;
  }

  // handle pump state changing
  if (lastEffluentPumpSense != inputState[EFFLUENT_PUMP_SENSE]) {
    publishPumpState = true;
    
    // current state is now last state
    lastEffluentPumpSense = inputState[EFFLUENT_PUMP_SENSE];
  }
  
  //  publish pump state
  if (inputStabilized && publishPumpState) {
    if (inputState[EFFLUENT_PUMP_SENSE] == SENSE_ACTIVE) {
      // pump is on
      if (mqtt.publish(HASS_PUMP_STATE_TOPIC, "ON", true, 1)) {
        Logln("Published '" HASS_PUMP_STATE_TOPIC "' MQTT topic, 'ON' topic value");
        publishPumpState = false;
      } else {
        Logln("Failed to publish '" HASS_PUMP_STATE_TOPIC "' MQTT topic, 'ON' topic value");
      }
    } else {
      // pump is off
      if (mqtt.publish(HASS_PUMP_STATE_TOPIC, "OFF", true, 1)) {
        Logln("Published '" HASS_PUMP_STATE_TOPIC "' MQTT topic, 'OFF' topic value");
        publishPumpState = false;
      } else {
        Logln("Failed to publish '" HASS_PUMP_STATE_TOPIC "' MQTT topic, 'OFF' topic value");
      }
    }
  }

  // publish septic status
  if (inputStabilized && publishSepticStatus) {
    if (curAlarmState != AS_OFF) {
      // alarm is on
      switch (curAlarmState) {
        case AS_TANK_HIGH:
          if (mqtt.publish(HASS_STATUS_STATE_TOPIC, "Tank High Alarm On", true, 1)) {
            Logln("Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Tank High Alarm On' topic value");
            publishSepticStatus = false;
          } else {
            Logln("Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Tank High Alarm On' topic value");
          }
          break;
        case AS_AIR_PUMP:
          if (mqtt.publish(HASS_STATUS_STATE_TOPIC, "Air Pump Alarm On", true, 1)) {
            Logln("Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Air Pump Alarm On' topic value");
            publishSepticStatus = false;
          } else {
            Logln("Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Air Pump Alarm On' topic value");
          }
          break;
        case AS_BLEACH_LEVEL:
          if (mqtt.publish(HASS_STATUS_STATE_TOPIC, "Bleach Alarm On", true, 1)) {
            Logln("Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Bleach Alarm On' topic value");
            publishSepticStatus = false;
          } else {
            Logln("Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Bleach Alarm On' topic value");
          }
          break;
        default:
          publishSepticStatus = false;
          break;
      }
    } else {
      // alarm is off so display pump status
      switch (curPumpState) {
        case PS_OFF:
        case PS_OFF_A:
          if (mqtt.publish(HASS_STATUS_STATE_TOPIC, "Idle", true, 1)) {
            Logln("Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Idle' topic value");
            publishSepticStatus = false;
          } else {  
            Logln("Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Idle' topic value");
          }
          break;
        case PS_TANK_EMPTY:
          if (mqtt.publish(HASS_STATUS_STATE_TOPIC, "Tank Empty", true, 1)) {
            Logln("Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Tank Empty' topic value");
            publishSepticStatus = false;
          } else {
            Logln("Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Tank Empty' topic value");
          }
          break;
        case PS_MANUAL_ON:
        case PS_MANUAL_ON_A:
          if (mqtt.publish(HASS_STATUS_STATE_TOPIC, "Manual Pump On", true, 1)) {
            Logln("Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Manual Pump On' topic value");
            publishSepticStatus = false;
          } else {
            Logln("Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Manual Pump On' topic value");
          }
          break;
        case PS_AUTO_ON:
          if (mqtt.publish(HASS_STATUS_STATE_TOPIC, "Auto Pump On", true, 1)) {
            Logln("Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Auto Pump On' topic value");
            publishSepticStatus = false;
          } else {
            Logln("Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Auto Pump On' topic value");
          }
          break;
        case PS_OVER_ON:
          if (mqtt.publish(HASS_STATUS_STATE_TOPIC, "Override Pump On", true, 1)) {
            Logln("Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Override Pump On' topic value");
            publishSepticStatus = false;
          } else {
            Logln("Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Override Pump On' topic value");
          }
          break;
        default:
          publishSepticStatus = false;
          break;
      }
    }
  }

  // publish alarm state
  if (inputStabilized && publishAlarmState) {
    // update display based on alarm state
    if (curAlarmState == AS_OFF) {
      // alarm is currently off
      if (mqtt.publish(HASS_ALARM_STATE_TOPIC, "OFF", true, 1)) {
        Logln("Published '" HASS_ALARM_STATE_TOPIC "' MQTT topic, 'OFF' topic value");
        publishAlarmState = false;
      } else {
        Logln("Failed to publish '" HASS_ALARM_STATE_TOPIC "' MQTT topic, 'OFF' topic value");
      }
    } else {
      // alarm is currently on
      if (mqtt.publish(HASS_ALARM_STATE_TOPIC, "ON", true, 1)) {
        Logln("Published '" HASS_ALARM_STATE_TOPIC "' MQTT topic, 'ON' topic value");
        publishAlarmState = false;
      } else {
        Logln("Failed to publish '" HASS_ALARM_STATE_TOPIC "' MQTT topic, 'ON' topic value");
      }
    }
  }

  // is it time to temperature sensor values?
  temperature = SepticLCD.getTemperatureC();          // retrieve the temperature
  if (temperature > -100.0) {
    uint32_t publishRate = TEMP_PUBLISH_RATE;
    if (inputState[EFFLUENT_PUMP_SENSE] == SENSE_ACTIVE) {
      publishRate = TEMP_PUMP_ON_PUBLISH_RATE;
    }
    // there is a valid temperature
    if (millis() >= nextTempPublish) {
      // it has been long enough since last temperature publish
      dtostrf(temperature, 1, 1, tempStr);            // convert temperature to string
      // publish the most recent temperture
      if (mqtt.publish(HASS_TEMP_STATE_TOPIC, tempStr, false, 1)) {
        /*Log("Published '" HASS_TEMP_STATE_TOPIC "' MQTT topic, '");
        Print(tempStr);
        Println("' topic value");*/
      } else {
        Log("Failed to publish '" HASS_TEMP_STATE_TOPIC "' MQTT topic, '");
        Print(tempStr);
        Println("' topic value");
      }
      
      nextTempPublish += publishRate;                 // prepare for next publish time
    }
  }
  // is it time to RSSI sensor values?
  if (millis() >= nextRSSIPublish) {
    itoa(WiFi.RSSI(), tempStr, 10);
    // publish the RSSI too
    if (mqtt.publish(HASS_RSSI_STATE_TOPIC, tempStr, false, 1)) {
      /*Log("Published '" HASS_RSSI_STATE_TOPIC "' MQTT topic, '");
      Print(tempStr);
      Println("' topic value");*/
    } else {
      Log("Failed to publish '" HASS_RSSI_STATE_TOPIC "' MQTT topic, '");
      Print(tempStr);
      Println("' topic value");
    }
    nextRSSIPublish += TEMP_PUBLISH_RATE;             // prepare for next publish time
  }
}

// Verify/Make connections
bool connect() {
  static uint32_t lastMQTTRetryTime = millis() - 10 * 1000;
  byte            mac[6];
  IPAddress       ip;

  if (WiFi.status() != WL_CONNECTED) {
    // Wifi is disconnected
    wifiDisconnectOccurred = true;                    // keep track of the fact we were disconnected
    return false;                                     // indicate there is a problem
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
    Println();
  }
  
  if (!mqtt.connected()) {
    // we are not currently connected to MQTT Server
    if (millis() - lastMQTTRetryTime <= 10 * 1000) {
      // not time to retry MQTT connection
      return false;
    }
    // time to retry server connection
    Log("Connecting to MQTT Server:" MQTT_SERVER "...");
    if (mqtt.connect(BOARD_NAME, MQTT_USERNAME, MQTT_PASSWORD)) {
      // successfully connected to MQTT server
      Println("Success");
    } else {
      // failed to connect to MQTT server
      Println("Failed");
      lastMQTTRetryTime = millis();
      return false;
    }
    
    if (!mqtt.publish(HASS_TEMP_CONFIG_TOPIC, HASS_TEMP_CONFIG, true, 1)) {
      Logln("Failed to publish '" HASS_TEMP_CONFIG_TOPIC "' MQTT topic");
    } else {
      Logln("Published '" HASS_TEMP_CONFIG_TOPIC "' MQTT topic, '" HASS_TEMP_CONFIG "' topic value");
    }
    
    // Publish Home Assistant RSSI config topic
    if (!mqtt.publish(HASS_RSSI_CONFIG_TOPIC, HASS_RSSI_CONFIG, true, 1)) {
      Logln("Failed to publish '" HASS_RSSI_CONFIG_TOPIC "' MQTT topic");
    } else {
      Logln("Published '" HASS_RSSI_CONFIG_TOPIC "' MQTT topic, '" HASS_RSSI_CONFIG "' topic value");
    }
    
    // Publish Home Assistant pump config topic
    if (!mqtt.publish(HASS_PUMP_CONFIG_TOPIC, HASS_PUMP_CONFIG, true, 1)) {
      Logln("Failed to publish '" HASS_PUMP_CONFIG_TOPIC "' MQTT topic");
    } else {
      Logln("Published '" HASS_PUMP_CONFIG_TOPIC "' MQTT topic, '" HASS_PUMP_CONFIG "' topic value");
    }
    
    // Publish Home Assistant alarm config topic
    if (!mqtt.publish(HASS_ALARM_CONFIG_TOPIC, HASS_ALARM_CONFIG, true, 1)) {
      Logln("Failed to publish '" HASS_ALARM_CONFIG_TOPIC "' MQTT topic");
    } else {
      Logln("Published '" HASS_ALARM_CONFIG_TOPIC "' MQTT topic, '" HASS_ALARM_CONFIG "' topic value");
    }
    
    // Publish Home Assistant pump status config topic
    if (!mqtt.publish(HASS_STATUS_CONFIG_TOPIC, HASS_STATUS_CONFIG, true, 1)) {
      Logln("Failed to publish '" HASS_STATUS_CONFIG_TOPIC "' MQTT topic");
    } else {
      Logln("Published '" HASS_STATUS_CONFIG_TOPIC "' MQTT topic, '" HASS_STATUS_CONFIG "' topic value");
    }
    
    if (resetOccurred) {                              // Hardware reset occurred
      // reset just recently occurred
      if (!mqtt.publish(HASS_STATUS_STATE_TOPIC, "Reset Hardware", true, 1)) {
        Logln("Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Reset Hardware' topic value");
      } else {
        Logln("Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Reset Hardware' topic value");
      }
    } else if (wifiConnectOccurred) {                 // WiFi Connected
      // Wifi just connected
      if (!mqtt.publish(HASS_STATUS_STATE_TOPIC, "Reset WiFi Connect", true, 1)) {
        Logln("Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Reset WiFi Connect' topic value");
      } else {
        Logln("Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Reset WiFi Connect' topic value");
      }
    } else {
      // MQTT just connected
      if (!mqtt.publish(HASS_STATUS_STATE_TOPIC, "Reset MQTT Connect", true, 1)) {
        // since no reset or WiFi connect occurred then it was a MQTT Connect
        Logln("Failed to publish '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'ResetMQTT Connect' topic value");
      } else {
        Logln("Published '" HASS_STATUS_STATE_TOPIC "' MQTT topic, 'Reset MQTT Connect' topic value");
      }
    }
    
    // Home Assistant will ignore too frequent publishing on topics
    // Specifically we want HASS_STATUS_STATE_TOPIC to show the Reset events above and the soon to come Idle
    delay(500);
    
    // clear the connect reason flags
    resetOccurred = false;
    wifiConnectOccurred = false;
    
    // we had to reconnect to MQTT Broker make sure at least one false is returned
    return false;
  }
  
  // be ready for next MQTT retry time
  lastMQTTRetryTime = millis();
  
  // if we got here then we were successfull
  return true;
}

// Reset the watchdog timer
inline void watchdogReset(void) {
  #ifdef ENABLE_WATCHDOG
  if (!WDT->STATUS.bit.SYNCBUSY)                // Check if the WDT registers are synchronized
    REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;        // Clear the watchdog timer
  #endif
}