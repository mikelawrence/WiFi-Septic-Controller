/*
  WiFi Septic Controller 16X2 LCD Module

  Supports the 16X2 RGB LCD display on the Wifi Septic Contoller board.

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
#include <stdlib.h>
#include <string.h>
#include <avr/dtostrf.h>
#include "SepticLCD.h"

// Text Liquid Crystal display
static LiquidCrystal _lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

/******************************************************************
 * Default Constructor
 ******************************************************************/
SepticLCDClass::SepticLCDClass() :
  _lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7),
  _oneWire(OWIRE),
  _oneWireTemp(&_oneWire)
{
}

/******************************************************************
 * Begin everything
 ******************************************************************/
void SepticLCDClass::begin(void) {
  _lcd.begin(16, 2);                  // start the LiquidCrystal library (16X2)
  
  pinMode(LCD_BACKLIGHT_R, OUTPUT);   // configure LCD RGB backlight PWM outputs
  pinMode(LCD_BACKLIGHT_G, OUTPUT);
  pinMode(LCD_BACKLIGHT_B, OUTPUT);
  setBacklightColor(255, 255, 255);   // backlight defaults to white

  // fill row strings with test all on pattern
  for (int i = 0; i < 16; i++) {
    _row1[i] = 255;
    _row2[i] = 255;
  }

  updateDisplay();                    // update the display
  _oneWireTemp.begin();               // Start up the Dallas Temperature library 
  _oneWireTemp.requestTemperatures(); // start the first temperature conversion
  _configured = true;                 // we are now configured
}

/******************************************************************
 * Keep things up to date
 ******************************************************************/
void SepticLCDClass::loop(void) {
  
  static uint16_t lastHour = 100;                     // last hour this loop had
  static uint8_t lastMinute = 100;                    // last minute this loop had
  static uint8_t lastSecond = 100;                    // last second this loop had
  uint16_t curHour = WiFiRTC.getHour();               // current hour
  uint16_t curMinute = WiFiRTC.getMinute();           // current minutes
  uint8_t curSecond = WiFiRTC.getSecond();            // current second
  char tempStr[16] = "";                              // temporary string

  if (!_configured) {                                 // do nothing if not configured
    return;
  }

  bool update = false;                                // update display flag

  if (_row1[0] == 255) {
    // this it the first time this loop has run
    for (int i = 0; i < 16; i++) {
      _row1[i] = ' ';
    }
    updateTime();                                     // add time row strings
    updateTemp();                                     // add temperature row strings
    updateMessage();                                  // add default message
    updateDisplay();                                  // update the display
    lastHour = curHour;
    lastMinute = curMinute;
    lastSecond = curSecond;                           // update last time
    return;
  }

  // check for changes in time, will occur every minute
  if (lastHour != curHour || lastMinute != curMinute) {
    // time needs to be updated
    updateTime();                                     // update time in row strings
    update = true;                                    // display has been changed
  }
  if (lastSecond != curSecond) {                      // seconds changed?    
    // is it time to request a temperature?
    if (_oneWireTemp.getDeviceCount() > 0) {          // we do nothing if there is no 1-wire sensor
      uint8_t secondsMod5 = curSecond % 5;
      if (secondsMod5 == 0) {                         // time to request a temperature conversion?
        _oneWireTemp.requestTemperatures();           // start temperature conversion
      } else if (secondsMod5 == 1) {                  // is conversion is complete?
        float temp = _oneWireTemp.getTempCByIndex(0); // get the temperature
        if (temp != _temperatureC) {                  // update only if temperature is different
          _temperatureC = temp;                       // keep track of last temperature in C
          _temperatureF = (temp * 9.0 / 5.0) + 32.0;  // keep track of last temperature in F
          updateTemp();                               // update temperature in row string
          update = true;                              // display has been changed
        }
      }
    }
  }

  if (update) {
    updateDisplay();                                  // time to update display
  }
  
  // update last time
  lastHour = curHour;
  lastMinute = curMinute;
  lastSecond = curSecond;
}

/******************************************************************
 * Sets the LCD Backlight Color given Red, Green, and Blue
 ******************************************************************/
void SepticLCDClass::setBacklightColor(uint8_t r, uint8_t g, uint8_t b) {
  if (r == 255) {
    pinMode(LCD_BACKLIGHT_R, OUTPUT);
    digitalWrite(LCD_BACKLIGHT_R, HIGH);
  } else {
    analogWrite(LCD_BACKLIGHT_R, r);
  }
  if (g == 255) {
    pinMode(LCD_BACKLIGHT_G, OUTPUT);
    digitalWrite(LCD_BACKLIGHT_G, HIGH);
  } else {
    analogWrite(LCD_BACKLIGHT_G, g);
  }
  if (b == 255) {
    pinMode(LCD_BACKLIGHT_B, OUTPUT);
    digitalWrite(LCD_BACKLIGHT_B, HIGH);
  } else {
    analogWrite(LCD_BACKLIGHT_B, b);
  }
}

/******************************************************************
 * Sets the LCD Backlight Color given color number
 ******************************************************************/
void SepticLCDClass::setBacklightColor(uint8_t color) {
  switch (color) {
    case COLOR_PUMPING:
      setBacklightColor(0, 0, 255); // blue
      break;
    case COLOR_ALARM:
      setBacklightColor(255, 0, 0);     // red
      break;
    case COLOR_IDLE:
      setBacklightColor(0, 255, 0);     // green
      break;
    default:
      setBacklightColor(255, 255, 255); // white
      break;  
  }
}

/******************************************************************
 * Sets the current display message
 ******************************************************************/
void SepticLCDClass::setDisplayMessage(const char* msg) {
  // keep a copy the new string
  strncpy(_message, msg, sizeof(_message));
  updateMessage();          // update the message on row strings
}

/******************************************************************
 * Update LCD display with row strings
 ******************************************************************/
void SepticLCDClass::updateDisplay() {
  _row1[16] = 0;            // terminate row strings
  _row2[16] = 0;
  _lcd.setCursor(0, 0);     // point to beginning of 1st row
  _lcd.print(_row1);        // write 1st row
  _lcd.setCursor(0, 1);     // point to beginning of 2nd row
  _lcd.print(_row2);        // write 2nd row
}

/******************************************************************
 * Update time in row string
 ******************************************************************/
void SepticLCDClass::updateTime() {
  char timeStr[8];
  
  WiFiRTC.getTimeHM(timeStr);  // get current time
  strncpy(_row1, timeStr, 7);     // copy time to row string
  if (strlen(timeStr) < 7) {
    _row1[6] = ' ';               // add a space to the end
  }
}

/******************************************************************
 * Update temperature in row string
 ******************************************************************/
void SepticLCDClass::updateTemp() {
  char tempStr[10];
  int  column;

  if (_temperatureF == -100.0) {
    // no temperature sensor detected
    strncpy(&_row1[9], "No Temp", 7);     // copy error to row string
    return;
  }
  dtostrf(_temperatureF, 5, 1, tempStr);  // convert temp in F to string
  strncpy(&_row1[9], tempStr, 5);         // copy temp to row string
  _row1[14] = 223;                        // degree symbol
  _row1[15] = 'F';                        // farenheit
}

/******************************************************************
 * Update message in row string
 ******************************************************************/
void SepticLCDClass::updateMessage() {
  // copy at most 16 characters to row2
  strncpy(_row2, _message, 16); 
  // fill with spaces the rest of the row
  for (int i = strlen(_message); i < 16; i++) {
    _row2[i] = ' ';
  }
  _lcd.setCursor(0, 1);         // point to beginning of 2nd row
  _lcd.print(_row2);            // write 2nd row 
}

/******************************************************************
 * Single instance of SepticLCDCLass
 ******************************************************************/
SepticLCDClass SepticLCD;
