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
#ifndef SEPTIC_LCD_H
#define SEPTIC_LCD_H

#include <LiquidCrystal.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
#include "WiFi_Septic_Controller.h"

#define COLOR_IDLE      1
#define COLOR_PUMPING   2
#define COLOR_ALARM     3

class SepticLCDClass {
public:
  SepticLCDClass();

  void begin(void);
  void loop(void);

  void setBacklightColor(uint8_t r, uint8_t g, uint8_t b);
  void setBacklightColor(uint8_t color);
  void setDisplayMessage(const char *msg);
  void setDisplayMessage(const String &msg) {
    return setDisplayMessage( msg.c_str() ); 
  }
  float getTemperatureC() {
    return _temperatureC;
  }
  float getTemperatureF() {
    return _temperatureF;
  }
  
private:
  void updateTime();
  void updateTemp();
  void updateMessage();
  void updateDisplay();
  
private:
  LiquidCrystal     _lcd;
  OneWire           _oneWire; 
  DallasTemperature _oneWireTemp;
  bool              _configured = false;
  char              _message[128];
  float             _temperatureC = -100.0;
  float             _temperatureF = -100.0;
  char              _row1[17];
  char              _row2[17];
  char              _timeStr[9];
};

extern SepticLCDClass SepticLCD;
#endif // SEPTIC_LCD_H
