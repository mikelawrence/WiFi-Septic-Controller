/*
  WiFi RTC Time  

  Creates a Real Time Clock on the SAMD ARM processor and sets the time via 
  WiFi101 library. Supports automatic Daylight Saving Time clock adjustments.
  Periodically updates the time from the WiFi101 library.

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

  Determining the start and end dates of Daylight Saving Time came from
  Clive (Max) Maxfield
  https://www.embedded.com/electronics-blogs/max-unleashed-and-unfettered/4441673/Is-that-the-daylight-saving-time-
*/
#include <WiFi101.h>
#include <stdlib.h>
#include "WiFi_Septic_Controller.h"
#include "WiFiRTC.h"

// These defines are used by the Start and End DST date calculator
#define MARCH               3
#define NOVEMBER            11
#define FIRST_DAY           1
#define NUM_DAYS_IN_WEEK    7

/******************************************************************
 * Day of the week calculator for DST
 ******************************************************************/
int dstDayOfWeek(int y, int m, int d) {
  int xRef[NUM_DAYS_IN_WEEK] = {0,6,5,4,3,2,1};
  
  return xRef[(d+=m<3?y--:y-2,23*m/9+d+4+y/4-y/100+y/400)%7];
}

/******************************************************************
 * Is Time Valid? (Updated from NTP server)
 ******************************************************************/
bool WiFiRTCClass::isValidTime() {
  return _validTime;
}

/******************************************************************
 * Is it Daylight Savings Time?
 ******************************************************************/
bool WiFiRTCClass::isDST() {
  if (_autoDST) {
    int year    = _rtc.getYear() + 2000;
    int month   = _rtc.getMonth();
    int day     = _rtc.getDay();
    int hour    = _rtc.getHours();
    int minutes = _rtc.getMinutes();
    int seconds = _rtc.getSeconds();
    
    // determine daylight savings time start and end dates
    int startDayDST = FIRST_DAY + NUM_DAYS_IN_WEEK + 
                      dstDayOfWeek(year, MARCH, FIRST_DAY);
    int endDayDST = FIRST_DAY + 
                    dstDayOfWeek(year, NOVEMBER, FIRST_DAY);

    // update DST change day and changed variables
    if ((month == MARCH && day == startDayDST) || 
        (month == NOVEMBER && day == endDayDST)) {
      // today IS a Daylight Savings Time update day
      _isDSTChangeDay = true;
    } else {
      // today IS NOT a Daylight Savings Time update day
      _isDSTChangeDay = false;
      _isDSTChanged = false;
    }

    // get DST status
    bool DST = true;
    if (month < MARCH) {
      // before March
      DST = false;
    } else if (month == MARCH) {
      // we are in March
      if (day < startDayDST) {
        // March but before start day
        DST = false;
      } else if (day == startDayDST) {
        // we are on start day
        if (hour < 2) {
          // before 2:00 AM
          DST = false;
        }
      }
    }
    else if (month > NOVEMBER) {
      // after November
      DST = false;
    } else if (month == NOVEMBER) {
      // we are in November
      if (day > endDayDST) {
        // November but after end day
        DST = false;
      } else if (day == endDayDST) {
        // we are on end day
        if (hour >= 2) {
          // after 2:00 AM
          DST = false;
        }
      }
    }

    // return DST status
    return DST;
    
  } else {
    // automatic DST is off so we do not adjust for DST
    _isDSTChangeDay = false;
    _isDSTChanged = false;
    return false;
  }  
}

/******************************************************************
 * Default Constructor
 ******************************************************************/
WiFiRTCClass::WiFiRTCClass() {
  _configured = false;              // We haven't been configured yet
  _validTime = false;               // RTC has not been set via NTP
  _tzDiff = 0;                      // timezone not set
  _autoDST = false;                 // Automatic DST off
}

/******************************************************************
 * Begin everything
 ******************************************************************/
void WiFiRTCClass::begin(int8_t tzDiff, bool autoDST) {
  _rtc.begin();                     // start the Zero RTC
  _tzDiff = tzDiff;                 // timezone offset without DST, txDiff is in hours
  _autoDST = autoDST;               // do we want automatic DST updates
  // last update and check time is two hours ago
  _lastUpdateTime = millis() - 2 * 60 * 60 * 1000;
  _configured = true;               // we are now configured
}

/******************************************************************
 * Keep things up to date
 ******************************************************************/
void WiFiRTCClass::loop() {
  // do nothing if not configured
  if (!_configured) {
    return;
  }

  // check time if top of the hour or 
  //   not valid time and top of minute or
  //   it's been more than an hour since last check
  if ((_rtc.getMinutes() == 0 && _rtc.getSeconds() == 0) || 
      (millis() - _lastUpdateTime > (60 * 60 * 1000)) ||
      (!_validTime && _rtc.getSeconds() == 0)) {
    // update time from WINC1500 module
    uint32_t epoch = WiFi.getTime();
    if (epoch != 0) {
      // time from WINC1500 module is valid, adjust for timezone difference
      epoch = epoch + (uint32_t)(_tzDiff * 60 * 60);
      _rtc.setEpoch(epoch);         // set RTC time
      if (isDST()) {
        // Daylight Savings Time is in effect
        epoch += 60 * 60;           // add an hour
        _rtc.setEpoch(epoch);       // set RTC time
        if (_isDSTChangeDay) {
          _isDSTChanged = true;     // stop further adjustments to time due to DST
        }
      }
      _validTime = true;            // time is now valid
      Println("Updated time from WINC1500.");
    } else {
      // time for WINC1500 module IS NOT valid, check for DST adjustment of RTC time
      bool DST = isDST();           // get current DST status
      if (_validTime && _isDSTChangeDay && !_isDSTChanged) {
        epoch = _rtc.getEpoch();    // get RTC epoch
        if (_rtc.getMonth() < 6) {
          // we are looking for change from ST to DST
          if (DST) {
            epoch += 60 * 60;       // add an hour
            _rtc.setEpoch(epoch);   // set RTC time
            _isDSTChanged = true;   // stop further adjustments to time due to DST change
          }
        } else {
          // we are looking for a change from DST to ST
          if (!DST) {
            epoch -= 60 * 60;       // subtract an hour
            _rtc.setEpoch(epoch);   // set RTC time
            _isDSTChanged = true;   // stop further adjustments to time due to DST change
          }
        }
      }
      Println("Failed to update time from WINC1500.");
    }
    _lastUpdateTime = millis();     // last update is now
  }
}


/******************************************************************
 * Get the current Second
 ******************************************************************/
uint8_t WiFiRTCClass::getSeconds() {
  return _rtc.getSeconds();
}

/******************************************************************
 * Get the current Minute
 ******************************************************************/
uint8_t WiFiRTCClass::getMinutes() {
  return _rtc.getMinutes();
}

/******************************************************************
 * Get the current Hour
 ******************************************************************/
uint8_t WiFiRTCClass::getHours() {
  return _rtc.getHours();
}

/******************************************************************
 * Get the current Day
 ******************************************************************/
uint8_t WiFiRTCClass::getDay() {
  return _rtc.getDay();
}

/******************************************************************
 * Get the current Month
 ******************************************************************/
uint8_t WiFiRTCClass::getMonth() {
  return _rtc.getMonth();
}

/******************************************************************
 * Get the current Year
 ******************************************************************/
uint16_t WiFiRTCClass::getYear() {
  return (uint16_t) _rtc.getYear() + 2000;
}


/******************************************************************
 * Store time in 12 hour format to timeStr pointer like 9:33:10 PM
 *   Must have at least 12 characters room to store the string
 ******************************************************************/
void WiFiRTCClass::getTimeHMSStr(char *timeStr) {
  bool pm = false;
  uint8_t hours = _rtc.getHours();

  if (hours >= 12) {
    pm = true;
  }
  if (hours > 12) {
    hours -= 12;
  }
  uint8_t minutes = _rtc.getMinutes();
  uint8_t seconds = _rtc.getSeconds();
  if (hours < 10) {
    // single digit hour
    *timeStr++ = '0';
    itoa(hours, timeStr, 10);
    ++timeStr;
  } else {
    // double digit hour
    itoa(hours, timeStr, 10);
    timeStr += 2;
  }
  *timeStr++ = ':';
  if (minutes < 10) {
    // single digit minutes
    *timeStr++ = '0';
    itoa(minutes, timeStr, 10);
    ++timeStr;
  } else {
    // double digit minutes
    itoa(minutes, timeStr, 10);
    timeStr += 2;
  }
  *timeStr++ = ':';
  if (seconds < 10) {
    // single digit seconds
    *timeStr++ = '0';
    itoa(seconds, timeStr, 10);
    ++timeStr;
  } else {
    // double digit seconds
    itoa(seconds, timeStr, 10);
    timeStr += 2;
  }
  *timeStr++ = ' ';
  if (pm) {
    *timeStr++ = 'P';
  } else {
    *timeStr++ = 'A';
  }
  *timeStr++ = 'M';
  *timeStr = 0;
}

/******************************************************************
 * Store time in 12 hour format to timeStr pointer like 9:33 PM
 *   Must have at least 9 characters room to store the string
 ******************************************************************/
void WiFiRTCClass::getTimeHMStr(char *timeStr) {
  bool pm = false;
  uint8_t hours = _rtc.getHours();

  if (hours >= 12) {
    pm = true;
  }
  if (hours > 12) {
    hours -= 12;
  }
  uint8_t minutes = _rtc.getMinutes();
  uint8_t seconds = _rtc.getSeconds();
  if (hours < 10) {
    // single digit hour
    itoa(hours, timeStr, 10);
    ++timeStr;
  } else {
    // double digit hour
    itoa(hours, timeStr, 10);
    timeStr += 2;
  }
  *timeStr++ = ':';
  if (minutes < 10) {
    // single digit minutes
    *timeStr++ = '0';
    itoa(minutes, timeStr, 10);
    ++timeStr;
  } else {
    // double digit minutes
    itoa(minutes, timeStr, 10);
    timeStr += 2;
  }
  //*timeStr++ = ' ';
  if (pm) {
    *timeStr++ = 'P';
  } else {
    *timeStr++ = 'A';
  }
  *timeStr++ = 'M';
  *timeStr = 0;
}

/******************************************************************
 * Store time in 24 hour format to timeStr pointer like 09:33:10
 *   Must have at least 9 characters room to store the string
 ******************************************************************/
void WiFiRTCClass::getTimeHMS24HrStr(char *timeStr) {
  uint8_t hours = _rtc.getHours();
  uint8_t minutes = _rtc.getMinutes();
  uint8_t seconds = _rtc.getSeconds();
  if (hours < 10) {
    // single digit hour
    itoa(hours, timeStr, 10);
    ++timeStr;
  } else {
    // double digit hour
    itoa(hours, timeStr, 10);
    timeStr += 2;
  }
  *timeStr++ = ':';
  if (minutes < 10) {
    // single digit minutes
    *timeStr++ = '0';
    itoa(minutes, timeStr, 10);
    ++timeStr;
  } else {
    // double digit minutes
    itoa(minutes, timeStr, 10);
    timeStr += 2;
  }
  *timeStr++ = ':';
  if (seconds < 10) {
    // single digit seconds
    *timeStr++ = '0';
    itoa(seconds, timeStr, 10);
  } else {
    // double digit seconds
    itoa(seconds, timeStr, 10);
  }
}


/******************************************************************
 * Single instance of WiFiRTCClass
 ******************************************************************/
WiFiRTCClass WiFiRTC;
