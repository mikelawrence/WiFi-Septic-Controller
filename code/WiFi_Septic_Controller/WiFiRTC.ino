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

bool WiFiRTC_Update_Time_Now = false;

/******************************************************************
 * Day of the week calculator for DST
 ******************************************************************/
int dstDayOfWeek(int y, int m, int d) {
  int xRef[NUM_DAYS_IN_WEEK] = {0,6,5,4,3,2,1};
  
  return xRef[(d+=m<3?y--:y-2,23*m/9+d+4+y/4-y/100+y/400)%7];
}

/******************************************************************
 * Time Match callback once a second
 ******************************************************************/
void alarmMatch()
{
  WiFiRTC_Update_Time_Now = true;
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
  _configured = true;               // we are now configured
  _lastUpdateTime = 0 - 60*60*1000; // last update time was 1 hour ago
  
  // update local time
  updateTime();

  // setup an alarm for every minute
  _rtc.setAlarmTime(0, 0, 0);
  _rtc.attachInterrupt(alarmMatch);  
  _rtc.enableAlarm(_rtc.MATCH_SS);
}

/******************************************************************
 * Keep things up to date
 ******************************************************************/
void WiFiRTCClass::loop() {
  uint32_t lastUpdateTimeDiff = millis() - _lastUpdateTime;
  // do nothing if not configured
  if (!_configured) {
    return;
  }

  // did RTC alarm match interrupt indicate time to update?
  if (WiFiRTC_Update_Time_Now) {
    WiFiRTC.updateTime();
    WiFiRTC_Update_Time_Now = false;;
  }
  
  // check time if top of the hour or 
  //   not valid time and top of minute or
  //   it's been more than an hour since last check
  if ((!_validTime && lastUpdateTimeDiff > 14*1000) ||    // every 14 seconds
      (_validTime && lastUpdateTimeDiff > 47*60*1000)) {  // every 47 minutes
    // update time from WINC1500 module
    uint32_t epoch = WiFi.getTime();
    if (epoch != 0) {
      Log("Updating time from WINC1500, new time = ");
      // time from WINC1500 module is valid, adjust for timezone difference
      epoch = epoch + (uint32_t)(_tzDiff * 60 * 60);
      _rtc.setEpoch(epoch);         // set RTC time
      updateTime();                 // update local time too
      if (isDST()) {
        // Daylight Savings Time is in effect
        epoch += 60 * 60;           // add an hour
        _rtc.setEpoch(epoch);       // set RTC time
        updateTime();               // update local time now
        if (_isDSTChangeDay) {
          _isDSTChanged = true;     // stop further adjustments to time due to DST
        }
      }
      printTimeHMS24Hr();           // print the new time
      Println("");                  // add linefeed
      _lastUpdateTime = millis();   // update last time was checked
      _validTime = true;            // time is now valid
    } else {
      // time for WINC1500 module IS NOT valid, check for DST adjustment of RTC time
      bool DST = isDST();           // get current DST status
      if (_validTime && _isDSTChangeDay && !_isDSTChanged) {
        epoch = _rtc.getEpoch();    // get RTC epoch
        if (_month < 6) {
          // we are looking for change from ST to DST
          if (DST) {
            epoch += 60 * 60;       // add an hour
          }
        } else {
          // we are looking for a change from DST to ST
          if (!DST) {
            epoch -= 60 * 60;       // subtract an hour
          }
        }
        _rtc.setEpoch(epoch);       // set RTC time
        updateTime();               // update local time now
        _isDSTChanged = true;       // stop further adjustments to time due to DST change
      }
      _validTime = false;           // time is NOT valid
      _lastUpdateTime = millis();   // update last time was checked
      Logln("Failed to update time from WINC1500.");
    }
  }
}

/******************************************************************
 * Get the current time from RTC and save in local variables
 ******************************************************************/
void WiFiRTCClass::updateTime() {
  RTC_MODE2_CLOCK_Type clockTime = _rtc.getTime();

  // save local variables
  _year = clockTime.bit.YEAR + 2000;
  _month = clockTime.bit.MONTH;
  _day = clockTime.bit.DAY;
  _hour = clockTime.bit.HOUR;
  _minute = clockTime.bit.MINUTE;
  _second = clockTime.bit.SECOND;
}

/******************************************************************
 * Is Time Valid? (Updated from NTP server)
 ******************************************************************/
bool WiFiRTCClass::isValidTime() {
  return _validTime;
}

/******************************************************************
 * Get the current Second
 ******************************************************************/
uint8_t WiFiRTCClass::getSecond() {
  // update seconds
  _second = _rtc.getSeconds();
  
  return _second;
}

/******************************************************************
 * Get the current Minute
 ******************************************************************/
uint8_t WiFiRTCClass::getMinute() {
  return _minute;
}

/******************************************************************
 * Get the current Hour
 ******************************************************************/
uint8_t WiFiRTCClass::getHour() {
  return _hour;
}

/******************************************************************
 * Get the current Day
 ******************************************************************/
uint8_t WiFiRTCClass::getDay() {
  return _day;
}

/******************************************************************
 * Get the current Month
 ******************************************************************/
uint8_t WiFiRTCClass::getMonth() {
  return _month;
}

/******************************************************************
 * Get the current Year
 ******************************************************************/
uint16_t WiFiRTCClass::getYear() {
  return _year;
}

/******************************************************************
 * Is it Daylight Savings Time?
 ******************************************************************/
bool WiFiRTCClass::isDST() {
  bool DST = true;
  
  // determine daylight savings time
  if (_autoDST) {
    // determine daylight savings time start and end dates
    int startDayDST = FIRST_DAY + NUM_DAYS_IN_WEEK + 
                      dstDayOfWeek(_year, MARCH, FIRST_DAY);
    int endDayDST = FIRST_DAY + 
                    dstDayOfWeek(_year, NOVEMBER, FIRST_DAY);

    // update DST change day and changed variables
    if ((_month == MARCH && _day == startDayDST) || 
        (_month == NOVEMBER && _day == endDayDST)) {
      // today IS a Daylight Savings Time update day
      _isDSTChangeDay = true;
    } else {
      // today IS NOT a Daylight Savings Time update day
      _isDSTChangeDay = false;
      _isDSTChanged = false;
    }

    // get DST status
    DST = true;
    if (_month < MARCH) {
      // before March
      DST = false;
    } else if (_month == MARCH) {
      // we are in March
      if (_day < startDayDST) {
        // March but before start day
        DST = false;
      } else if (_day == startDayDST) {
        // we are on start day
        if (_hour < 2) {
          // before 2:00 AM
          DST = false;
        }
      }
    }
    else if (_month > NOVEMBER) {
      // after November
      DST = false;
    } else if (_month == NOVEMBER) {
      // we are in November
      if (_day > endDayDST) {
        // November but after end day
        DST = false;
      } else if (_day == endDayDST) {
        // we are on end day
        if (_hour >= 2) {
          // after 2:00 AM
          DST = false;
        }
      }
    }    
  } else {
    // automatic DST is off so we do not adjust for DST
    _isDSTChangeDay = false;
    _isDSTChanged = false;
    DST = false;
  }

  return DST;
}

/******************************************************************
 * Store time in 12 hour format to timeStr pointer like 9:33:10PM
 *   Must have at least 12 characters room to store the string
 ******************************************************************/
void WiFiRTCClass::getTimeHMS(char *timeStr) {
  bool pm = false;
  uint8_t hour = _hour;

  // update seconds
  _second = _rtc.getSeconds();
  
  if (hour >= 12) {
    pm = true;
  }
  if (hour > 12) {
    hour -= 12;
  }
  if (hour < 10) {
    // single digit hour
    itoa(hour, timeStr, 10);
    ++timeStr;
  } else {
    // double digit hour
    itoa(hour, timeStr, 10);
    timeStr += 2;
  }
  *timeStr++ = ':';
  if (_minute < 10) {
    // single digit minutes
    *timeStr++ = '0';
    itoa(_minute, timeStr, 10);
    ++timeStr;
  } else {
    // double digit minutes
    itoa(_minute, timeStr, 10);
    timeStr += 2;
  }
  *timeStr++ = ':';
  if (_second < 10) {
    // single digit seconds
    *timeStr++ = '0';
    itoa(_second, timeStr, 10);
    ++timeStr;
  } else {
    // double digit seconds
    itoa(_second, timeStr, 10);
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
 * Store time in 12 hour format to timeStr pointer like 9:33PM
 *   Must have at least 9 characters room to store the string
 ******************************************************************/
void WiFiRTCClass::getTimeHM(char *timeStr) {
  bool pm = false;
  uint8_t hour = _hour;

  if (_hour >= 12) {
    pm = true;
  }
  if (_hour > 12) {
    hour -= 12;
  }
  if (hour < 10) {
    // single digit hour
    itoa(hour, timeStr, 10);
    ++timeStr;
  } else {
    // double digit hour
    itoa(hour, timeStr, 10);
    timeStr += 2;
  }
  *timeStr++ = ':';
  if (_minute < 10) {
    // single digit minutes
    *timeStr++ = '0';
    itoa(_minute, timeStr, 10);
    ++timeStr;
  } else {
    // double digit minutes
    itoa(_minute, timeStr, 10);
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
 * Store time in 24 hour format to timeStr pointer like 14:33:10
 *   Must have at least 9 characters room to store the string
 ******************************************************************/
void WiFiRTCClass::getTimeHMS24Hr(char *timeStr) {
  // update seconds
  _second = _rtc.getSeconds();
  
  if (_hour < 10) {
    // single digit hour
    *timeStr++ = '0';
    itoa(_hour, timeStr, 10);
    ++timeStr;
  } else {
    // double digit hour
    itoa(_hour, timeStr, 10);
    timeStr += 2;
  }
  *timeStr++ = ':';
  if (_minute < 10) {
    // single digit minutes
    *timeStr++ = '0';
    itoa(_minute, timeStr, 10);
    ++timeStr;
  } else {
    // double digit minutes
    itoa(_minute, timeStr, 10);
    timeStr += 2;
  }
  *timeStr++ = ':';
  if (_second < 10) {
    // single digit seconds
    *timeStr++ = '0';
    itoa(_second, timeStr, 10);
  } else {
    // double digit seconds
    itoa(_second, timeStr, 10);
  }
}

/******************************************************************
 * Print time in 12 hour format like 9:33:10PM
 ******************************************************************/
void WiFiRTCClass::printTimeHMS() {
#ifdef ENABLE_SERIAL
  char timeStr[12];

  getTimeHMS(timeStr);

  Print(timeStr);
#endif
}

/******************************************************************
 * Print time in 12 hour format like 9:33PM
 ******************************************************************/
void WiFiRTCClass::printTimeHM() {
#ifdef ENABLE_SERIAL
  char timeStr[9];

  getTimeHM(timeStr);

  Print(timeStr);
#endif
}

/******************************************************************
 * Print time in 24 hour format like 14:33:10
 ******************************************************************/
void WiFiRTCClass::printTimeHMS24Hr() {
#ifdef ENABLE_SERIAL
  char timeStr[9];

  getTimeHMS24Hr(timeStr);

  Print(timeStr);
#endif
}

/******************************************************************
 * Single instance of WiFiRTCClass
 ******************************************************************/
WiFiRTCClass WiFiRTC;
