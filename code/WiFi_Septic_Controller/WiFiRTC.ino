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

  The NTP client code came from Arduino
  https://github.com/arduino-libraries/NTPClient
  
  Determining the start and end dates of Daylight Saving Time came from
  Clive (Max) Maxfield
  https://www.embedded.com/electronics-blogs/max-unleashed-and-unfettered/4441673/Is-that-the-daylight-saving-time-
*/
#include <WiFi101.h>
#include <stdlib.h>
#include "WiFi_Septic_Controller.h"
#include "WiFiRTC.h"

// NTP defines
//#define NTP_DEFAULT_SERVER      "time.nist.gov"
#define NTP_PACKET_SIZE         48
#define NTP_INCOMING_PORT       2390
#define NTP_OUTGOING_PORT       123
#define SEVENTYYEARS            2208988800UL
// With invalid time or recent missed NTP response how often to update (default 14 seconds)
#define NTP_FAST_UPDATE_TIME    14*1000
// With valid time how often should NTP requests happen (default a little less than 12 hours)
#define NTP_SLOW_UPDATE_TIME    12*60*60*1000 - 3*60*1000

// These defines are used by the Start and End DST date calculator
#define MARCH                   3
#define NOVEMBER                11
#define FIRST_DAY               1
#define NUM_DAYS_IN_WEEK        7

// global variable used to let the one and only WiFiRTC object know it is update time
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
  _configured = false;                  // We haven't been configured yet
  _validTime = false;                   // RTC has not been set via NTP
  _tzDiff = 0;                          // timezone not set
  _autoDST = false;                     // Automatic DST off
}

/******************************************************************
 * Begin everything
 ******************************************************************/
void WiFiRTCClass::begin(int8_t tzDiff, const char* ntpServerName, bool autoDST) {
  _rtc.begin();                         // start the Zero RTC
  _tzDiff = tzDiff;                     // timezone offset without DST, txDiff is in hours
  _timeServerName = ntpServerName;      // save specified NTP server
  _autoDST = autoDST;                   // do we want automatic DST updates
  _configured = true;                   // we are now configured
  _lastUpdateTime = 0 - (NTP_FAST_UPDATE_TIME + 1000);// should update time immediately

  _UDPRequestSent = false;              // we are not currently waiting for an NTP Server response
  _udp.begin(NTP_INCOMING_PORT);        // begin listening for NTP response UDP packets     
  
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
    updateTime();
    WiFiRTC_Update_Time_Now = false;
  }

  // if we aren't connected then we can't be waiting for a UDP NTP response packet
  if (WiFi.status() != WL_CONNECTED) {
    // prep for immediate time update when reconnected
    _validTime = false;
    _UDPRequestSent = false; 
    _lastUpdateTime = millis() - (NTP_FAST_UPDATE_TIME + 1000);
    // do nothing else until we are connected
    return;
  }

  if (_UDPRequestSent) {
      uint8_t packetBuffer[NTP_PACKET_SIZE];
      uint32_t curEpoch = _rtc.getEpoch();
    // we are waiting for a response from NTP Server
    if (_udp.parsePacket()) {
      // packet received from NTP server
      _udp.read(packetBuffer, NTP_PACKET_SIZE);
    } else {
      // no packet received from NTP server
      if (millis() - _lastUDPRequestTime > 2*1000) {
        // timeout ocurred waiting for NTP response
        _UDPRequestSent = false;      // not waiting for UDP response anymore
        _validTime = false;           // will switch to faster update until success
        Logln("Failed to receive response from NTP Server.");
      }
      // done with current loop execution
      return;
    }
        
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    uint32_t newEpoch = word(packetBuffer[40], packetBuffer[41]) << 16 | 
                     word(packetBuffer[42], packetBuffer[43]);
    // convert NTP epoch (since 1900) to UNIX epoch (since 1970)
    newEpoch -= SEVENTYYEARS;
    // adjust for timezone difference
    newEpoch += (uint32_t)(_tzDiff * 60 * 60);
    // Nearest second round comes from https://github.com/aharshac/EasyNTPClient
    // Round to the nearest second if we want accuracy
    // The fractionary part is the next byte divided by 256: if it is
    // greater than 500ms we round to the next second; we also account
    // for an assumed network delay of 50ms and 10ms processing time
    if (packetBuffer[44] > (uint8_t)(0.5-0.06)*256) {
      newEpoch ++;
    }
    _rtc.setEpoch(newEpoch);          // set RTC time
    updateTime();                     // update local time too
    if (_isDST) {
      // Daylight Savings Time is in effect
      newEpoch += 60 * 60;            // add an hour
      _rtc.setEpoch(newEpoch);        // set RTC time
      updateTime();                   // update local time again since we adjusted for DST
      if (_isDSTChangeDay) {
        _isDSTChanged = true;         // stop further adjustments to time due to DST
      }
    }
    Log("Updated time from '");       // log updated time
    Print(_timeServerName);
    Print("', time diff = ");
    Print((int32_t)(newEpoch - curEpoch));
    Println(" sec.");
    _validTime = true;                // time is now valid
    _UDPRequestSent = false;          // no longer waiting for a response
    _lastUpdateTime = _lastUDPRequestTime; // time was just updated
  } else {
    // we are not waiting for a response from NTP Server 
    // Time to sync with NTP server if
    //   WiFi is connected and
    //   not valid time and it's been longer than NTP_FAST_UPDATE_TIME
    //   or valid time and it's been longer than NTP_SLOW_UPDATE_TIME
    if ((!_validTime && lastUpdateTimeDiff > NTP_FAST_UPDATE_TIME) ||
        (_validTime && lastUpdateTimeDiff > NTP_SLOW_UPDATE_TIME)) {
      // send a request to NTP server
      sendNTPPacket();
      _lastUDPRequestTime = millis(); // start UDP request time  
      _lastUpdateTime = _lastUDPRequestTime; // start last update time too
      _UDPRequestSent = true;         // let class know we sent a packet
    } else {
      // not time to update, check for DST adjustment of RTC time
      if (_validTime && _isDSTChangeDay && !_isDSTChanged) {
        Logln("Time invalidated due to Daylight Savings Time change.");
        // time is NOT valid
        _validTime = false;       
        // force an immediate update of time
        _lastUpdateTime = millis() - NTP_FAST_UPDATE_TIME - 1000; 
        // stop further adjustments to time due to DST change
        _isDSTChanged = true;
      }
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

  // determine daylight savings time
  if (_autoDST) {
    // start with DST as true
    _isDST = true;
    
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
    if (_month < MARCH) {
      // before March
      _isDST = false;
    } else if (_month == MARCH) {
      // we are in March
      if (_day < startDayDST) {
        // March but before start day
        _isDST = false;
      } else if (_day == startDayDST) {
        // we are on start day
        if (_hour < 2) {
          // before 2:00 AM
          _isDST = false;
        }
      }
    }
    else if (_month > NOVEMBER) {
      // after November
      _isDST = false;
    } else if (_month == NOVEMBER) {
      // we are in November
      if (_day > endDayDST) {
        // November but after end day
        _isDST = false;
      } else if (_day == endDayDST) {
        // we are on end day
        if (_hour >= 2) {
          // after 2:00 AM
          _isDST = false;
        }
      }
    }    
  } else {
    // automatic DST is off so we do not adjust for DST
    _isDSTChangeDay = false;
    _isDSTChanged = false;
    _isDST = false;
  }
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
  updateTime();             // update current time variables from RTC
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
 * Store time in 12 hour format to timeStr pointer like 9:33:10PM
 *   Must have at least 11 characters room to store the string
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
    *timeStr++ = '0';
    itoa(hour, timeStr, 10);
    timeStr += 1;
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
 * Send NTP request UDP
 ******************************************************************/
void WiFiRTCClass::sendNTPPacket() {
  uint8_t packetBuffer[NTP_PACKET_SIZE];
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  packetBuffer[ 0] = 0b11100011;     // LI, Version, Mode
  packetBuffer[ 1] = 0;              // Stratum, or type of clock
  packetBuffer[ 2] = 6;              // Polling Interval
  packetBuffer[ 3] = 0xEC;           // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  IPAddress timeServer(129, 6, 15, 28);
  _udp.beginPacket(_timeServerName, NTP_OUTGOING_PORT);
  _udp.write(packetBuffer, NTP_PACKET_SIZE);
  _udp.endPacket();
}

/******************************************************************
 * Single instance of WiFiRTCClass
 ******************************************************************/
WiFiRTCClass WiFiRTC;
