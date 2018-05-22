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
*/
#ifndef WIFI_RTC_H
#define WIFI_RTC_H

#include <RTCZero.h>

class WiFiRTCClass {
public:
  WiFiRTCClass();
  
  void      begin(int8_t tzDiff = 0, bool autoDST = true);
  void      loop(void);
  
  uint8_t   getSeconds();
  uint8_t   getMinutes();
  uint8_t   getHours();
  
  uint8_t   getDay();
  uint8_t   getMonth();
  uint16_t  getYear();

  void      getTimeHMStr(char* timeStr);
  void      getTimeHMSStr(char* timeStr);
  void      getTimeHMS24HrStr(char* timeStr);

  bool      isValidTime();
  bool      isDST();
  
private:
  // Real Time Clock for SAMD
  RTCZero _rtc;
  // true when we have been configured
  bool _configured = false;
  // true when time has been updated from NTP server
  bool _validTime;
  // true when today a Daylight Savings Time change day
  bool _isDSTChangeDay;
  // true when Daylight Savings Time has been changed today
  //   prevent multiple changes from occurring
  bool _isDSTChanged;
  // last update time in milliseconds
  uint32_t _lastUpdateTime;
  // timezone difference from GMT in hours
  int8_t _tzDiff;
  // true when time should adjust for DST automatically
  bool _autoDST;  
};

extern WiFiRTCClass WiFiRTC;
#endif // WIFI_RTC_H
