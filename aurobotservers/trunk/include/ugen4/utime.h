/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef UTIME_H
#define UTIME_H

#include <sys/time.h>
#include "conf.h"
//#include "ucommon.h"


/**
Class encapsulation the time structure used by 'gettimeofday'
with resolution in years down to micro-seconds.
The class has functions to make simple time calculations and
conversion to and from string in localized format. */
class UTime
{
public:
  /**
  Constructor */
  UTime();
  /**
  Destructor */
  ~UTime();
  /**
  Clear to 0.0 */
  void clear();
  /**
  Get time value in seconds (since 1970) */
  unsigned long GetSec();
  inline unsigned long getSec() { return GetSec();};
  /**
  Get milisecond value within second in range 0..999 */
  long GetMilisec();
  inline long getMilisec() { return GetMilisec();};
  /**
  Get microsecond value within second in range 0..999999 */
  unsigned long GetMicrosec();
  inline unsigned long getMicrosec() { return GetMicrosec();};
  /**
  Get second value with microsecond as decimals */
  double GetDecSec();
  inline double getDecSec()
    { return GetDecSec(); };
  /**
  Get time since t1 as decimal seconds. */
  double GetDecSec(UTime t1);
  /**
  Get time past since this time */
  double getTimePassed();
  /**
  Get number of seconds within last minute in range 0..59 */
  inline char GetJustSec()
    { return char(time.tv_sec % 60); };
  inline char getJustSec()
    { return char(time.tv_sec % 60); };
  /**
  Get minute value within last hour in range 0..59 */
  char GetMin();
  /**
  Get hour value on this day in range 0..23 */
  char GetHour(bool local = true);
  /**
  Get number of days with day 1 as 1 jan 1970. */
  long GetDaySerial();
  /**
  Save time value to microseconf to config file */
  int SaveToReg(Uconfig * ini, const char * subject, const char * key);
  /**
  Save time value in XML frmat */
//  bool save(UxmlFile * fxml, const char * name);
  /**
  Load tie value from config file */
  int LoadFromReg(Uconfig * ini, const char * subject, const char * key);
  /**
  Set time value to system time now using gettimeofday() */
  inline void Now()
     { gettimeofday(&time, NULL); valid = true; }
  /**
  Set time value to system time now using gettimeofday() */
  inline void now()
  { gettimeofday(&time, NULL); valid = true; }
  /**
  Set time from a timeval structure */
  void SetTime(timeval iTime);
  /**
  Set time from double decimal second value (less precise (about 1 ms)).
  NB! This may not allow full precition.  */
  void setTime(double decimalSec);
  inline void SetTime(double decimalSec)
     { setTime(decimalSec);};
  /**
  Set time using seconds and microseconds. */
  void SetTime(long sec, long uSec);
  inline void setTime(long sec, long uSec) {SetTime(sec, uSec);};
  /**
  Set time from a time of day string, i.e. 12345678.067877.
  The functions acceps any number of decimals, i.e. 6 decimals or less.
  Sets time invalid if no decimal point is found. */
  void setTimeTod(const char * tod);
  /**
  Set time using unsigned values */
  void setTimeU(unsigned long sec, unsigned long uSec);
  /**
  Set time from date. */
  inline void SetTime(int year, int month, int day,
                int hour = 0, int min = 0, int sec = 0, long usec = 0)
  {
    setTime(year, month, day, hour, min, sec, usec);
  }
  /**
  Set time from date. */
  void setTime(int year, int month, int day,
               int hour = 0, int min = 0, int sec = 0, long usec = 0);
  /**
   * Writes time to INFO in format "hh:mm:ss.msec"
   * \param info destination buffer, must be at least 13 characters long
   * \param local converts time to local time (is system time is UTM or somthing)
  */
  int GetTimeAsString(char * info, bool local = true);
  /**
  Writes time to INFO in format "yyyyMMdd_hhmmss.msec"
   * \param info is a bugger for the string, must be at least 19 characters long.
   * \param local should time be in local time (else UTM if set on computer)
   * \returns pointer to the info buffer */
  char * getForFilename(char * info, bool local = true);
  /**
  Writes time to INFO in format "yyyy-MM-dd hh:mm:ss.msec"
   * \param info is a bugger for the string, must be at least 24 characters long.
   * \param local should time be in local time (else UTM if set on computer)
   * \returns pointer to the info buffer */
  char * getDateTimeAsString(char * info, bool local = true);
  /**
   * Writes time to INFO in format "hh:mm:ss.msec"
   * \param info destination buffer, must be at least 13 characters long
   * \param local converts time to local time (is system time is UTM or somthing) */
  inline int getTimeAsString(char * info, bool local = true)
    { return GetTimeAsString(info, local);};
  /**
  Get year - valid from 1970 to 2099 only. */
  int GetYear(int * dayInYear = NULL, bool local = true);
  /**
  Get month in range 1..12. */
  int GetMonth(int * dayInMonth = NULL, bool local = true);
  /**
  Get day in year in range 1..366. */
  int GetDayInYear();
  /**
  Get day in month range 1..31. */
  int GetDayInMonth();
  /**
  Get date on format "day month year", e.g. "22 Jun 2003". */
  int GetDateString(char * sDate, bool local = true);
  /**
  Same as above, but with different spelling */
  inline int getDateString(char * sDate, bool local = true)
    { return GetDateString(sDate, local);};
  /**
  Returns trye if 'year' is a leap year.
  (every 4th year is, except every 100th year wich is not, except
   evert 1000th year wich is a leap year). */
  bool isLeapYear(int year);
  /**
  Compare two times */
  inline bool operator==(UTime other)
  {
    bool result;
    if ((time.tv_sec == other.time.tv_sec) and (time.tv_usec == other.time.tv_usec))
      result = true;
    else
      result = false;
    return result;
  };
  /**
  Compare two times */
  inline bool operator> (UTime other)
  {
    bool result;
    if ((time.tv_sec > other.time.tv_sec) or
         ((time.tv_sec == other.time.tv_sec) and (time.tv_usec > other.time.tv_usec)))
      result = true;
    else
      result = false;
    return result;
  };
  /**
  Compare two times */
  inline bool operator>= (UTime other)
  { return not (*this < other); };
  /**
  Compare two times */
  inline bool operator< (UTime other)
  {
    bool result;
    if ((time.tv_sec < other.time.tv_sec) or
         ((time.tv_sec == other.time.tv_sec) and (time.tv_usec < other.time.tv_usec)))
      result = true;
    else
      result = false;
    return result;
  };
  /**
  Compare two times, where other is a double float */
  inline bool operator< (double other)
  {
    return ((getDecSec() - other) < 0.0);
  };
  /**
  Compare two times, where other is a double float */
  inline bool operator> (double other)
  {
    return ((getDecSec() - other) > 0.0);
  };
  /**
  Compare two times, where other is a double float */
  inline bool operator<= (double other)
  {
    return ((getDecSec() - other) <= 0.0);
  };
  /**
  Compare two times, where other is a double float */
  inline bool operator>= (double other)
  {
    return ((getDecSec() - other) >= 0.0);
  };
  /**
  Compare two times */
  inline bool operator<= (UTime other)
  { return not (*this > other); };
  /**
  Compare two times */
  inline bool operator!=(UTime other)
  {
    return not (*this == other);
  };
  /**
  Subtract two UTime values and get result in decimal seconds */
  inline double operator- (UTime old)
  { return GetDecSec(old);};
  /**
  Add a number of seconds to this time */
  UTime operator+ (double seconds);
  /**
  sub a number of seconds to this time */
  UTime operator- (double seconds);
  /**
  Add a number of decimal seconds to this time. */
  inline void operator+= (double seconds)
    { add(seconds); };
  /**
  Add a number of decimal seconds to this time. */
  inline void operator-= (double seconds)
    { sub(seconds); };
  /**
  Add this number of seconds to the current value */
  void add(double seconds);
  /**
  Subtract a number of seconds from this time.
  Can not handle negative time, and seconds must be positive. */
  void sub(double seconds);
  /**
  Convert seconds to time_tm strucure.
  \param when 'local' is true the local time is returned, else GMT.
  \return the structure with year (year 1900 == 0), month, day, hour, min and sec. */
  struct tm getTimeTm(bool local = true);
  /**
  Get copy of timevalue structure */
  inline struct timeval getTimeval()
  {
    return time;
  }
  /**
  Get month number form 3 character string.
  String value must match one of:
  Jan Feb Mas Apr May Jun Jul Aug Sep Oct Nov Dec.
  Returns 0 if no match were found. */
  int getMdrFromString(const char * month3char);
  /**
  Show date and time on console */
  void show(const char * prestring = NULL);
  /**
  print date and time on console */
  inline void print(const char * prestring = NULL)
    { show(prestring); };
  /**
  Code time in XML-like format.
  If 'name' is NULL, then no name attribute is included.
  The tag name is 'time', and format is like:\n
  \< time name="name" sec=107096665 usec=123456/>
  (in seconds and microsec since 1970).
  Returns pointer to provided buffer. */
  char * getAsSml(const char * name, char * buff, int buffCnt);
public:
  /**
  Time as 'timeval' - i.e. same format as in 'timeofday' call. */
  timeval time;
  /**
  A valid flag, that are used when setting the time */
  bool valid;
};

/**
 * Class just as UTime, but the class initializes to creation time.
 * This can be usefull when creation time for application is needed, as
 * constructors are initiated before the main() call. */
class UTimeNow : public UTime
{
public:
  UTimeNow()
  {
    now();
  }
};

#endif
