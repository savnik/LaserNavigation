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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include "utime.h"

/////////////////////////////////////////

UTime::UTime()
{
  clear();
}

/////////////////////////////////////////////

UTime::~UTime()
{
}

/////////////////////////////////////////

void UTime::clear()
{ // clear to zero
  time.tv_sec = 0;
  time.tv_usec = 0;
  valid = false;
}

unsigned long UTime::GetSec()
{
  if (valid)
    return time.tv_sec;
  else
    return 0;
}

/////////////////////////////////////////

double UTime::GetDecSec()
{
  if (valid)
    return double(time.tv_sec) + double(time.tv_usec) * 1e-6;
  else
    return 0;
}

/////////////////////////////////////////

double UTime::GetDecSec(UTime t1)
{ // get time compared to t1
  return double(time.tv_sec - t1.time.tv_sec) + double(time.tv_usec - t1.time.tv_usec) * 1e-6;
}

/////////////////////////////////////////

double UTime::getTimePassed()
{
  UTime t;
  t.Now();
  return (t - *this);
}

/////////////////////////////////////////

long UTime::GetMilisec()
{
  if (valid)
    return time.tv_usec / 1000;
  else
    return 0;
}

///////////////////////////////////////////////

unsigned long UTime::GetMicrosec()
{
  if (valid)
    return time.tv_usec;
  else
    return 0;
}

//////////////////////////////////////////////////
/*
char UTime::GetJustSec()
{
  return char(time.tv_sec % 60);
}
*/
/////////////////////////////////////////////////

char UTime::GetMin()
{
  return char((time.tv_sec / 60) % 60);
}

/////////////////////////////////////////////////

char UTime::GetHour(bool local)
{
  struct tm ymd;
  //
  if (local)
    localtime_r(&time.tv_sec, &ymd);
  else
    gmtime_r(&time.tv_sec, &ymd);
  //
  //return char((time.tv_sec / 3600) % 24);
  return ymd.tm_hour;
}

/////////////////////////////////////////////////////

long UTime::GetDaySerial()
{
  return time.tv_sec / (24 * 60 * 60);
}

/////////////////////////////////////////////////////

int UTime::GetYear(int * dayInYear, bool local)
{
//  long day, days;
//  int year = 1970;
  struct tm ymd;
  //
  if (local)
    localtime_r(&time.tv_sec, &ymd);
  else
    gmtime_r(&time.tv_sec, &ymd);
  //
  /*
  day = GetDaySerial();
  while (day > 0)
  { // forward in time
    if (isLeapYear(year))
      days = 366;
    else
      days = 365;
    if (day >= days)
      // days for another year
      day -= days;
    else
      // this year - stop
      break;
    year++;
  }
  while (day < 0)
  { // back in time
    year--;
    if (isLeapYear(year))
      days = 366;
    else
      days = 365;
    day += days;
  }
  */
  if (dayInYear != NULL)
    *dayInYear = ymd.tm_yday;
  return ymd.tm_year;
}

/////////////////////////////////////////////////////

int UTime::GetDayInYear()
{
  int result;
  GetYear(&result);
  return result;
}

/////////////////////////////////////////////////////

int UTime::GetMonth(int * dayInMonth, bool local)
{
  //int day;
  //int year;
  //int month = 0;
  //int days[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
  //year = GetYear(&day);
  struct tm ymd;
  //
  if (local)
    localtime_r(&time.tv_sec, &ymd);
  else
    gmtime_r(&time.tv_sec, &ymd);
  /*
  // change to zero based day 1 jan is day zero
  day--;
  // leap year
  if (isLeapYear(year))
    days[1] = 29;
  while ((day > days[month]) and (month < 11))
  { // next month
    day -= days[month];
    month++;
  }
  // return also day in month
  */
  if (dayInMonth != NULL)
    *dayInMonth = ymd.tm_mday;
  // return month
  return ymd.tm_mon + 1;
}

/////////////////////////////////////////////////////


int UTime::GetDayInMonth()
{
  int result;
  //
  GetMonth(&result);
  return result;
}

/////////////////////////////////////////////////////
/*
void UTime::Now()
{ // gettime ofday  date sleep
  int err;
  err = gettimeofday(&time, NULL);
  if (err==0)
    valid = true;
  else
    valid = false;
}
*/
/////////////////////////////////////////////

int UTime::GetTimeAsString(char * info, bool local)
{ // writes time to string in format "hh:mm:ss.msec"
  struct tm ymd;
  //
  if (local)
    localtime_r(&time.tv_sec, &ymd);
  else
    gmtime_r(&time.tv_sec, &ymd);
  //
  sprintf(info, "%2d:%02d:%02d.%03d", ymd.tm_hour,
            ymd.tm_min, ymd.tm_sec, (int)GetMilisec());
  return strlen(info);
}

//////////////////////////////////////////

char * UTime::getForFilename(char * info, bool local /*= true*/)
{
  struct tm ymd;
  //
  if (local)
    localtime_r(&time.tv_sec, &ymd);
  else
    gmtime_r(&time.tv_sec, &ymd);
  //
  sprintf(info, "%04d%02d%02d_%02d%02d%02d.%03d",
            ymd.tm_year+1900, ymd.tm_mon+1, ymd.tm_mday,
            ymd.tm_hour,
            ymd.tm_min, ymd.tm_sec, (int)GetMilisec());
  return info;
}

//////////////////////////////////////////

char * UTime::getDateTimeAsString(char * info, bool local /*= true*/)
{
  struct tm ymd;
  //
  if (local)
    localtime_r(&time.tv_sec, &ymd);
  else
    gmtime_r(&time.tv_sec, &ymd);
  //
  sprintf(info, "%04d-%02d-%02d %02d:%02d:%02d.%03d",
          ymd.tm_year+1900, ymd.tm_mon+1, ymd.tm_mday,
          ymd.tm_hour,
          ymd.tm_min, ymd.tm_sec, (int)GetMilisec());
  return info;
}

//////////////////////////////////////////

int UTime::SaveToReg(Uconfig * ini, const char * subject, const char * key)
{
  int n = 0;
  char s[40];
  // save in very long decimal format
  sprintf(s, "%ld.%06ld", GetSec(), GetMicrosec());
  n = ini->strPut(subject, key, s);
  return n;
}


//////////////////////////////////////////
/*
bool UTime::save(UxmlFile * fxml, const char * name)
{
  bool result = (fxml != NULL);
  const int MAX_LNG = 30;
  char s[MAX_LNG];
  //
  if (result)
  {
    result = fxml->saveStartTag("datetime", name);
    if (result)
    {
      getDateString(s);
      fxml->saveStringTag("date", s);
      getTimeAsString(s, true);
      fxml->saveStringTag("time", s);
      result = fxml->saveEndTag("datetime");
    }
  }
  //
  return result;
}
*/
//////////////////////////////////////////
/*
bool UTime::load(UxmlFile * fxml, char * name, const int bufLng)
{
  bool result = (fxml != NULL);
  const int MAX_LNG = 30;
  char nm[MAX_LNG];
  char value[MAX_LNG];
  int year,mdr,day,hour,min,sec,msec;
  int n;
  char * ps;
  bool gotDate, gotTime;
  //
  if (result)
  {
    ps = fxml->getNextTag();
    result = (ps != NULL);
    if (result)
      result = fxml->getNextAttribute(&ps, nm, value, MAX_LNG);
    if (result)
      result = (strcmp(nm, "datetime") == 0);
    if (result and (name != NULL) and (bufLng > 1))
      // decode also name for date-time field and save
      // in provided string
      fxml->getNextAttribute(&ps, nm, name, mini(MAX_LNG, bufLng));
    gotDate = false;
    gotTime = false;
    while (result)
    {
      if (result)
      { // next tag should be a date string     nm[0]
        ps = fxml->getNextTag();
        result = (ps != NULL);
      }
      if (result)
        result = fxml->getNextAttribute(&ps, nm, value, MAX_LNG);
      if (strcmp(nm, "date") == 0)
      {  // get year month and day
        n = sscanf(value, "%d %s %d", &day, nm, &year);
        mdr = getMdrFromString(nm);
        gotDate = ((n == 3) and (mdr > 0));
      }
      else if (strcmp(nm, "time") == 0)
      {  // get year month and day
        n = sscanf(value, "%d:%d:%d.%d", &hour, &min, &sec, &msec);
        gotTime = (n == 4);
      }
      else if (strcmp(nm, "/datetime") == 0)
        break;
    }
    result = (gotTime and gotDate);
    if (result)
      SetTime(year, mdr, day, hour, min, sec, msec * 1000);
  }
  //
  return result;
}
*/
//////////////////////////////////////////

int UTime::getMdrFromString(const char * month3char)
{
  int result = 0;
  const char sm[37] = "JanFebMarAprMayJunJulAugSepOctNovDec";
  int m;
  //
  for (m = 0; m < 12; m++)
  {
    if (strncmp(month3char, &sm[m*3], 3) == 0)
    {
      result = m+1;
      break;
    }
  }
  //
  return result;
}

//////////////////////////////////////////

int UTime::LoadFromReg(Uconfig * ini, const char * subject, const char * key)
{
  int err = 0;
  const char * s = NULL;
  long sec, usec;
  s = ini->strGet(subject, key, s);
  if (s != NULL)
  {
    sscanf(s, "%ld.%ld", &sec, &usec);
    time.tv_sec = sec;
    time.tv_usec = usec;
    valid = true;
  }
  else
  {
    valid = false;
    err = -1;
  }
  return err;
}

//////////////////////////////////////////

void UTime::SetTime(timeval iTime)
{
  time = iTime;
  valid = true;
}

////////////////////////////////////////////

void UTime::setTime(double decimalSec)
{
  time.tv_sec = long(decimalSec);
  time.tv_usec = long((decimalSec - double(time.tv_sec)) * 1e6);
  valid = true;
}

/////////////////////////////////////////

void UTime::setTimeTod(const char * tod)
{
  unsigned long ts, tn;
  int n;
  const char * p1 = tod;
  char * p2;
  //
  ts = strtol(p1, &p2, 10);
  if (p2 > p1)
  {
    if (*p2 == '.')
    { // get also micro second part
      p1 = p2 + 1;
      tn = strtol(p1, &p2, 10);
      // get number of decimal places
      n = p2 - p1;
      // convert to microseconds
      while (n++ < 6)
        tn *= 10;
    }
    else
      tn = 0;
    setTime(ts, tn);
    valid = true;
  }
  else
    valid = false;
}

/////////////////////////////////////////

void UTime::setTimeU(unsigned long sec, unsigned long uSec)
{
  time.tv_sec = sec;
  time.tv_usec = uSec;
  valid = true;
}

/////////////////////////////////////////

void UTime::SetTime(long sec, long uSec)
{
  time.tv_sec = sec;
  time.tv_usec = uSec;
  valid = true;
}

/////////////////////////////////////////

bool UTime::isLeapYear(int year)
{
  return ((year % 4 == 0) and ((year % 100 != 0) or (year % 1000 == 0)));
}

/////////////////////////////////////////

void UTime::setTime(int year, int month, int day, int hour, int min, int sec, long usec)
{
  struct tm ts;
  // tm year is since 1900
  if (year > 1900)
    ts.tm_year = year - 1900;
  else
    ts.tm_year = year;
  ts.tm_mon = month; // range 0..11
  ts.tm_mday = day; // range 1..31
  ts.tm_hour = hour; // range 0..23
  ts.tm_min = min;
  ts.tm_sec = sec;
  time.tv_sec = mktime(&ts);
  // add decimal seconds
  time.tv_usec = usec;
  valid = true;
}

// void UTime::setTime(int year, int month, int day, int hour, int min, int sec, long usec)
// {
//   long d = 0;
//   long s;
//   int y = year;
//   int dy;
//   int m;
//   int days[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
//   // leap year
//   if (isLeapYear(year))
//     days[1] = 29;
//   //
//   while (y > 1970)
//   { // after year zero
//     y--;
//     if (isLeapYear(y))
//       dy = 366;
//     else
//       dy = 365;
//     d += dy;
//   }
//   while (y < 1970)
//   { // before year zero
//     if (isLeapYear(y))
//       dy = 366;
//     else
//       dy = 365;
//     d -= dy;
//     y++;
//   }
//   // then add months
//   if (month <= 12)
//     for (m = 0; m < (month - 1); m++)
//       d += days[m];
//   // then date zero based
//   d += day - 1;
//   s = ((d * 24 + hour) * 60 + min) * 60 + sec;
//   SetTime(s, usec);
// }

/////////////////////////////////////////

int UTime::GetDateString(char * sDate, bool local)
{
//  int year;
//  int month;
//  int date;
  int mi;
  char sm[37] = "JanFebMarAprMayJunJulAugSepOctNovDec";
  struct tm ymd;
  int year;
  //
  if (local)
    localtime_r(&time.tv_sec, &ymd);
  else
    gmtime_r(&time.tv_sec, &ymd);
  year = ymd.tm_year;
  if (year < 500)
    year += 1900;
  //month = GetMonth(&date);
  mi = (ymd.tm_mon) * 3;
  sprintf(sDate,"%d %c%c%c %d", ymd.tm_mday, sm[mi], sm[mi+1], sm[mi+2], year);
  return 0;
}

///////////////////////////////////////// ctime()   time_t  time

struct tm UTime::getTimeTm(bool local)
{
  struct tm ymd;
  //
  if (local)
    localtime_r(&time.tv_sec, &ymd);
  else
    gmtime_r(&time.tv_sec, &ymd);
  //
  return ymd;
}

/////////////////////////////////////////////

void UTime::show(const char * prestring)
{
  char date[30];
  char time[30];
  //
  getTimeAsString(time, true);
  getDateString(date, true);
  if (prestring == NULL)
    printf("%s %s\n", date, time);
  else
    printf("%s %s %s\n", prestring, date, time);
}

/////////////////////////////////////////////

UTime UTime::operator+ (double seconds)
{
  UTime t = *this;
  t.add(seconds);
  return t;
}

/////////////////////////////////////////////

UTime UTime::operator- (double seconds)
{
  UTime t = *this;
  t.sub(seconds);
  return t;
}

/////////////////////////////////////////////

void UTime::add(double seconds)
{
  double result = GetDecSec();
  result += seconds;
  setTime(result);
}

/////////////////////////////////////////////

void UTime::sub(double seconds)
{
  double rem = seconds - long(seconds);
  long remL = (unsigned long)(rem * 1000000.0);
  //
  time.tv_sec -= (unsigned long)seconds;
  if (time.tv_usec < remL)
  { // adjust if overflow
    time.tv_sec--;
    time.tv_usec += 1000000 - remL;
  }
  else
    time.tv_usec -= remL;
}

/////////////////////////////////////////////

char * UTime::getAsSml(const char * name, char * buff, int buffCnt)
{
  if (name != NULL)
    snprintf(buff, buffCnt, "<time name=\"%s\" sec=\"%lu\" usec=\"%lu\"/>",
         name, time.tv_sec, time.tv_usec);
  else
    snprintf(buff, buffCnt, "<time sec=\"%lu\" usec=\"%lu\"/>\n",
         time.tv_sec, time.tv_usec);
  return buff;
}

/////////////////////////////////////////////


