/***************************************************************************
 *   Copyright (C) 2008 by DTU (Christian Andersen)   *
 *   jca@elektro.dtu.dk   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Library General Public License as       *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "ulogfile.h"

////////////////////////////////////////////////////

ULogFile::ULogFile()
{
  logFile = NULL;
  logName[0] = '\0';
  logFileName[0] = '\0';
  logFlush = false;
  logTimeUse = false;
  //shouldBeOpen = false;
  failCnt = 0;
}

////////////////////////////////////////////////////

ULogFile::~ULogFile()
{
  closeLog();
}

////////////////////////////////////////////////////

bool ULogFile::openLog(const char * logName)
{
  setLogName(logName);
  return openLog();
}

////////////////////////////////////////////////////

bool ULogFile::openLog()
{ // open or reopen logfile
  struct stat attrib;                   // create a file attribute structure
  int err, n;
  bool fileExist, isMoved;
  //
  err = stat(logFileName, &attrib);      // get the attributes of old logfile
  fileExist = (err == 0);
  if (fileExist)
    closeLog();
  logFileLock.lock();
  //
  if (fileExist)
  { // try rename the old logfile
    isMoved = logRename(logFileName);
    if (not isMoved)
    { // change the logfile name
      n = 1;
      while (fileExist and n < 100)
      { // find a number to add, that is unused
        setLogNameNumber(n);
        err = stat(logFileName, &attrib);      // get the attributes of old logfile
        fileExist = (err == 0);
        n++;
      }
    }
  }//
  logFile = fopen(logFileName, "w");
  if (logFile == NULL)
  {
    failCnt++;
    if (failCnt < 5)
      printf("ULogFile:: Failed %d to open: %s\n", failCnt, logFileName);
    else if (failCnt == 5)
      printf("ULogFile:: Last report Failed to open: %s\n", logFileName);
  }
  else
    failCnt = 0;
  //shouldBeOpen = true;
  logFileLock.unlock();
  return logFile != NULL;
}

//////////////////////////////////////

bool ULogFile::logRename(const char * name)
{ // make log-rotate here
  struct stat attrib;                   // create a file attribute structure
  struct tm* clock;                     // create a time structure
  const int MFL = 1000;
  char f[MFL];
  int err;
  int y, mdr, d, h, min, n, sec;
  const char * p1;
  //
  //
  err = stat(name, &attrib);      // get the attributes of old logfile
  if (err == 0)
  { // Get the last status change time and put it into the time structure
    //clock = gmtime(&(attrib.st_ctime));
    // Get the last modified time and put it into the time structure
    clock = localtime(&(attrib.st_mtime));
    y   = clock->tm_year + 1900; // returns the year (since 1900)
    mdr = clock->tm_mon + 1; // returns the month (January = 0)
    d   = clock->tm_mday; // returns the day of the month
    h   = clock->tm_hour;
    min = clock->tm_min;
    sec = clock->tm_sec;
    p1 = strrchr(name, '.');
    if (p1 == NULL)
      // name has no extension, so add .logg
      snprintf(f, MFL, "mv %s %s%d%02d%02d_%02d%02d%02d.logg", name, name, y, mdr, d, h, min, sec);
    else
    { // name has an extension, so insert timestamp and add a 'g' at the end
      snprintf(f, MFL, "mv %s ", name);
      n = strlen(f);
      strncat(f, name, MFL - 1);
      n += p1 - name;
      snprintf(&f[n], MFL, "%d%02d%02d_%02d%02d%02d%sg", y, mdr, d, h, min, sec, p1);
    }
    err = system(f);
    if (err == 0)
      printf("Logfile renamed OK: %s\n", f);
    else
      fprintf(stderr,"Logfile rename FAILED: %s\n", f);
  }
  //
  return err == 0;
}


// from http://linux.die.net/man/2/stat
/*
struct tm* clock;                             // create a time structure
struct stat attrib;                     // create a file attribute structure
stat("afile.txt", &attrib);             // get the attributes of afile.txt
clock = gmtime(&(attrib.st_mtime));     // Get the last modified time and put it into the time structure
// clock->tm_year returns the year (since 1900)
// clock->tm_mon returns the month (January = 0)
// clock->tm_mday returns the day of the month*/

////////////////////////////////////////////////////

bool ULogFile::openLog(bool doOpen)
{
  bool result = true;
  if (doOpen)
  { // set logname again, as path may have changed
    setLogName(logName);
    result = openLog();
  }
  else
    closeLog();
  return result;
}

////////////////////////////////////////////////////

void ULogFile::setLogName(const char * resName, const char * ext)
{
  logFileLock.lock();
  memmove(logName, resName, MAX_FILENAME_SIZE);
  snprintf(logFileName, MAX_FILENAME_SIZE, "%s/%s.%s", dataPath, logName, ext);
  logFileLock.unlock();
}

////////////////////////////////////////////////////

void ULogFile::setLogNameNumber(int number)
{
  const char * p1;
  const int MSL = 30;
  char s[MSL] = "";
  //
  // get current extension
  p1 = strrchr(logFileName, '.');
  if (p1 != NULL)
    strncpy(s, ++p1, MSL);
  snprintf(logFileName, MAX_FILENAME_SIZE, "%s/%s_%d.%s",
           dataPath, logName, number, s);
}

////////////////////////////////////////////////////

void ULogFile::closeLog()
{
  int err;
  //
  logFileLock.lock();
  if (logFile != NULL)
  {
    //fflush(logFile);
    err = fclose(logFile);
    logFile = NULL;
    if (err == 0)
      printf("ULogFile closed OK: %s\n", logFileName);
    else
      fprintf(stderr, "ULogFile close FAILED: %s\n", logFileName);
  }
  //shouldBeOpen = false;
  logFileLock.unlock();
}

///////////////////////////////////////////////////////

void ULogFile::toLog(const char * logString)
{
  UTime t;
  char sn[2] = "\n";
  //
/*  if (logFile == NULL and shouldBeOpen)
    openLog();*/
  if (logFile != NULL)
  {
    if (logTimeUse)
      t = logTime;
    else
      t.Now();
    if (logString[strlen(logString)-1] == '\n')
      sn[0]='\0';
    logFileLock.lock();
    fprintf(logFile, "%lu.%06lu %s%s",  t.getSec(), t.getMicrosec(), logString,sn);
    if (logFlush)
      fflush(logFile);
    logFileLock.unlock();
  }
}

///////////////////////////////////////////////////////

void ULogFile::toLog(const char * s1, const char * s2)
{
  UTime t;
  char sn[2] = "\n";
  //
/*  if (logFile == NULL and shouldBeOpen)
    openLog();*/
  if (logFile != NULL)
  {
    if (logTimeUse)
      t = logTime;
    else
      t.Now();
    if (s2[strlen(s2)-1] == '\n')
      sn[0]='\0';
    logFileLock.lock();
    fprintf(logFile, "%lu.%06lu %s %s%s",  t.getSec(), t.getMicrosec(), s1, s2, sn);
    if (logFlush)
      fflush(logFile);
    logFileLock.unlock();
  }
}

///////////////////////////////////////////////////////

void ULogFile::toLog(const int num, const char * s1, const char * s2)
{
  UTime t;
  char sn[2] = "\n";
  //
/*  if (logFile == NULL and shouldBeOpen)
    openLog();*/
  if (logFile != NULL)
  {
    if (logTimeUse)
      t = logTime;
    else
      t.Now();
    if (s2[strlen(s2)-1] == '\n')
      sn[0]='\0';
    logFileLock.lock();
    fprintf(logFile, "%lu.%06lu %d %s %s%s",  t.getSec(), t.getMicrosec(), num, s1, s2, sn);
    if (logFlush)
      fflush(logFile);
    logFileLock.unlock();
  }
}

///////////////////////////////////////////////////////

void ULogFile::toLog(const char * s1, const int num, const char * s2)
{
  UTime t;
  char sn[2] = "\n";
  //
/*  if (logFile == NULL and shouldBeOpen)
    openLog();*/
  if (logFile != NULL)
  {
    if (logTimeUse)
      t = logTime;
    else
      t.Now();
    if (s2[strlen(s2)-1] == '\n')
      sn[0]='\0';
    logFileLock.lock();
    fprintf(logFile, "%lu.%06lu %s %d %s%s",  t.getSec(), t.getMicrosec(), s1, num, s2, sn);
    if (logFlush)
      fflush(logFile);
    logFileLock.unlock();
  }
}

///////////////////////////////////////////////////////

void ULogFile::toLog(const char * s1, const int n1, const int n2, const char * s2)
{
  UTime t;
  char sn[2] = "\n";
  //
/*  if (logFile == NULL and shouldBeOpen)
    openLog();*/
  if (logFile != NULL)
  {
    if (logTimeUse)
      t = logTime;
    else
      t.Now();
    if (s2[strlen(s2)-1] == '\n')
      sn[0]='\0';
    logFileLock.lock();
    fprintf(logFile, "%lu.%06lu %s %d %d %s%s",  t.getSec(), t.getMicrosec(), s1, n1, n2, s2, sn);
    if (logFlush)
      fflush(logFile);
    logFileLock.unlock();
  }
}

////////////////////////////////////////////////////////

void ULogFile::toLog(const char * s1, const int n1, const char * s2, const char * s3)
{
  UTime t;
  char sn[2] = "\n";
  //
/*  if (logFile == NULL and shouldBeOpen)
    openLog();*/
  if (logFile != NULL)
  {
    if (logTimeUse)
      t = logTime;
    else
      t.Now();
    if (s3[strlen(s3)-1] == '\n')
      sn[0]='\0';
    logFileLock.lock();
    fprintf(logFile, "%lu.%06lu %s %d %s %s%s",  t.getSec(), t.getMicrosec(), s1, n1, s2, s3, sn);
    if (logFlush)
      fflush(logFile);
    logFileLock.unlock();
  }
}

/////////////////////////////////////////////////////////

void ULogFile::toLog(const char * s1, const int n1, const int n2, const char * s2, const char * s3)
{
  UTime t;
  char sn[2] = "\n";
  //
/*  if (logFile == NULL and shouldBeOpen)
    openLog();*/
  if (logFile != NULL)
  {
    if (logTimeUse)
      t = logTime;
    else
      t.Now();
    if (s3[strlen(s3)-1] == '\n')
      sn[0]='\0';
    logFileLock.lock();
    fprintf(logFile, "%lu.%06lu %s %d %d %s %s%s",  t.getSec(), t.getMicrosec(), s1, n1, n2, s2, s3, sn);
    if (logFlush)
      fflush(logFile);
    logFileLock.unlock();
  }
}

///////////////////////////////////////////////////////

void ULogFile::doFlush()
{
  logFileLock.lock();
  if (logFile != NULL)
    fflush(logFile);
  logFileLock.unlock();
}

FILE * ULogFile::getF()
{
/*  if (logFile == NULL and shouldBeOpen)
    openLog();*/
  return logFile;
}
