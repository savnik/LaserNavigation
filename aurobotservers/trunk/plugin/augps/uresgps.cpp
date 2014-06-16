/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@elektro.dtu.dk                                                    *
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

#include <termios.h>
#include <stdio.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <ctype.h>

#include <urob4/uvarcalc.h>
#include <ugen4/ucommon.h>

#include "uresgps.h"


/////////////////////////////////////////////

void UGpsStatus::clear()
{
  int i;

   //Internal status struct
  opr_mode = '\0'; //Operational mode
  mode = 0;  //Calculation mode
  for(i = 0; i < GPS_SATELLITES_SUPPORTED; i++)
    satUsed[i] = 0;//Satellites used to make calculation
  for(i = 0; i < GPS_SATELLITES_TOTAL; i++)
  {
    satVisGID[i] = 0; //Number
    satVisElev[i] = 0; //Elevation
    satVisAz[i] = 0; //Azimuth
    satVisSN[i] = 0; //Signal to Noise Ratio
  }
  PDOP = 0.0;  //Positional Dillution of Precision
  HDOP = 0.0;  //Horizontal Dillution of Precision
  VDOP = 0.0;  //Vertical Dillution of precision
  satVisCnt = 0;//Satellites in view
  satVisIdx = 0;
  satUsedCnt = 0;
  // FOM = 0.0; //Precision of the soultion
   //These variables are not updated every second or every 5 seconds.
  EGNOS = 0; //SBAS (EGNOS/WAAS) augmentation indicator
  //zone = 0;  //UTM zone used to specify the base
             //for the coordinate system
}

/////////////////////////////////////////////////

bool UGpsStatus::parseGPGSA(char * inBuf)
{ //   $GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39
  bool isOK = true;
  char *p1, *p2;
  int param = 0, sat;
  //
  // make a copy of the sentence - for debug - monitoring
  strncpy(sentence, inBuf, GPS_SENTENCE_MAX_LENGTH);
  sat = 0;
  p2 = inBuf;
  p1 = strchr(p2, '*');
  if (p1 != NULL)
    // terminate before checksum
    *p1 = '\0';
  lock();
  while (*p2 >= ' ')
  { // separate into substring
    p1 = strsep(&p2, ",");
    switch (param)
    {
    case 0: // type
      if (strcmp(p1, "$GPGSA") != 0)
      { // not my message
        printf("Wring message I know about $GPGSA, not '%s'\n", p1);
        isOK = false;
      }
      break;
    case 1:  //GPS operation mode "A" Automatic "M" manual
      opr_mode = *p1;
      break;
    case 2: //GPS mode "1" No fix "2" 2D fix "3" 3D fix
      mode = atoi(p1);
      break;
    case 15: //Parse PDOP
      PDOP = atof(p1);
      break;
    case 16: //Parse HDOP
      HDOP = atof(p1);
      break;
    case 17: //Parse VDOP
      VDOP = atof(p1);
      break;
    default: //ID(PRN) numbers of the satellites used in the solution
      if ((strlen(p1) > 0) and (param < 15))
      {
        satUsed[sat] = atoi(p1);
        sat++;
      }
      break;
    }
    if (p2 == NULL)
      break;
    param++;
  }
  satUsedCnt = sat;
  unlock();
  return isOK;
}

/////////////////////////////////////////////////////////////

bool UGpsStatus::parseGPGSV(char * inBuf)
{ //   $GPGSV,2,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75
  bool isOK = true;
  char *p1, *p2;
  int param = 0;
  int msg = 0;
  //
  p2 = inBuf;
  p1 = strchr(p2, '*');
  if (p1 != NULL)
    // terminate before checksum
    *p1 = '\0';
  // lock while updating
  lock();
  // make a copy of the sentence - for debug - monitoring
  strncpy(sentence, inBuf, GPS_SENTENCE_MAX_LENGTH);
  while (*p2 >= ' ')
  { // separate into substring
    p1 = strsep(&p2, ",");
    switch (param)
    {
      case 0: // type
        if (strcmp(p1, "$GPGSV") != 0)
        { // not my message
          printf("Wring message I know about $GPGSV, not '%s'\n", p1);
          isOK = false;
        }
        break;
      case 1:  // number of messages of this type
        //msgCnt = strtol(p1, NULL, 10);
        break;
      case 2: // this message is number
        msg = strtol(p1, NULL, 0);
        if (msg == 1) // first message
          satVisIdx = 0;
        break;
      case 3: // number of satellites in view
        satVisCnt = strtol(p1, NULL, 10);
        break;
      default: // satelite data
        if (param < 20)
        {
          switch ((param - 4) % 4)
          { // get data for one satellite
            case 0: // satelite ID number
              satVisGID[satVisIdx] = strtol(p1, NULL, 10);
              break;
            case 1: // Elevation in degrees
              satVisElev[satVisIdx] = strtol(p1, NULL, 10);
              break;
            case 2:
              satVisAz[satVisIdx] = strtol(p1, NULL, 10);
              break;
            case 3:
              satVisSN[satVisIdx] = strtol(p1, NULL, 10);
              satVisIdx++;
              break;
            default:;
              break;
          }
        }
    }
    if (p2 == NULL)
      break;
    param++;
  }
  //
  unlock();
  return isOK;
}

/////////////////////////////////////////////////////////////

int UGpsStatus::getTrueVisCnt()
{
  int i;
  int cnt = 0;
  //
  for (i = 0; i < satVisCnt; i++)
  {
    if (satVisSN[i] > 0)
      cnt++;
  }
  return cnt;
}



/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////

void UGpsUTM::clear()
{
   //Interanal GPS struct
  valid = 0;
  quality = 0;
  satellites = 0;
  dop = 0.0;
  northing = 0.0;
  easting = 0.0;
/*  height = 0.0;
  height2 = 0;*/
}

///////////////////////////////////////////////////////////////

bool UGpsUTM::parseGPLLK(char * inBuf)
{
// $GPLLK,   113616.00,041006,764413.024,M,252946.774,M,3,08,0.010,1171.279,M*12
//           hhmmss.ss ddmmyy,eeeeeeeeee   nnnnnnnnnn,m,mode,sats,dop,height,m
  bool isOK = 1;
  char *p1, *p2;
  int param = 0, len;
  char parse[3][20]={{0},{0}};
  double dTemp;
  //struct tm tod;
  int year,month,day,hour = 0,minu = 0,sec = 0,usec = 0;
  p2 = inBuf;
  p1 = strchr(p2, '*');
  if (p1 != NULL)
    // terminate before checksum
    *p1 = '\0';

  // make a copy of the sentance - for debug - monitoring
  //strncpy(sentance, inBuf, GPS_SENTANCE_MAX_LENGTH);
  while (*p2 >= ' ')
  { // separate into substring
    p1 = strsep(&p2, ",");
    switch (param)
    {
      case 0: // Header type, incl. talker ID
        if (strcmp(p1, "$GPLLK") != 0)
        { // not my message
          printf("Wrong message I know about $GPLLK, not '%s'\n", p1);
          isOK = 0;
          return -1;
        }
        break;
      case 1:  // UTC time of position
            len = sscanf(p1,"%2s%2s%lf",parse[0],parse[1],&dTemp);
        isOK = len == 3;
        if (isOK)
        { // reconstruct date and time (from time only)
          hour = atoi(parse[0]);
          minu = atoi(parse[1]);
          sec = (int) dTemp;
          usec = (int)(((dTemp - (int)dTemp) * 1e6) + 0.5); //Convert to ms
        }
        break;
      case 2:  // UTC date
        len = sscanf(p1,"%2s%2s%2s",parse[0],parse[1],parse[2]);
        if(len == 3)
        {
          day = atoi(parse[0]);
          month = atoi(parse[1]);
          year = atoi(parse[2]) + 2000;
          gmt.SetTime(year, month, day, hour, minu, sec, usec);
        }
        break;
      case 3: // Grid Easting in metres (assume this is signed)
        len = sscanf(p1,"%lf",&easting);
        break;
      case 4: // Units of grid Easting as fixed text M
        break;
      case 5: // Grid Northing in metres (assume this is signed)
        len = sscanf(p1,"%lf",&northing);
        break;
      case 6: // Units of grid Easting as fixed text M
        break;
      case 7: // Position Quality:
                  // 0 = Fix not available or invalid
                          // 1 = No real-time position, navigation fix
                          // 2 = Real-time position, ambiguities not fixed
                          // 3 = Real-time position, ambiguities fixed
        quality = atoi(p1); // mode
        break;
      case 8: // number of satellites
        satellites = atoi(p1);
        break;
      case 9: // GDOP
        dop = strtod(p1,NULL);
        break;
      case 10: // Altitude of position marker above/below mean sea level in metres
        height = strtod(p1,NULL);
        break;
      case 11: // Units of altitude as fixed text M
        break;
      default:
        break;
    }

    if (p2 == NULL)
      break;

    param++;
  }
  return isOK;
}

bool UGpsUTM::parsePTNL(char * inBuf)
{ // decode proparitary message from HAKO RSK-GPS - in UTM
// $PTNL,PJK,123148.30,091807,+6174357.366,N,+707854.368,E,1,07,3.1,EHT+64.037,M*74
// $PTNL,PJK,123148.35,091807,+6174357.360,N,+707854.365,E,1,07,3.1,EHT+64.020,M*7C
// $PTNL,PJK,123148.40,091807,+6174360.926,N,+707856.020,E,2,05,3.9,EHT+66.995,M*7E
// $PTNL,PJK,123148.45,091807,+6174360.925,N,+707856.022,E,2,05,3.9,EHT+67.014,M*7B
// ...
// $PTNL,PJK,123544.45,091807,+6174453.876,N,+707855.593,E,3,09,2.1,EHT+67.223,M*7D
// $PTNL,PJK,123544.50,091807,+6174453.792,N,+707855.610,E,3,09,2.1,EHT+67.225,M*72
// $PTNL,PJK,123544.55,091807,+6174453.676,N,+707855.613,E,3,08,1.6,EHT+67.208,M*75
// $PTNL,PJK,123544.60,091807,+6174453.576,N,+707855.623,E,3,08,1.6,EHT+67.201,M*7A
  bool isOK = true;
  char *p1, *p2;
  int param = 0, len;
  char parse[3][20]={{0},{0}};
  double dsec;
  struct tm t;
  unsigned long sec;
  //
  p2 = inBuf;
  p1 = strchr(p2, '*');
  if (p1 != NULL)
    // terminate before checksum
    *p1 = '\0';
  // lock while updating
  lock();
  // make a copy of the sentence - for debug - monitoring
  strncpy(sentence, inBuf, GPS_SENTENCE_MAX_LENGTH);
  while (*p2 >= ' ')
  { // separate into substring
    p1 = strsep(&p2, ",");
    switch (param)
    {
      case 0: // type
        if (strcmp(p1, "$PTNL") != 0)
        { // not my message
          printf("Wrong message I know about $PTNL, not '%s'\n", p1);
          isOK = false;
        }
        break;
      case 1:  // PJK - sub-message type - there is others like GGK (date,time,position in minutes ...)
        if (strcmp(p1, "PJK") != 0)
        { // not my message subtype
          printf("Wrong message I know about $PTNL,PJK not $PTNL,'%s'\n", p1);
          isOK = false;
        }
        break;
      case 2: // time of day
        len = sscanf(p1,"%2s%2s%lf",parse[0],parse[1],&dsec);
        isOK = len == 3;
        if (isOK)
        { // reconstruct date and time (from time only)
          t.tm_hour = atoi(parse[0]);
          t.tm_min = atoi(parse[1]);
          t.tm_sec = int(dsec);
          dsec -= t.tm_sec;
        }
        break;
      case 3: // date
        len = sscanf(p1,"%2s%2s%2s",parse[0],parse[1],parse[2]);
        if(len == 3)
        {
          t.tm_mday = atoi(parse[0]);
          t.tm_mon = atoi(parse[1]) - 1;
          t.tm_year = atoi(parse[2]) + 100;
          // convert to seconds in this epoc (since 1 jan 1970)
          sec = mktime(&t);
          // set as time type
          gmt.setTime(sec, (unsigned int)(dsec * 1e6));
          // just a rough time check
          isOK = (t.tm_mon < 12 and t.tm_mday > 0 and t.tm_hour < 24);
        }
        break;
      case 4: // northing (assume this is signed)
        northing = strtod(p1, NULL);
        break;
      case 5: // N or S
        break;
      case 6:
        easting = strtod(p1, NULL);
        break;
      case 7: // E or W
        break;
      case 8: // fix mode - assume 1=no fix, 2=floating (or DGPS), 3=FIX
        quality = strtol(p1, NULL, 10);
        break;
      case 9: // number of satellites
        satellites = strtol(p1, NULL, 10);
        break;
      case 10: // DOP
        dop = strtod(p1, NULL);
        break;
      default:
        break;
    }
    if (p2 == NULL)
      break;
    param++;
  }
  //
  unlock();
  return isOK;
}

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////

void UGpsLatLong::clear()
{
  lock();
  valid = false;
//  quality = 0;
//  satellites = 0;
  dop = 99.0;
  gmt.clear(); // time
/*  time_h = 0;
  time_m = 0;
  time_s = 0;
  time_cs = 0;
  date_d = 0;
  date_m = 0;
  date_y = 0;*/
  latDeg = 0.0;
  north_lat = 'N';
  longDeg = 0.0;
  east_lon = 'E';
  speed = 0.0;
  heading = 0.0;
  height = 0.0;
  height2 = 0;
  unlock();
}

///////////////////////////////////////////////////

bool UGpsLatLong::parseGPRMC(char * inBuf, UTime lastTime)
{ // $GPRMC,180432,A,4027.027912,N,08704.857070,W, 000.04,181.9,131000,1.8,W,D*25
  //        hhmmss   ddmm.mmmmmm   dddmm,mmmmmm    knots   deg  ddmmyy, mag  A/D/N
  bool isOK = true;
  //char tmp[20][20]={{0},{0}};
  char parse[3][20]={{0},{0}};
  int len = 0;
  double dsec = 0.0, mm, dd;
  char *p1, *p2;
  int param = 0;
  struct tm t;
  unsigned long sec;
  //
  p2 = inBuf;
  p1 = strchr(p2, '*');
  if (p1 != NULL)
    // terminate before checksum
    *p1 = '\0';
  lock();
  // make a copy of the sentence - for debug - monitoring
  strncpy(sentence, inBuf, GPS_SENTENCE_MAX_LENGTH);
  while (*p2 >= ' ')
  { // get string to next comma
    p1 = strsep(&p2, ",");
    switch (param)
    { // decode message parameters
      case 0: // just the name
        if (strcmp(p1, "$GPRMC") != 0)
        {
          printf("this is not the right message type (expected $GPRMC, got %s\n", p1);
          isOK = false;
        }
        break;
      case 1: // time of day
        len = sscanf(p1,"%2s%2s%s",parse[0],parse[1],parse[2]);
        isOK = len == 3;
        if (isOK)
        { // reconstruct date and time (from time only)
          int hour = atoi(parse[0]);
          t = lastTime.getTimeTm(false);
          if (t.tm_hour == 0 and hour > 12)
          { // the last time is tomorrow add a day
            lastTime -= 3600 * 24; // subtract one day
            t = lastTime.getTimeTm(false);
          }
          else if (t.tm_hour == 23 and hour < 12)
          { // last time is yesterday
            lastTime += 3600 * 24;
            t = lastTime.getTimeTm(false);
          }
          t.tm_hour = hour;
          t.tm_min = atoi(parse[1]);
          dsec = strtod(parse[2], NULL);
          t.tm_sec = int(dsec);
          dsec -= t.tm_sec;
          // convert to seconds in this epoc (since 1 jan 1970)
          sec = mktime(&t);
          // set as time type
          gmt.setTime(sec, (unsigned int)(dsec * 1e6));
        }
        break;
      case 2:  //Convert the validity of the measurement
        if(*p1 == 'V') // warning
          valid = false;
        else if(*p1 == 'A') // valid
          valid = true;
        else
          valid = false;
        break;
      case 3: // Parsse latitude (ddmm.mmmmm)
        sscanf(p1, "%2s%lf", parse[0], &mm);
        dd = atof(parse[0]);
        latDeg = dd + mm/60.0;
        break;
      case 4: // N or S
        north_lat = *p1;
        if (north_lat == 'S')
          latDeg = -latDeg;
        break;
      case 5: // Parse lontitude (dddmm.mmmmm)
        sscanf(p1, "%3s%lf", parse[0], &mm);
        dd = atof(parse[0]);
        longDeg = dd + mm/60.0;
        break;
      case 6: // E or W
        east_lon = *p1;
        if (east_lon == 'W')
          longDeg = -longDeg;
        break;
      case 7: // Convert speed
        speed = atof(p1) * KNOTS2M_S;
        break;
      case 8: // Convert heading
        heading = atof(p1);
        break;
      case 9: //Convert date
        len = sscanf(p1,"%2s%2s%2s",parse[0],parse[1],parse[2]);
        if(len == 3)
        {
          t.tm_mday = atoi(parse[0]);
          t.tm_mon = atoi(parse[1]) - 1;
          t.tm_year = atoi(parse[2]);
          if (t.tm_year < 70)
            // should be since year 1900
            t.tm_year += 100;
            // convert to seconds in this epoc (since 1 jan 1970)
          sec = mktime(&t);
          // set as time type
          gmt.setTime(sec, (unsigned int)(dsec * 1e6));
          isOK = (t.tm_mon < 12 and t.tm_mday > 0 and t.tm_hour < 24);
        }
        break;
      default: // ignore the rest
        break;
    }
    if (p2 == NULL)
      break;
    param++;
  }
  // release data
  unlock();
  return isOK;
}

/////////////////////////////////////////////////////////////

bool UGpsLatLong::parseGPGGA(char * inBuf, UTime lastTime)
{ // $GPGGA,180432.00,4027.027912,N,08704.857070, W,2,07,1.0,212.15,M,-33.81,M,4.2,0555*73
  //        hhmmss.ss ddmm.mmmmmm   dddmm.mmmmmm     sat hdop hgt[m]   (DGPS base)
  bool isOK = false;
  char parse[3][20]={{0},{0}};
  int len;
  //static int GSV_old=0;
  int hour;
  double dsec, dd, mm;
  char * p1, *p2;
  int param = 0;
  struct tm t;
  unsigned long sec;
  //
  p2 = inBuf;
  p1 = strchr(p2, '*');
  if (p1 != NULL)
    // terminate before checksum
    *p1 = '\0';
  lock();
  // make a copy of the sentence - for debug - monitoring
  strncpy(sentence, inBuf, GPS_SENTENCE_MAX_LENGTH);
  while (*p2 >= ' ')
  {
    p1 = strsep(&p2, ",");
    switch(param)
    { // decode parameters
      case 0: // just $GPGGA
        if (strcmp(p1, "$GPGGA") != 0)
        {
          printf("this is not the right message type (expected $GPGGA, got %s\n", p1);
          isOK = false;
        }
        break;
      case 1: // time hhmmss.ss
        //Convert the time
        len = sscanf(p1,"%2s%2s%s",parse[0],parse[1],parse[2]);
        if (len == 3)
        {
          UTime tn;
          tn.now();
          t = tn.getTimeTm(false);
          t = lastTime.getTimeTm(false);
          hour = atoi(parse[0]);
          if (t.tm_hour == 0 and hour > 12)
          { // the last time is tomorrow add a day
            lastTime -= 3600 * 24; // subtract one day
            t = lastTime.getTimeTm(false);
          }
          else if (t.tm_hour == 23 and hour < 12)
          { // last time is yesterday
            lastTime += 3600 * 24;
            t = lastTime.getTimeTm(false);
          }
          t.tm_hour = hour;
          t.tm_min = atoi(parse[1]);
          dsec = strtod(parse[2], NULL);
          t.tm_sec = int(dsec);
          dsec -= t.tm_sec;
          // convert to seconds in this epoc (since 1 jan 1970)
          sec = mktime(&t);
          // set as time type
          gmt.setTime(sec, (unsigned int)(dsec * 1e6));
        }
        break;
      case 2: // Parsse latitude
        sscanf(p1, "%2s%lf", parse[0], &mm);
        dd = atof(parse[0]);
        latDeg = dd + mm/60.0;
        break;
      case 3: // N or S
        north_lat = *p1;
        if (north_lat == 'S')
          latDeg = -latDeg;
        break;
      case 4: // Parse lontitude
        sscanf(p1, "%3s%lf", parse[0], &mm);
        dd = atof(parse[0]);
        longDeg = dd + mm/60.0;
        break;
      case 5: // E or W
        east_lon = *p1;
        if (east_lon == 'W')
          longDeg = -longDeg;
        break;
      case 6: // Parse quality
        quality = atoi(p1);
        valid = (quality == 1 or quality == 2);
        egnos = (quality == 2);
        break;
      case 7: // Parse number of satellites
        satellites = atoi(p1);
        break;
      case 8: // Parse HDOP
        dop = atof(p1);
        isOK = true;
        break;
      case 9: // Antenna height
        height = atoi(p1);
        break;
      case 10: //Antenna height difference
        height2 = atoi(p1);
        break;
      default:
        ; // ignore the rest
        break;
    }
    if (p2 == NULL)
      break;
    param++;
  }
  unlock();
  return isOK;
}

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////

void UResGps::clear()
{
  utm.clear();
  latLong.clear();
  status.clear();
}

/////////////////////////////////////////////////////////////

bool UResGps::parseNMEA(char* inBuf, UTime tod)
{
  bool isOK;
  char * p1;
   //Find the string format
  isOK = validateNMEA(inBuf);
  //strip off linefeed
  p1 = &inBuf[strlen(inBuf)-1];
  while (*p1 < ' ' and p1 > inBuf)
    *p1-- = '\0';
  //
  if (not isOK)
    printf("Failed to validate the GPS message: '%s'\n", inBuf);
  // make a copy of the sentence - for debug - monitoring
  strncpy(sentence, inBuf, GPS_SENTENCE_MAX_LENGTH);
  if (isOK)
  { //Recommended Minimum Specific GNSS information relayed by all satellites
    if(strncmp(inBuf,"$GPRMC", 6) == 0)
    { // $GPRMC,180432,A,4027.027912,N,08704.857070,W, 000.04,181.9,131000,1.8,W,D*25
      //        hhmmss   ddmm.mmmmmm   dddmm,mmmmmm    knots   deg  ddmmyy, mag  A/D/N
      varGPRMC->setValues(inBuf, 0, true);
      if (latLong.parseGPRMC(inBuf, lastTime))
      {
        lastTime = latLong.gmt;
        latLongUpdated = true;
        msgTime = tod;
      }
    }
    //GPS Fix data
    else if(strncmp(inBuf,"$GPGGA", 6) == 0)
    { // $GPGGA,180432.00,4027.027912,N,08704.857070, W,2,07,1.0,212.15,M,-33.81,M,4.2,0555*73
      //        hhmmss.ss ddmm.mmmmmm   dddmm.mmmmmm     sat hdop hgt[m]   (DGPS base)
      varGPGGA->setValues(inBuf, 0, true);
      if (latLong.parseGPGGA(inBuf, lastTime))
      {
        lastTime = latLong.gmt;
        latLongUpdated = true;
        msgTime = tod;
      }
    }
    //Geografic position Lat/Lon
    //Redundant information
    else if(strncmp(inBuf,"$GPGLL", 6) == 0)
    {
      varGPGLL->setValues(inBuf, 0, true);
      //printf("$GPGLL - not supported\n");
      isOK = false;
    }
    //GNSS Dillution Of Presition and Active Satellites
    else if(strncmp(inBuf,"$GPGSA", 6) == 0)
    {
      varGPGSA->setValues(inBuf, 0, true);
      if (status.parseGPGSA(inBuf))
        statusUpdated = true;
    }
    //GNSS Satellites in view
    //There has not been done any handling on missing GSV messages
    //GSV_new and GSV_old are for this purpose
    else if(strncmp(inBuf,"$GPGSV", 6) == 0)
    {
      varGPGSV->setValues(inBuf, 0, true);
      if (status.parseGPGSV(inBuf))
        statusUpdated = true;
    }
    //Course over Ground and Ground Speed
    //Redundant information
    else if(strncmp(inBuf,"$GPVTG", 6) == 0)
    {
      varGPVTG->setValues(inBuf, 0, true);
      //printf("$GPVTG - not supported\n");
      isOK = false;
    }
    //Position figure of merit
    else if(strncmp(inBuf,"$PFST", 5) == 0)
    {
      printf("$PFST - proparity sentence not supported\n");
      isOK = false;
    }
    //Proparitary SiRF message
    else if(strncmp(inBuf,"$PSRF151", 8) == 0)
    {
      if(true)
      {
        printf("%s\n", inBuf);
      }
      p1 = strchr(inBuf, ',');
      status.EGNOS = strtol(p1, NULL, 10);
      //       if(PRINT_INFO)
      printf("EGNOS status %i\n",status.EGNOS);
    }
    else if (strncmp(inBuf, "$PTNL", 5) == 0)
    {
      varPTNL->setValues(inBuf, 0, true);
      if (utm.parsePTNL(inBuf))
      {
        utmUpdated = true;
        msgTime = tod;
      }
    }
    else if (strncmp(inBuf, "$GPLLK", 6) == 0)
    {
      varPTNL->setValues(inBuf, 0, true);
      if (utm.parseGPLLK(inBuf))
      {
        utmUpdated = true;
        msgTime = tod;
      }
    }
    else
      printf("Data error, tag not known\n");
    //
  }
  //
  return isOK;
}

/////////////////////////////////////////////////////

bool UResGps::validateNMEA(char* inBuf)
{
  char tmp = 0;
  int i = 2;
  bool result;
  //
  tmp = inBuf[1];
  if(inBuf[0] == '$' && isalpha(inBuf[1]))
  {
    do
    {
      tmp = tmp ^ inBuf[i];
      i++;
    } while((inBuf[i] != '*') and (inBuf[i] >= ' '));
  }
  //
  if((tmp >> 4) == ascii2hex(inBuf[i + 1]) and (tmp & 0x0f) == ascii2hex(inBuf[i + 2]))
    result = true;
  else
    result = false;

  return result;
}

//////////////////////////////////////////////////

UTime UResGps::getTimeLocal()
{
  UTime t;
  struct tm ts;
  // get time assuming lastTime is GMT
  ts = lastTime.getTimeTm(true);
  // set time using this data
  t.setTime(ts.tm_year, ts.tm_mon, ts.tm_mday, ts.tm_hour, ts.tm_min, ts.tm_sec, lastTime.getMicrosec());
  return t;
}

/////////////////////////////////////////////////////

char UResGps::ascii2hex(char input)
{
  if(isalpha(input))
  {
    if(isupper(input))
      return input - 'A' + 10;
    else
      return input - 'a' + 10;
  }
  else
    return input - '0';
}

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////

void UResGps::UResGpsInit()
{ // these first two lines is needed
  // to save the ID and version number
  latLongUpdated = false;
  utmUpdated = false;
  statusUpdated = false;
  lastTime.Now();
  //
  logGps.setLogName("noname");
  dataRxMsgCnt = 0;
  replaySentenceNew = false;
  //
  //strncpy(replaySentance, "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\n\r", GPS_SENTENCE_MAX_LENGTH);
  //strncpy(replaySentance, "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\n\r", GPS_SENTENCE_MAX_LENGTH);
  //strncpy(replaySentance, "$GPGSV,2,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75\n\r", GPS_SENTENCE_MAX_LENGTH);
  //strncpy(replaySentance, "$GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39\n\r", GPS_SENTENCE_MAX_LENGTH);
  //strncpy(replaySentance, "$PTNL,PJK,123544.60,091807,+6174453.576,N,+707855.623,E,3,08,1.6,EHT+67.201,M*7A\n\r", GPS_SENTENCE_MAX_LENGTH);
  //
  utmPose = NULL;
  mapPose = NULL;
  // other local initializations
  createBaseVar();
  // start read thread
  // degug
  // log2.openLog("gpsDebug");
  // debug end
  start();
}

///////////////////////////////////////////

UResGps::~UResGps()
{
  // stop read thread
  stop();
  // close serial port to device
  closePort();
  log2.closeLog();
}

///////////////////////////////////////////

const char * UResGps::print(const char * preString, char * buff, int buffCnt)
{ // print status for resource (short 1-3 lines of text followed by a line feed (\n)
  int n = 0;
  char * p1 = buff;
  const int MSL = 50;
  char s[MSL];
  //
  snprintf(p1, buffCnt - n, "%s Connected (%s) to %s, log '%s' open (%s)\n", preString,
          bool2str(isConnected()), varDevice->getValues(), logGpsName, bool2str(logGps.isOpen()));
  n += strlen(p1);
  p1 = &buff[n];
  lastTime.getTimeAsString(s, true);
  snprintf(p1, buffCnt - n, "    Last update at %s was\n       '%s'\n", s, sentence);
  return buff;
}

///////////////////////////////////////////

void UResGps::createBaseVar()
{
  varDevice =      addVar("device", "/dev/ttyUSB0", "s", "(rw) device file name");
  varBaud =        addVar("baud", 9600.0, "d", "(rw) device baudrate (set to -1 to leave unchanged)");
  varOpen =        addVar("open", 0.0, "d", "(rw) should the device be open or not (1=open, 0=close)");
  varIsOpen =      addVar("isOpen", 0.0, "d", "(r) is the device be open (1=open 0=closed)");
  varUTMZone  =    addVar("UTMzone", 32, "d", "(rw) Projection zone for Lat-long to UTM conversion");
  varSatsVisible = addVar("satsVisible", 0.0, "d", "(r) Number of visible satellites");
  varSatsUsed = addVar("satsUsed", 0.0, "d", "(r) Number of used satelites in calculation");
  varSatsUsed->makeTimeSeries(10000, 0.2);
  varDop      = addVar("dop", 0.0, "d", "(r) Dilution of position 1 is perfect, 5 is fair, >8 is bad");
  varDop->makeTimeSeries(10000, 0.2);
  varFixRate  = addVar("fixRate", 0.0, "d", "(r) Number of fixes per second");
  varLat      = addVar("lat", 0.0, "d", "(r) Latitude in decimal degrees, positive is North");
  varLat->makeTimeSeries(10000, 0.2);
  varLong     = addVar("long", 0.0, "d", "(r) Longitude in decimal degrees, positive is East");
  varLong->makeTimeSeries(10000, 0.2);
  varHeight   = addVar("height", 0.0, "d", "(r) Height in meters, (geoide)");
  varTime     = addVar("Time", 0.0, "d", "(r) Last fix time (computer time)");
  varGmt      = addVar("Gmtt", 0.0, "d", "(r) Last received GMT time");
  varTimes     = addVar("TimeStr", "none", "s", "(r) Last fix time (computer time)");
  varGmts      = addVar("GmttStr", "none", "s", "(r) Last received GMT time");
  varHeading  = addVar("heading", 0.0, "d", "(r) Heading received from GPS (compas degrees)");
  varHeading->makeTimeSeries(10000, 1);
  varSpeed  =   addVar("speed", 0.0, "d", "(r) Speed received from GPS (m/sec)");
  varSpeed->makeTimeSeries(10000, 1.0);
  varEasting  = addVar("easting", 0.0, "d", "(r) UTM Easting in current zone in meter");
  varNorthing = addVar("northing", 0.0, "d", "(r) UTM northing in current zone in meter");
  varMakeUTM  = addVar("makeUTM", 1.0, "d", "(rw) Do conversion to UTM (when source is Lat-Long)");
  varUtmPoseKeep  = addVar("UTMPoseKeep", 1.0, "d", "(rw) set UTM pose to utmPose module");
  varMapPoseKeep  = addVar("mapPoseKeep", 1.0, "d", "(rw) set UTM pose to mapPose module");
  varUseOdoHeading = addVar("useOdoHeading", 1.0, "d", "(rw) use odometry history to estimate heading");
  varMapPoseRef  = addVar("mapPoseRef", "0 0", "d", "(rw) set mapPose relative to this UTM position (east, north) in meters");
  varEgnos    = addVar("egnos", 0.0, "d", "(r) Is EGNOS/WAAS on");
  varMode     = addVar("mode", 1.0, "d", "(r) GPS operation mode '1' No fix '2' 2D fix '3' 3D fix, "
                                            "or 1=float, 2=DGPS, 3=RSK-FIX");
  createReplayVar(getVarPool());
//  varReplay   =  addVar("replay", 0.0, "d", "(rw) Is function to listen to replay or device data");
  varUpdCntLL =  addVar("updateCntLL", 0.0, "d", "(r) number of Lat Long updates received");
  varUpdCntUTM = addVar("updateCntUTM", 0.0, "d", "(r) number of UTM updates received");
    //
  varGPVTG = addVar("GPVTG", "none", "s", "(r) Course over ground (not used)");
  varGPGLL = addVar("GPGLL", "none", "s", "(r) Position data (not used)");
  varGPGSA = addVar("GPGSA", "none", "s", "(r) DOP and sats in view");
  varGPGSV = addVar("GPGSV", "none", "s", "(r) Satellites in view");
  varGPRMC = addVar("GPRMC", "none", "s", "(r) Minimum spec message");
  varGPGGA = addVar("GPGGA", "none", "s", "(r) Fix data");
  varPTNL =  addVar("PTNL", "none", "s", "(r) Trimple position message");
}


//////////////////////////////////////////////


bool UResGps::setResource(UResBase * resource, bool remove)
{
  bool result = true;

  if (resource->isA(UResPoseHist::getUtmPoseID()))
  { // resource may change
    if (remove)
      utmPose = NULL;
    else if (utmPose != resource)
      utmPose = (UResPoseHist *)resource;
    else
      result = false;
  }
  if (resource->isA(UResPoseHist::getOdoPoseID()))
  { // resource may change
    if (remove)
      odoPose = NULL;
    else if (odoPose != resource)
      odoPose = (UResPoseHist *)resource;
    else
      result = false;
  }
  else if (resource->isA(UResPoseHist::getMapPoseID()))
  { // resource may change
    if (remove)
      mapPose = NULL;
    else if (mapPose != resource)
      mapPose = (UResPoseHist *)resource;
    else
      result = false;
  }
  result &= UResVarPool::setResource(resource, remove);
  return result;
}

////////////////////////////////////////////////////////////////////////

bool UResGps::isConnected()
{
  return (varIsOpen->getBool());
}


////////////////////////////////////////////////////////////////////

void UResGps::toLog(const char * message)
{
  UTime t;
  //
  if (logGps.isOpen())
  {
    t.now();
    if (message[strlen(message) -1] < ' ')
      fprintf(logGps.getF(), "%lu.%06lu %s", t.getSec(), t.getMicrosec(), message);
    else
      fprintf(logGps.getF(), "%lu.%06lu %s\n", t.getSec(), t.getMicrosec(), message);
  }
}

/////////////////////////////////////////////////////////////////////

void UResGps::run()
{
  const int MRXB = 1000;
  char rxBuff[MRXB + 1];
  bool isOK;
  UTime tod;
  FILE * gpsDev = NULL;
  int failCnt = 0;
  int rxBuffCnt = 0;
  const double rxTimeout = 0.030;
  //
  threadRunning = true;
  // wait to allow init script to finish
  Wait(1.2);
  while (not threadStop)
  {
    if (varOpen->getBool() and not varIsOpen->getBool())
    { // device is to be open - try
      printf("UResGps::run: Trying to open %s for device %s\n", varDevice->getValues(), getResID());
      if (varBaud->getInt() > 0)
      {
        setDeviceSpeed(varDevice->getValues(), varBaud->getInt());
        Wait(0.3);
      }
      gpsDev = fopen(varDevice->getValues(), "r");
      if (gpsDev == NULL)
      { // file not found
        if (failCnt < 5)
        {
          toLog("# Device file not found (readable)");
          printf("UFuncGps::run: GPS device %s could not be opened\n", varDevice->getValues());
          failCnt++;
        }
      }
      else
      {
        varIsOpen->setBool(true, 0);
        failCnt = 0;
        rxBuffCnt = 0;
        rxBuff[0] = '\0';
      }
    }
    else if (not varOpen->getBool() and varIsOpen->getBool())
    { // device is to be closed
      printf("UResGps::run: Trying to close %s for device %s\n", varDevice->getValues(), getResID());
      fclose(gpsDev);
      gpsDev = NULL;
      varIsOpen->setBool(false, 0);
    }
    if (replay)
    { // we are in replay mode, so listen to new NMEA sentences from replay
      if (replaySentenceNew and replaySentenceLock.lock())
      {
        char * p1 = replaySentence;
        tod.setTimeTod(p1);
        if (not tod.valid)
          // time is not valid, so time-stamp is probably missing
          tod.Now();
        else
          // timestamp OK, so advance to after timestamp - space separated.
          p1 = strchr(p1, ' ');
        // advance past the white space
        while (isspace(*p1))
          p1++;
        // parse the NMEA sentence
        isOK = parseNMEA(p1, tod);
        // mark sentence as used
        replaySentenceNew = false;
        replaySentenceLock.unlock();
      }
      else
      { // just wait a bit
        isOK = false;
        Wait(0.05);
      }
      if (isOK)
            // update global variables - etc
        updateVars();
    }
    if (varIsOpen->getBool())
    { // is still open, so get data
      char * pEnd = rxBuff;
      //printf("UResGps::run: Trying to read from %s for device %s\n", varDevice->getValues(), getResID());
      /// replace with non-blocking - see below
      //p1 = fgets(rxBuff, MRXB, gpsDev);
      //printf("UResGps::run: read from %s: %s", varDevice->getValues(), rxBuff);
      if (rxBuffCnt > 1 and rxBuff[0] != '$')
        // start of buffer is not start of message, so advance
        rxBuffCnt = advanceNmeaMessage(rxBuff, rxBuffCnt, rxBuff);
      if (not hasNmeaMessage(rxBuff, rxBuffCnt, NULL))
      { // get new data from device
        int n;
        n = receiveFromDevice(gpsDev, &rxBuff[rxBuffCnt], MRXB - rxBuffCnt, rxTimeout);
        if (n > 0)
          rxBuffCnt+= n;
      }
      if (hasNmeaMessage(rxBuff, rxBuffCnt, &pEnd))
      {
        toLog(rxBuff);
          isOK = parseNMEA(rxBuff, dataRxTime);
        if (isOK)
        { // update global variables - etc
          //printf("Parsing is OK - updating vars\n");
          updateVars();
        }
        // advance one message
        rxBuffCnt = advanceNmeaMessage(rxBuff, rxBuffCnt, pEnd);
      }
    }
    if (not varIsOpen->getBool())
      Wait(1.0);
  }
  threadRunning = false;
}

///////////////////////////////////////////////////

bool UResGps::hasNmeaMessage(char * rxBuff, int rxBuffCnt, char ** pEnd)
{
  char * p1 = rxBuff;
  bool hasNmea = false;
  //
  if (*p1 == '$')
  {
    p1 = strchr(p1, '*');
    if (p1 != NULL)
    {
      int m = p1 - rxBuff;
      if ((rxBuffCnt - m) > 2)
        hasNmea = true;
      p1 += 3;
      *p1 = '\0';
      if (pEnd != NULL)
        *pEnd = p1 + 1;
    }
  }
  return hasNmea;
}

///////////////////////////////////////////////////

int UResGps::advanceNmeaMessage(char * rxBuff, int rxBuffCnt, char * pEnd)
{
  char * p1 = pEnd;
  int newBufCnt = 0;
  //
  p1 = strchr(p1, '$');
  if (p1 != NULL)
  { // there is (start of) next message
    newBufCnt = rxBuffCnt - (p1 - rxBuff);
    if (newBufCnt > 0)
      memmove(rxBuff, p1, newBufCnt);
    else
      newBufCnt = 0;
  }
  // make sure buffer is 0 terminated
  rxBuff[newBufCnt] = '\0';
  // return new length
  return newBufCnt;
}

///////////////////////////////////////////////////

void * threadRunGps(void * obj)
{ // call the hadling function in provided object
  UResGps * ce = (UResGps *)obj;
  ce->run();
  pthread_exit((void*)NULL);
  return NULL;
}

///////////////////////////////////////////////////

bool UResGps::start()
{
  bool result = true;
  pthread_attr_t  thAttr;
  //
  if (not threadRunning)
  {
    pthread_attr_init(&thAttr);
    //
    threadStop = false;
    // create socket server thread
    result = (pthread_create(&threadHandle, &thAttr,
              &threadRunGps, (void *)this) == 0);
    pthread_attr_destroy(&thAttr);
  }
  return result;
}

///////////////////////////////////////////////////

void UResGps::stop()
{
  if (threadRunning)
  { // stop and join thread
    threadStop = true;
    pthread_join(threadHandle, NULL);
  }
}

///////////////////////////////////////////////////

int UResGps::receiveFromDevice(FILE * device,
                               char * start, // put data here
                               int maxLng,   // max number of characters
                               const double timeoutSec) // timeout in seconds
{ // poll wait time
  int pollTime = roundi(timeoutSec * 1000.0); // ms
  int n, m = maxLng;
  struct pollfd ps;
  // Set poll structure for test of indata available
  ps.fd = device->_fileno;
  ps.events = POLLIN;
  bool timeout;
  int result = 0; // number of characters received
  char * bp = start; // put new data here
  char * p1;
  //
  // receive a number of data that should include a header
  /* get new data */
  timeout = false;
  while ((m > 0) and not timeout)
  { // Wait for data in up to the timeout period
    if (poll(&ps, 1, pollTime) != POLLIN)
      // timeout or other error - return
      timeout = true;
    else
    { /* data available, read up to a full message */
      n = read(device->_fileno, bp, m);
      if (n == -1)
      { // error
        perror("Error in read from serial line");
        result = -1;
        break;
      }
      else
      { //
        if (result == 0)
        { // first data - timestamp
          dataRxTime.Now();
        }
        bp[n] = '\0';
        p1 = strchr(bp, '\r');
        if (p1 == NULL)
        {
          p1 = bp-1;
          // wait for approx 50 characters
          Wait(600.0/varBaud->getDouble());
        }
        //printf("len=%d, CR=%d n=%d, m=%d bp='%s'\n", strlen(bp), p1 - bp, n, m, bp);
        // terminate string
        m -= n;
        result += n;
        // stop if a newline is received
        if (strchr(bp, '\n') != NULL)
          // a full message is received (at least one).
          break;
        bp = &start[result];
      }
    }
  }
  //
  return result;
}

/////////////////////////////////////////////////

void UResGps::updateVars()
{
  double east, north, lat, lon;
  int zone;
  double heading;
  UPoseTVQ pt;
  bool utmPoseNew = false;
  const int MSL = 50;
  char s[MSL];
  //
  getVarPool()->lock();
  if (latLongUpdated)
  {
    latLong.lock();
    varLat->setDouble(latLong.latDeg);
    varLong->setDouble(latLong.longDeg);
    varHeight->setDouble(latLong.height);
    varTime->setTime(msgTime);
    varGmt->setTime(latLong.gmt);
    varTime->getTime(0).getDateTimeAsString(s);
    varTimes->setValues(s, 0, true);
    varGmt->getTime(0).getDateTimeAsString(s);
    varGmts->setValues(s, 0, true);
    varEgnos->setBool(latLong.egnos);
    varDop->setDouble(latLong.dop, 0);
    varUpdCntLL->add(1.0, 0);
    varHeading->setDouble(latLong.heading, 0);
    varSpeed->setDouble(latLong.speed, 0);
    latLongUpdated = false;
    latLong.unlock();
    if (varMakeUTM->getBool() or
        varMapPoseKeep->getBool() or
        varUtmPoseKeep->getBool())
    { // convert Lat Long to UTM in the desired zone
      zone = varUTMZone->getInt();
      // convert
      latlon2UTM(latLong.latDeg, latLong.longDeg, zone, &east, &north);
      // save result
      varEasting->setDouble(east, 0);
      varNorthing->setDouble(north, 0);
      utmPoseNew = true;
      // debug - convert back
/*      gpsData.utm2latlon(east, north, zone, &lat, &lon);
      printf("From %f,%f to %fE,%fN, to %f,%fdeg\n",
             gpsData.latLong.latDeg, gpsData.latLong.longDeg,
             east, north, lat, lon);*/
      // dbeug end
      // debug log
      if (log2.isOpen())
      {
        fprintf(log2.getF(), "%ld.%06ld %.6f %.6f %.2f %.2f %.1f %.4f %d %d %.2f %.2f\n",
                msgTime.getSec(), msgTime.getMicrosec(),
                latLong.latDeg, latLong.longDeg, east, north, latLong.height,
                latLong.dop, varSatsUsed->getInt(0), varMode->getInt(0), latLong.speed, latLong.heading
               );
      }
      //
    }
  }
  if (utmUpdated)
  {
    utm.lock();
    varEasting->setDouble(utm.easting, 0);
    varNorthing->setDouble(utm.northing, 0);
    varTime->setTime(msgTime);
    varGmt->setTime(utm.gmt);
    varTime->getTime(0).getDateTimeAsString(s);
    varTimes->setValues(s, 0, true);
    varGmt->getTime(0).getDateTimeAsString(s);
    varGmts->setValues(s, 0, true);
    varDop->setDouble(utm.dop, 0);
    varMode->setDouble(utm.quality, 0);
    varSatsUsed->setInt(utm.satellites, 0);
    varUpdCntUTM->add(1, 0);
    utmUpdated = false;
    // update utm pose history
    utmPoseNew = true;
    if (varMakeUTM->getBool())
    {
      zone = varUTMZone->getInt();
      // convert
      east = utm.easting;
      north = utm.northing;
      utm2latlon(east, north, zone, &lat, &lon);
      // save result
      varLat->setDouble(lat, 0);
      varLong->setDouble(lon, 0);
      // debug
/*      gpsData.latlon2UTM(lat, lon, zone, &east, &north);
      printf("From %fE,%fN, to %f,%f to %fE,%fN\n",
             gpsData.utm.easting, gpsData.utm.northing,
             lat, lon, east, north);*/
      // dbeug end
    }
    utm.unlock();
  }
  if (utmPoseNew)
  { // update utm/map pose history
    // convert heading to math angle in radians
    // default
    heading = (90.0 - latLong.heading) * M_PI / 180.0;
    if (varUseOdoHeading->getBool() and odoPose != NULL and
        (varUtmPoseKeep->getBool() or varMapPoseKeep->getBool()))
    { // try to find a better heading
      UPoseTVQ pOdo1;
      UPoseTime ptOdo;
      double dh;
      ptOdo = odoPose->getNewest();
      if (odoPose->getPoseNearDistance(20.0, &ptOdo, &pOdo1, NULL))
      {
        dh = limitToPi(ptOdo.h - atan2(ptOdo.y - pOdo1.y, ptOdo.x - pOdo1.x));
        if (varUtmPoseKeep->getBool() and utmPose != NULL)
        { // use utmPose history
          UPoseTVQ pUtm1;
          UPoseTime ptUtm;
          double h;
          ptUtm = utmPose->getNewest();
          if (utmPose->getPoseNearDistance(20.0, &ptUtm, &pUtm1, NULL))
          { // result pose is valid
            h = atan2(ptUtm.y - pUtm1.y, ptUtm.x - pUtm1.x);
            heading = limitToPi(h + dh);
          }
        }
        else if (varMapPoseKeep->getBool() and mapPose != NULL)
        { // use map pose history
          UPoseTVQ pMap1;
          UPoseTime ptMap;
          double h;
          ptMap = mapPose->getNewest();
          if (mapPose->getPoseNearDistance(20.0, &ptMap, &pMap1, NULL))
          { // result pose is valid
            h = atan2(ptMap.y - pMap1.y, ptMap.x - pMap1.x);
            heading = limitToPi(h + dh);
          }
        }
      }
    }
    if (utmPose != NULL and varUtmPoseKeep->getBool())
    {  // set pose
      pt.set(east, north, limitToPi(heading),
             msgTime,
             latLong.speed, latLong.dop);
      // update map pose
      utmPose->addIfNeeded(pt, -8);
    }
    if (mapPose != NULL and varMapPoseKeep->getBool())
    { // convert heading to math angle in radians
      // set pose
      pt.set(east - varMapPoseRef->getDouble(0),
              north - varMapPoseRef->getDouble(1),
              limitToPi(heading),
              msgTime,
              latLong.speed, latLong.dop);
      // update map pose
      mapPose->addIfNeeded(pt, -8);
    }
  }
  if (statusUpdated)
  {
    status.lock();
    varSatsVisible->setInt(status.getTrueVisCnt(), 0);
    varSatsUsed->setInt(status.satUsedCnt, 0);
    varEgnos->setInt(status.EGNOS, 0);
    varDop->setDouble(status.HDOP, 0);
    varMode->setInt(status.mode, 0);
    statusUpdated = false;
    status.unlock();
  }
  getVarPool()->unlock();
}

//////////////////////////////////////////////////////////////////

class UFileDevPoll
{
private:
  int hfd;
  char * buff;
  int buffCnt;
  char * next;
  int dataCnt;
  bool isOpen;
  //
  UFileDevPoll(int bufSize)
  {
    hfd = -1;
    buff = (char*)malloc(bufSize);
    buffCnt = bufSize;
    next = buff;
    dataCnt = 0;
    isOpen = false;
  };

  bool open(const char * deviceName)
  { return false; };

  void close()
  { ; };

  /**
  Get a line from a device file, but wait at maximum this amount of miliseconds */
  const char * getNextLine(int timeoutMiliSec) // timeout in seconds
  { // poll wait time
    int pollTime = timeoutMiliSec / 5 + 1; // ms
    int pollSum = 0;
    int n;
    struct pollfd ps;
    // Set poll structure for test of indata available
    ps.fd = hfd;
    ps.events = POLLIN;
    const char * result = NULL; // number of characters received
    char * bp = &buff[dataCnt]; // put new data here
    //
    if (next > buff and (dataCnt > (next - buff)))
    { // remove last line
      dataCnt -= next - buff;
      memmove(buff, next,  dataCnt + 1);
    }
    next = buff;
    /* get new data */
    while (dataCnt < buffCnt - 1)
    { // Wait for data in up to the timeout period
      if (poll(&ps, 1, pollTime) != POLLIN)
      { // timeout or other error - return
        pollSum += pollTime;
        if (pollSum > timeoutMiliSec)
          break;
      }
      else
      { /* data available, read up to a full message */
        n = read(hfd, bp, buffCnt - dataCnt);
        if (n == -1)
        { // error
          perror("Error in read from serial line");
          break;
        }
        else
        { //
          bp[n] = '\0';
          dataCnt += n;

/// hertil - denne tokenset skal nok op tidligere i while for at tillade at der er læst mere end 1 linie.

          // separate into line tokens, replasing token with '\0'
          // result is first line (terminated) and nedt is beginning of new line
          result = strsep(&next, "\r\n");
          if (next != NULL)
            break;

        }
      }
    }
    //
    return result;
  }
};

///////////////////////////////////////////////////////////

bool UResGps::decodeReplayLine(char * line)
{ // line is decoded in input thread
  // unread lines are dumped - assumed to be redundant
  replaySentenceLock.lock();
  strncpy(replaySentence, line, GPS_SENTENCE_MAX_LENGTH);
  replaySentenceNew = true;
  replaySentenceLock.unlock();
  return true;
}

