/***************************************************************************
 *   Copyright (C) 2009 by Christian Andersen   *
 *   chrand@mail.dk   *
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


#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <iostream>
#include <cstdlib>
#include <limits.h>
#include <string.h>
#include <stdio.h>
#include <ugen4/utime.h>
#include <ugen4/ucommon.h>

/// GPS max sentence length
#define GPS_SENTENCE_MAX_LENGTH 200

using namespace std;

/**
 * Open source and destination files */
bool openSourceFile(const char * src, FILE ** fs);
/**
 * Open source and destination files */
bool openDestFile(const char * dst, FILE ** fd);
/**
 * make destination file */
bool processFiles(FILE * fs, FILE * fd,
                  double timeOffset, int zone, int poseUtm, int poseOdo);

/////////////////////////////////////

int main(int argc, char *argv[])
{
  int i;
  const char * src = NULL;
  const char * dst = NULL;
  bool isHelp = false;
  char * p1;
  FILE *fs = NULL, *fd = NULL;
  bool isOK = false;
  double timeOffset;
  int zone = -1;
  int poseOdo = -1, poseUtm = -1;
  //
  for (i = 1; i < argc; i++)
  {
    if (argv[i][0] > ' ' and argv[i][0] != '-')
    { // assume a legal argument
      if (src == NULL)
      { // first argument is source
        src = argv[i];
      }
      else if (dst == NULL)
      { // destination filename
        dst = argv[i];
      }
    }
    else if (argv[i][0] > ' ')
    {
      p1 = &argv[i][1];
      if (*p1 == 's')
        timeOffset = strtod(&p1[1], NULL);
      else if (*p1 == 'z')
        zone = strtol(&p1[1], NULL, 0);
      else if (*p1 == 'p')
        poseOdo = strtol(&p1[1], NULL, 0);
      else if (*p1 == 'u')
        poseUtm = strtol(&p1[1], NULL, 0);
      else if (*p1 == 'h')
        isHelp = true;
      else if (*p1 == '-')
        isHelp = true;
      else
      {
        isHelp = true;
        printf("Unknown option %s\n\n", p1);
      }
    }
  }
  if (isHelp)
  { /* print help */
    printf("\nUtility restore timestamp in raw gps-NMEA logfile from GPGGA and PTNL,PJK messages\n");
    printf("Use:\n");
    printf(" %s [-sS] [-zZone] [-uA] [-pB] [-h] sourcefile destinationfile\n", argv[0]);
    printf(" where S is time offset in decimal seconds added to timestamp in GPS file - default 0sec)\n");
    printf(" where Zone is zone to be used for LL-to-UTM conversion - default from longitude\n");
    printf(" where A is 1 for utmPose from PTNL upm position, 2 for utmPose from GPGGA calc using zone\n");
    printf(" where B is 1 for odoPose from PTNL upm position, 2 for odoPose from GPGGA calc using zone\n\n");
  }
  else
  { // do some conversion
    isOK = openSourceFile(src, &fs);
    if (isOK)
    { // source and destination file is OK
      isOK = openDestFile(dst, &fd);
      if (isOK)
        isOK = processFiles(fs, fd, timeOffset, zone, poseUtm, poseOdo);
    }
    if (fs != NULL)
      fclose(fs);
    if (fd != NULL)
      fclose(fd);
  }
  if (isOK)
    printf("Finished OK\n");
  else
    printf("Failed\n");
  //
  return EXIT_SUCCESS;
}


bool openSourceFile(const char * src, FILE ** fs)
{
  const char * dsrc = "smrcl.log";
  const char * sfn = src;
  bool result = false;
  //
  if (sfn == NULL)
    sfn = dsrc;
  *fs = fopen(sfn, "r");
  if (*fs == NULL)
    printf("sourcefile not found: %s\n", src);
  else
  {
    result = true;
    printf("Reading from file '%s'\n", sfn);
  }
  return result;
}

/////////////////////////////////////////////////

bool openDestFile(const char * dst, FILE ** fd)
{
  const char * ddest = "smrcl-part.log";
  const char * dfn = dst;
  bool result = false;
  //
  if (dfn == NULL)
    dfn = ddest;
  *fd = fopen(dfn, "w");
  if (*fd == NULL)
    printf("Could not create destination file: %s\n", dfn);
  else
  {
    result = true;
    printf("Created destination file '%s'\n", dfn);
  }
  return result;
}

////////////////////////////////////////////

class UGpsUTM  
{
public:
  /**
  Constructor */
  UGpsUTM()
  {
    clear();
  };
  /**
  Clear the structure */
  void clear()
  {
    //Interanal GPS struct
    valid = 0;
    quality = 0;
    satellites = 0;
    dop = 0.0;
    northing = 0.0;
    easting = 0.0;
  }

  /**
   * Parse UTM position using Leika protocold */
  bool parseGPLLK(char * inBuf)
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
    return isOK;
  }

public:
   /// Show if the fix is valid
  bool valid;
   /// Show the quality of the fix (No gps/gps/dgps)
  int quality;
   /// Show the nuber of satellites used in the fix
  int satellites;
   /// Dillution of Precision for the fix
  double dop;
   /// detect time (UTM)
  UTime gmt;
   /// Northing coordinate UTM (zone 32)
  double northing;
   /// Easting coordinate UTM (zone 32)
  double easting;
   /// Antenna height data
  double height;
   /// Not used
  int height2;
  /// projection zone used
  int zone;
  /// last message received
  char sentence[GPS_SENTENCE_MAX_LENGTH];
};

class UGpsLatLong
{
public:
  /**
  Constructor */
  UGpsLatLong()
  {
    clear();
  }
  /**
  Clear the structure */
  void clear()
  {
    valid = false;
    dop = 99.0;
    gmt.clear(); // time
    latDeg = 0.0;
    north_lat = 'N';
    longDeg = 0.0;
    east_lon = 'E';
    speed = 0.0;
    heading = 0.0;
    height = 0.0;
    height2 = 0;
  }
  /**
   * Parse the GPRMC message
   * \param[in] *inBuf Pointer to the string to be parsed.
   * \param[in] *lastTime is the last time data were received, to reconstruct date (also for replay).
   * \return true on succes
   */
  bool parseGPGGA(char * inBuff, UTime lastTime)
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
    p2 = inBuff;
    p1 = strchr(p2, '*');
    if (p1 != NULL)
      // terminate before checksum
      *p1 = '\0';
    // make a copy of the sentence - for debug - monitoring
    strncpy(sentence, inBuff, GPS_SENTENCE_MAX_LENGTH);
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
    return isOK;
  }

public:
   /// Show if the fix is valid
  bool valid;
   /// Show the quality of the fix (No gps/gps/dgps)
  int quality;
  ///Has the fix used the EGNOS correction
  bool egnos;
   /// Show the nuber of satellites used in the fix
  int satellites;
   /// Dillution of Precision for the fix
  float dop;
   /// GMT time, as reported by the message (date may be reconstructed)
  UTime gmt;
   /// Latitude coordinate in decimal degrees, with North as positive
  double latDeg;
   /// Direction of the latitude
  char north_lat;
   /// Lontitude coordinate in decimal degrees, with East as positive
  double longDeg;
   /// Direction of the lontitude
  char east_lon;
   /// Speed of the reciever (m/s)
  float speed;
   /// Heading of the reciever (0-360 deg)
  float heading;
   /// Antenna height data (geoid height)
  double height;
   /// Height differential between an ellipsion and geoid
  int height2;
  /// last message received
  char sentence[GPS_SENTENCE_MAX_LENGTH];
};


///////////////////////////////////////////

int findZoneNumber(double LongDeg, double Lat)
{
  int ZoneNumber = int((LongDeg + 180)/6) + 1;
  //
  if( Lat >= 56.0 && Lat < 64.0 && LongDeg >= 3.0 && LongDeg < 12.0 )
    ZoneNumber = 32;
  // Special zones for Svalbard
  if( Lat >= 72.0 && Lat < 84.0 )
  {
    if(      LongDeg >= 0.0  && LongDeg <  9.0 ) ZoneNumber = 31;
    else if( LongDeg >= 9.0  && LongDeg < 21.0 ) ZoneNumber = 33;
    else if( LongDeg >= 21.0 && LongDeg < 33.0 ) ZoneNumber = 35;
    else if( LongDeg >= 33.0 && LongDeg < 42.0 ) ZoneNumber = 37;
  }
  return ZoneNumber;
}

///////////////////////////////////////////

bool processFiles(FILE * fs, FILE * fd,
                  double timeoffset, int zone, int poseUtm, int poseOdo)
{
  const int MSL = 10000;
  char s[MSL];
  char * p1;
  int n = 0;
  UGpsUTM utm;
  UGpsLatLong latlong;
  UTime tGPS, tLine;
  bool ignore;
  char sentence[GPS_SENTENCE_MAX_LENGTH];
  double northing, easting, latDeg, longDeg;
  FILE * fo = NULL;
  FILE * fu = NULL;
  int line = 0;
  double angle, eu, nu, eo, no;
  //
  if (poseUtm > 0)
    fu = fopen("utmPose.log", "w");
  if (poseOdo > 0)
    fo = fopen("odoPose.log", "w");
  //
  while (not feof(fs))
  {
    p1 = fgets(s, MSL, fs);
    if (p1 != NULL)
    {
      n++;
      ignore = p1[0] != '$';
      strncpy(sentence, p1, GPS_SENTENCE_MAX_LENGTH);
      if (strncmp(p1, "$GPGGA,", 7) == 0)
      { // fix quality, but not full data
        latlong.parseGPGGA(p1, tGPS);
        if (zone < 0)
        {
          zone = findZoneNumber(latlong.longDeg, latlong.latDeg);
          printf("Zone set to %d\n", zone);
        }
        LLtoUTM(23, latlong.latDeg, latlong.longDeg, &northing, &easting, zone);
//         sentence[50]= '\0';
//         printf(" Zone %d %fLa %fLo, %.2fN %.2fE LLtoUTM %s\n", 
//                zone, latlong.latDeg, latlong.longDeg, northing, easting, sentence);
        if (poseUtm == 2 or poseOdo == 2)
        {
          if (line == 0)
          {
            nu = northing;
            eu = easting;
            angle = 0.0;
            eo = eu; // save start position for odometry
            no = nu;
          }
          else
          {
            double a = angle = atan2(northing - nu, easting - eu);
            angle = angle * 0.8 + a * 0.2;
            nu = northing;
            eu = easting;
          }
          line++;
          tLine = tGPS;
          tLine.add(timeoffset);
          if (line > 1)
          {
            if (fo != NULL and poseOdo == 2)
              fprintf(fo, "%lu.%06lu %.3f %.3f %.3f %.3f\n", tLine.getSec(), 
                      tLine.getMicrosec(), eu - eo, nu - no, angle, latlong.dop);
            if (fu != NULL and poseUtm == 2)
              fprintf(fu, "%lu.%06lu %.3f %.3f %.3f %.3f %d\n", tLine.getSec(), 
                      tLine.getMicrosec(), eu, nu, angle, latlong.dop, latlong.quality);
          }
        }
      }
      else if (strncmp(p1, "$PTNL,PJK,", 10) == 0)
      { // UTM with full date-time data
        utm.parseGPLLK(p1);
        tGPS = utm.gmt;
        UTMtoLL(23, utm.northing, utm.easting, zone, &latDeg, &longDeg);
        //sentence[58]= '\0';
        //printf(" Zone %d %fLa %fLo, %.2fN %.2fE UTMtoLL %s\n", 
        //       zone, latDeg, longDeg, utm.northing, utm.easting, sentence);
        if (poseUtm == 1 or poseOdo == 1)
        {
          if (line == 0)
          {
            nu = utm.northing;
            eu = utm.easting;
            angle = 0.0;
            eo = eu; // save start position for odometry
            no = nu;
          }
          else
          { // find heading angle from movement
            double a = angle = atan2(utm.northing - nu, utm.easting - eu);
            angle = angle * 0.8 + a * 0.2;
            nu = utm.northing;
            eu = utm.easting;
          }
          line++;
          tLine = tGPS;
          tLine.add(timeoffset);
          if (line > 1)
          {
            if (fo != NULL and poseOdo == 1)
              fprintf(fo, "%lu.%06lu %.3f %.3f %.3f %.3f\n", tLine.getSec(), 
                      tLine.getMicrosec(), eu - eo, nu - no, angle, utm.dop);
            if (fu != NULL  and poseUtm == 1)
              fprintf(fu, "%lu.%06lu %.3f %.3f %.3f %.3f %d\n", tLine.getSec(), 
                      tLine.getMicrosec(), eu, nu, angle, utm.dop, utm.quality);
          }
        }
      }
    }
    if (not ignore)
    {
      tLine = tGPS;
      tLine.add(timeoffset);
      //fprintf(stdout, "%lu.%06lu %s", tLine.getSec(), tLine.getMicrosec(), sentence);
      fprintf(fd, "%lu.%06lu %s", tLine.getSec(), tLine.getMicrosec(), sentence);
    }
  }
  printf("wrote %d lines\n", n);
  if (fo != NULL)
    fclose(fo);
  if (fu != NULL)
    fclose(fu);
  return n > 0;
}

