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


#include <time.h>
#include <getopt.h>
#include <termios.h> // termio stuff
#include <ctype.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
       
#include "ucommon.h"
#include "u3d.h"
#include "umatrix.h"
#include "utime.h"
#include "uline.h"
#include "ulock.h"

/**
Allows (or not) some experimental packing using z-lib */
#define NO_ZLIB

#ifndef NO_ZLIB
#include <zlib.h>
#endif

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

// debug
ULock actionFlag;
// debug end

// path where to place or read files
char imagePath[MAX_PATH_LENGTH];
char dataPath[MAX_PATH_LENGTH];
char replayPath[MAX_PATH_LENGTH];
// camera configuration file
//char configFileCam[MAX_PATH_LENGTH];
int serverPort;
//char serverName[MAX_PATH_LENGTH];
const char * appName;
int s_argc;
char ** s_argv;

//void readPathFromConfigFile();

///////////////////////////////////////////////////////

void printHelp(const char * leadText)
{
  printf("\n");
  printf("%s %s\n", appName, leadText);
  printf(" Command line options:\n");
  printf(" -a --daemon\n");
  printf("        Run server as daemon (no console input)\n");
  printf(" -i --imagepath <path>\n");
  printf("        Sets path where images are expected to be found and written\n");
  printf("        (default ./log_%s)\n", appName);
  printf(" -d --datapath <path>\n");
  printf("        Sets path where logfiles and data (other than images) are expected\n");
  printf("        to be found and written (default ./log_%s)\n", appName);
  printf(" -s --script <file>\n");
  printf("        Set path and filename for command script (default ./%s.ini)\n", appName);
  printf(" -p --port <port>\n");
  printf("        Sets the server port number (default %d)\n", serverPort);
  printf(" -h --help :  This help message\n");
  printf("\n");
}


bool setCommonPathAndOtherOptions(int argc, char *argv[],
                 char * script, int scriptCnt, bool * daemon, int defaultServerPort,
                 const char * appHelpPreText)
{
  static struct option long_options[] = {
                   {"imagepath", 1, 0, 'i'},
                   {"datapath", 1, 0, 'd'},
                   {"daemon", 1, 0, 'a'},
                   {"script", 1, 0, 's'},
                   {"help", 0, 0, 'h'},
                   {"port", 1, 0, 'p'},
                   {0, 0, 0, 0}
               };
  bool ask4help = false;
  int opt;
  int option_index = 0;
  // make sure decimal point is '.'
  setlocale(LC_ALL, "C");
  // application name - with no path
  if (strrchr(argv[0], '/') == NULL)
    // application found in $PATH
    appName = argv[0];
  else
  { // application started using absolute or relative path
    appName = strrchr(argv[0], '/');
    appName++;
  }
  // set default values
  snprintf(imagePath, MAX_PATH_LENGTH, "./log_%s", appName);
  snprintf(dataPath, MAX_PATH_LENGTH, "./log_%s", appName);
  snprintf(replayPath, MAX_PATH_LENGTH, "./replay");
  // default port
  serverPort=defaultServerPort;
  // default scriptname is application name
  snprintf(script, scriptCnt, "./%s.ini",  appName);
  // set openCV to not breaf - this will probably give a protection error crash, but these are easier to trace.
  cv::setBreakOnError(false);
  // take path from command line
  while(true)
  {
    opt = getopt_long(argc, argv, "ahi:d:p:s:", long_options, &option_index);
    if (opt == -1)
      break;
    switch (opt)
    {
      case -1: // no more options
        break;
      case 'i': // path setting for images
        if (optarg != NULL)
          strncpy(imagePath, optarg, MAX_PATH_LENGTH);
        break;
      case 'd': // path setting for data (result path)
        if (optarg != NULL)
          strncpy(dataPath, optarg, MAX_PATH_LENGTH);
        break;
      case 'p': // server port number
        if (optarg != NULL)
          sscanf(optarg, "%d", &serverPort);
        break;
      case 's': // init script filename
        if (optarg != NULL)
          strncpy(script, optarg, scriptCnt);
        break;
      case 'a': // init script filename
        *daemon = true;
        break;
      case 'h':
        ask4help = true;
        break;
      default:
        //printf("Unknown option '%c'\n", opt);
        break;
    }
  }
  if (ask4help)
    printHelp(appHelpPreText);
  else
    testPathSettings();
  return ask4help;
}


///////////////////////////////////////////////////////

/**
 * make path (if it do not exist already)
 * \param path is the path to make or test if it exist
 * \returns true if the path was created or already existed.
 * \returns false if the path could not be created */
bool makePath(const char * path)
{ // make sure this path exist
  int err;
  struct stat sb;
  err = stat(path, &sb);
  if (err != 0)
    // directory does not exist (or other error)
    // try to make it
    err = mkdir(path, 0777);
  return (err == 0);
}

///////////////////////////////////////////////////////

void getIniFiles(const char * iniScript)
{
  const int MCL = 260;
  char cmd[MCL];
  const char * defIniPath = "/usr/local/smr/bin/aursconf";
  //
  snprintf(cmd, MCL, "stat %s >/dev/null", iniScript);
  // printf("Dooing a '%s'\n", cmd);
  if (system(cmd) != 0 and strstr(iniScript, appName) != NULL)
  { // no local ini scriptfile - so try to copy from /usr/local/smr/bin
    snprintf(cmd, MCL, "cp %s/%s.ini . >/dev/null", defIniPath, appName);
    if (system(cmd) == 0)
      printf("Default ini script copied from %s/%s.ini\n", defIniPath, appName);
    else
      printf("Default ini script not copied from %s/%s.ini\n", defIniPath, appName);
  }
}

///////////////////////////////////////////////////////

bool readConfigFile(Uconfig * ini)
{
  int err;
  // config path
  ini->clear();
  err = -1; // ini->readConfig(configFileCam);
  return err == 0;
}

///////////////////////////////////////////////////////

bool testPath(const char * path)
{
  bool result = (path != NULL);
  FILE * t;
  char s[MAX_PATH_LENGTH];
  //
  if (result)
    result = (strlen(path) > 0);
  if (result)
  { // try create a file
    snprintf(s, MAX_PATH_LENGTH, "%s/.PATH.TEST", path);
    t = fopen(s,"w");
    result = (t != NULL);
    if (result)
    {
      fclose(t);
      /// @todo should maybe remove the created file
    }
  }
  //
  return result;
}

///////////////////////////////////////////////////////////

void testPathSettings()
{ // test path settings
  char s[MAX_PATH_LENGTH];
  Uconfig ini;
  bool result;
  const char * homeDir = getenv("HOME");
  // open config file (may hold path settings)
  //result = readConfigFile(&ini);
  // image path
  makePath(imagePath);
  result = testPath(imagePath);
  if (not result and (strlen(imagePath) > 0))
  { // try relative to home
    snprintf(s, MAX_PATH_LENGTH, "%s/%s/",  homeDir, imagePath);
    strncpy(imagePath, s, MAX_PATH_LENGTH);
    result = testPath(imagePath);
  }
  if (not result)
  {
    strcpy(imagePath, ini.strGet("path", "imagePath", homeDir));
    result = testPath(imagePath);
  }
  if (not result)
    // no path, so use home directory
    snprintf(imagePath, MAX_PATH_LENGTH, "%s/", homeDir);
  //
  // data path
  makePath(dataPath);
  result = testPath(dataPath);
  if (not result and (strlen(dataPath) > 0))
  { // try relative to home
    snprintf(s, MAX_PATH_LENGTH, "%s/%s/", homeDir, dataPath);
    strncpy(dataPath, s, MAX_PATH_LENGTH);
    result = testPath(dataPath);
  }
  if (not result)
  {
    strcpy(dataPath, ini.strGet("path", "dataPath", homeDir));
    result = testPath(dataPath);
  }
  if (not result)
    // no path, so use home directory
    snprintf(dataPath, MAX_PATH_LENGTH, "%s/", homeDir);
  // set default replay path (different from log path)
  snprintf(replayPath, MAX_PATH_LENGTH, "%s/replay", dataPath);
}

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// lines

int pointsToLine(const float x1, const float y1,
                 const float x2, const float y2,
                 float * A, float * B, float * C)
{ // find line parameters from two points (x1,y1) and (x2,y2)
  // line equation is Ax + By + C = 0.
  // Returns -1 if points are not separated (within precition) else 0.
  int err = 0;
  float margin = maxf(maxf(absf(x1), absf(x2)), maxf(absf(y1),absf(y2))) * 1.0e-8;
  //
  if ((absf(x1-x2) < margin) and
      (absf(y1-y2) < margin))
    // points must be separated
    err = -1;
  else
  { // points are separated, so find line parameters
    // get line equation Ax + By + C = 0
    if (absf(y1-y2) < absf(x1-x2))
    { // more x oriented, so set B = 1
      *B = 1.0;
      *C = (x2*y1 - x1*y2)/(x1-x2);
      if (absf(x1) > absf(x2))
        *A = -(y1 + *C)/x1;
      else
        *A = -(y2 + *C)/x2;
    }
    else
    { // more y oriented, so set A = 1
      *A = 1.0;
      *C = (x1*y2 - x2*y1)/(y1-y2);
      if (absf(y1) > absf(y2))
        *B = -(x1 + *C)/y1;
      else
        *B = -(x2 + *C)/y2;
    }
  }
  return err;
}

/////////////////////////////////////////////////////////////////////

int pointsToLine(const double x1, const double y1,
                 const double x2, const double y2,
                 double * A, double * B, double * C)
{ // find line parameters from two points (x1,y1) and (x2,y2)
  // line equation is Ax + By + C = 0.
  // Returns -1 if points are not separated (within precition) else 0.
  int err = 0;
  double margin = maxd(maxd(absd(x1), absd(x2)), maxd(absd(y1),absd(y2))) * 1.0e-12;
  //
  if ((absd(x1-x2) < margin) and
      (absd(y1-y2) < margin))
    // points must be separated
    err = -1;
  else
  { // points are separated, so find line parameters
    // get line equation Ax + By + C = 0
    if (absd(y1-y2) < absd(x1-x2))
    { // more x oriented, so set B = 1
      *B = 1.0;
      *C = (x2*y1 - x1*y2)/(x1-x2);
      if (absd(x1) > absd(x2))
        *A = -(y1 + *C)/x1;
      else
        *A = -(y2 + *C)/x2;
    }
    else
    { // more y oriented, so set A = 1
      *A = 1.0;
      *C = (x1*y2 - x2*y1)/(y1-y2);
      if (absd(y1) > absd(y2))
        *B = -(x1 + *C)/y1;
      else
        *B = -(x2 + *C)/y2;
    }
  }
  return err;
}

///////////////////////////////////////////////////////

double DistanceSqFromEHNLine(const double E, const double H, const double N,
                       const double A, const double B, const double C,
                       const double E0, const double H0, const double N0)
{ // squared distance form 3D line
  double t, dE, dH, dN, result;
  // parameter line: [e,h,n] = [A,B,C]*t + [e0,h0,n0];
  // find point on parameter line closest to point
  // (assumes [A,B,C] is a unit vector).
  t = PositionOnEHNLine(E, H, N, A, B, C, E0, H0, N0);
  // get vector from line to point.
  dE = A * t + E0 - E;
  dH = B * t + H0 - H;
  dN = C * t + N0 - N;
  // get distance
  result = sqr(dE) + sqr(dH) + sqr(dN);
  return result;
}

/////////////////////////////////////////////////////////

const char * bool2str(bool value)
{
  if (value)
    return "true";
  else
    return "false";
}

////////////////////////////////////////////////////////

bool str2bool(const char * value)
{
  return str2bool2(value, false);
}

////////////////////////////////////////////////////////

bool str2bool2(const char * value, bool def)
{
  bool result;
  const char * c = value;
  while (*c == ' ')
    // skip leading white space
    c++;
  if (def)
    result = (strncasecmp(c, "false", 5) != 0);
  else
    result = (strncasecmp(c, "true", 4) == 0);
  //
  return result;
}

////////////////////////////////////////////////////////

bool str2bool3(const char * value, bool def, const char ** p2)
{
  bool result;
  const char * c = value;
  while (*c == ' ')
    // skip leading white space
    c++;
  if (def)
    result = (strncasecmp(c, "false", 5) != 0);
  else
    result = (strncasecmp(c, "true", 4) == 0);
  if (p2 != NULL)
  { // set next character
  if (def and not result)
    *p2 = &c[5];
  else if (not def and result)
    *p2 = &c[4];
  else if (result and strncasecmp(c, "true", 4) == 0)
    *p2 = &c[4];
  else if (not result and strncasecmp(c, "false", 4) == 0)
    *p2 = &c[5];
  else
    *p2 = value;
  }
  //
  return result;
}

////////////////////////////////////////////////////////


int hex2int(char msn, char lsn)
{
  int result;
  if (msn < 'a')
    result = (msn - '0') << 4;
  else
    result = (msn - ('a' - 10)) << 4;
  if (lsn < 'a')
    result += (lsn - '0');
  else
    result += (lsn - ('a' - 10));
  return result;
}

////////////////////////////////////////////////////////

// Taken from MOOSGenLibGlobalHelper by Paul Newman
// via S�ren & Rufus
int MOOSGetch()
{
  int c, fd = 0;
  struct termios term, oterm;

  /* get the terminal settings */
  tcgetattr(fd, &oterm);

  /* get a copy of the settings, which we modify */
  memcpy(&term, &oterm, sizeof(term));

  /* put the terminal in non-canonical mode, any
     reads will wait until a character has been
     pressed. This function will not time out
  */
  term.c_lflag = term.c_lflag & (!ICANON);
  term.c_cc[VMIN] = 1;
  term.c_cc[VTIME] = 0;
  tcsetattr(fd, TCSANOW, &term);

  /* get a character. c is the character */
  c = getchar();

  /* reset the terminal to its original state */
  tcsetattr(fd, TCSANOW, &oterm);

  /* return the charcter */
  return c;
}

//////////////////////////////////////////////////////

int getKey(int * nkey, int * ckey)
{
  int input;
  int res = 1;
  //
  *ckey = 0;
  input = MOOSGetch();
  if ((int) input == 126) // PAGE UP/DOWN 4th CHARS: 126 plus ARROW key chars!!!
  { // first char is just ignored (part of old key sequense)
    input = MOOSGetch();
  }
  *nkey = input;
  // printf(",0x%02x(%d)", input, input);
  if ((int) input == 27) // char escape code 1 - ARROW KEYS
  {
    *ckey = MOOSGetch();
    // printf(",0x%02x(%d)", *ckey, *ckey);
    res++;
    // show known
    if ((int) *ckey == 91) // char escape code 2 - ARROW KEYS
    {
      *ckey = MOOSGetch();
      // printf(",0x%02x(%d)", *ckey, *ckey);
      res++;
    }
  }
  // printf(" ( n = %d)\n", res);
  //
  /*
  if (res == 1)
  {
    if (*nkey > ' ')
      printf("%c\n", char(*nkey));
  }
  else
  {
    if ((int) *ckey == UP) // char escape code for "up"
      printf("UP\n");
    else if ((int) *ckey == DOWN)
      printf("DOWN\n");
    else if ((int) *ckey == RIGHT)
      printf("RIGHT\n");
    else if ((int) *ckey == LEFT)
      printf("LEFT\n");
    else if ((int) *ckey == PAGEUP)
      printf("PAGEUP\n");
    else if ((int) *ckey == PAGEDOWN)
      printf("PAGEDOWN\n");
    else
      printf("other esc2 0x%x\n", *ckey);

  } // Char escape code 1
  */
  //
  return res;
}

//////////////////////////////////////////////

// bool inThisStringList(const char * str, const char * strList)
// {
//   bool result = false;
//   const char * p1 = strList;
//   const char * p2;
//   int sl = strlen(str);
//   //
//   while (not result)
//   {
//     if ((strncasecmp(str, p1, sl) == 0) and
//          (p1[sl] <= ' '))
//       // string is found (and is not a substring)
//       result = true;
//     else
//     { // try the next in list
//       p2 = strchr(p1, ' ');
//       if (p2 == NULL)
//         // string not found
//         break;
//       else
//       { // advance to first pointer past the space
//         p1 = p2;
//         p1++;
//       }
//     }
//   }
//   return result;
// }

///////////////////////////////////////////////////////

bool inThisStringList(const char * str, const char * strList)
{
  const char * p1, * p2 = strList;
  int m = strlen(str);
  bool found = false;
  //
  if (strlen(str) > 0)
  {
    while (not found)
    {
      p1 = strcasestr(p2, str);
      found = p1 != NULL;
      if (not found)
        break;
      // may be a substring
      p2 = p1 + m;
      // OK to add if either side of the found string is not a separator
      if (p1 == strList)
        found = (*p2 <= ' '); // space or terminator
      else
        found = (*p2 <= ' ') and (*(--p1) == ' ');
    }
  }
  return found;
}

//////////////////////////////////////////////////////

bool pattern_match(const char *str, const char *pattern)
{
  if (true)
    // use simpler version
    return wildcmp(pattern, str);
  else
  { // old method maintained until debugged
    enum State
    {
      Exact,        // exact match
      Any,        // ?
      AnyRepeat    // *
    };
    const char *s = str;
    const char *p = pattern;
    const char *q = 0;
    int state = 0;
    bool match = true;
    //
    while (match && *p)
    {
      if (*p == '*')
      {
        state = AnyRepeat;
        q = p+1;
      }
      else if (*p == '?')
        state = Any;
      else
        state = Exact;
      //
      if (*s == 0)
        break;
      //
      switch (state)
      {
        case Exact:
          match = *s == *p;
          s++;
          p++;
          break;
        case Any:
          match = true;
          s++;
          p++;
          break;
        case AnyRepeat:
          match = true;
          s++;
          if (*s == *q) p++;
          break;
      }
    }
    if (state == AnyRepeat)
      return (*s == *q);
    else if (state == Any)
      return (*s == *p);
    else
      return match && (*s == *p);
  }
}


///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// URGBPIXEL
///////////////////////////////////////////////////////

UPixel::UPixel(unsigned char ip1,
                     unsigned char ip2,
                     unsigned char ip3):
p1(ip1), p2(ip2), p3(ip3)
{
}

////////////////////////////////////////////////////////

UPixel::UPixel()
{
  // do nothing
}

////////////////////////////////////////////////////////

void UPixel::clear()
{
  p1 = 0;
  p2 = 0;
  p3 = 0;
}

////////////////////////////////////////////////////////

void UPixel::setYUVto(unsigned char iy,
                      unsigned char iu,
                      unsigned char iv,
                      int toFormat /*= PIX_PLANES_BGR*/)
{
  //unsigned char rb;
  switch (toFormat)
  {
    case PIX_PLANES_BW:
      y = y;
      break;
    case PIX_PLANES_RGB:
      *this = YUVtoRGB(iy, iu, iv);
      break;
    case PIX_PLANES_BGR:
      *this = YUVtoBGR(iy, iu, iv);
      break;
    case PIX_PLANES_YUV:
      set(iy, iu, iv);
      break;
    default:
      break;
  }
}

////////////////////////////////////////////////////////

void UPixel::setRGBto(unsigned char ir,
                      unsigned char ig,
                      unsigned char ib,
                      int toFormat /*= PIX_PLANES_BGR*/)
{
  switch (toFormat)
  {
    case PIX_PLANES_BW:
      y = (ir + ig + ib)/3;
      break;
    case PIX_PLANES_RGB:
      set(ir, ig, ib);
      break;
    case PIX_PLANES_BGR:
      p3 = ir;
      p2 = ig;
      p1 = ib;
      break;
    case PIX_PLANES_YUV:
      *this = RGBtoYUV(ir, ig, ib);
      break;
    default:
      break;
  }
}

////////////////////////////////////////////////////////

// UPixel UPixel::YUVtoRGB(unsigned char iy,
//                       unsigned char iu,
//                       unsigned char iv)
// { // convert from YUV to RGB
//   UPixel result;
//   if (true)
//   { // converion from file:
//     // ... camstream/camstream-0.26pre2/lib/ccvt/ccvt_c1.c
//     // R = Y +                0. (V - 128)
//     // G = Y - 0. (U - 128) - 0. (V - 128)
//     // B = Y + 1.499 (U - 128)
//     int yy, uu ,vv;
//     yy = iy << 8;
//     uu = iu - 128;
//     vv = iv - 128;
//     // red
//     result.p1 = LIMIT_RGB((yy            + 359 * vv) >> 8);
//     // green
//     result.p2 = LIMIT_RGB((yy -  88 * uu - 183 * vv) >> 8);
//     // blue
//     result.p3 = LIMIT_RGB((yy + 454 * uu           ) >> 8);
//   }
// }

//////////////////////////////////////////

// UPixel UPixel::YUVtoBGR(unsigned char iy,
//                       unsigned char iu,
//                       unsigned char iv)
// { // convert from YUV to RGB
//   UPixel result;
//   if (true)
//   { // converion from file:
//     // ... camstream/camstream-0.26pre2/lib/ccvt/ccvt_c1.c
//     // R = Y +                0. (V - 128)
//     // G = Y - 0. (U - 128) - 0. (V - 128)
//     // B = Y + 1.499 (U - 128)
//     int yy, uu ,vv;
//     yy = iy << 8;
//     uu = iu - 128;
//     vv = iv - 128;
//     // red
//     result.p3 = LIMIT_RGB((yy            + 359 * vv) >> 8);
//     // green
//     result.p2 = LIMIT_RGB((yy -  88 * uu - 183 * vv) >> 8);
//     // blue
//     result.p1 = LIMIT_RGB((yy + 454 * uu           ) >> 8);
//   }
//   if (false)
//   { //  conversion from
//     //  "/usr/src/linux-2.4.18-18.8.0/drivers/usb/usbvideo.h"
//     int mm_y, mm_yc, mm_u, mm_v, mm_r, mm_g, mm_b;
//     // NB! U and V are exchanged relative to calculated values
//     mm_y = (iy) - 16;
//     mm_u = (iv) - 128;
//     mm_v = (iu) - 128;
//     mm_yc= mm_y * 76284;
//     mm_b = (mm_yc +                 132252 * mm_v) >> 16;
//     mm_g = (mm_yc -  53281 * mm_u -  25625 * mm_v) >> 16;
//     mm_r = (mm_yc + 104595 * mm_u                ) >> 16;
//     result.p1 = LIMIT_RGB(mm_b);
//     result.p2 = LIMIT_RGB(mm_g);
//     result.p3 = LIMIT_RGB(mm_r);
//   }
//   if (false)
//   {
//     // y,u,v in [0..255], rgb in [0..255]
//     // for R,G,B,Y in [0..1] and U,V in [-1..1]:
//     // R = y -1/3u +1/sqrt(3)v
//     // G = y -1/3u -1/sqrt(3)v
//     // B = y +2/3u
//     //float fy = float(y)/256.0;
//     //float fu = 1.0/3.0*(float(u)/128.0-1.0);
//     //float fv = 1.0/sqrt(3.0)*(float(v)/128.0-1.0);
//     int fu = (iu * 2 - 256) / 3;
//     int fv = ((iv * 2 - 256) * 577) / 1000;
//     // red
//     int i = iy - fu + fv;
//     result.p3 = LIMIT_RGB(i);
//     // green
//     i = iy - fu - fv;
//     result.p2 = LIMIT_RGB(i);
//     // blue
//     i = iy + 2 * fu;
//     result.p1 = LIMIT_RGB(i);
//   }
//   return result;
// }

/////////////////////////////////////////////////////////

/*
 The following macro is found in
 "/usr/src/linux-2.4.18-18.8.0/drivers/usb/usbvideo.h"

 * We use macros to do YUV -> RGB conversion because this is
 * very important for speed and totally unimportant for size.
 *
 * YUV -> RGB Conversion
 * ---------------------
 *
 * B = 1.164*(Y-16)       + 2.018*(V-128)
 * G = 1.164*(Y-16) - 0.813*(U-128) - 0.391*(V-128)
 * R = 1.164*(Y-16) + 1.596*(U-128)
 *
 * If you fancy integer arithmetics (as you should), hear this:
 *
 * 65536*B = 76284*(Y-16)     + 132252*(V-128)
 * 65536*G = 76284*(Y-16) -  53281*(U-128) -  25625*(V-128)
 * 65536*R = 76284*(Y-16) + 104595*(U-128)
 *
 * Make sure the output values are within [0..255] range.
 *
#define LIMIT_RGB(x) (((x) < 0) ? 0 : (((x) > 255) ? 255 : (x)))
#define YUV_TO_RGB_BY_THE_BOOK(my,mu,mv,mr,mg,mb) { \
    int mm_y, mm_yc, mm_u, mm_v, mm_r, mm_g, mm_b; \
    mm_y = (my) - 16;  \
    mm_u = (mu) - 128; \
    mm_v = (mv) - 128; \
    mm_yc= mm_y * 76284; \
    mm_b = (mm_yc   + 132252*mm_v ) >> 16; \
    mm_g = (mm_yc -  53281*mm_u -  25625*mm_v ) >> 16; \
    mm_r = (mm_yc + 104595*mm_u     ) >> 16; \
    mb = LIMIT_RGB(mm_b); \
    mg = LIMIT_RGB(mm_g); \
    mr = LIMIT_RGB(mm_r); \
}
*/


// UPixel UPixel::RGBtoYUV(unsigned char ir, unsigned char ig, unsigned char ib)
// { // modified to new calculation
//   UPixel result;
//   /*
//     65536*(y-16)  = 19592*R + 38480*G +  7468*B
//     65536*(U-128) = 28787*R - 24107*G -  4674*B
//     65536*(V-128) = -9713*R - 19068*G + 28787*B
//   */
//   int mmY, mmU, mmV;
//   //
//   mmY = ((19592 * ir + 38480 * ig +  7468 * ib) >> 16);
//   mmU = ((28787 * ir - 24107 * ig -  4674 * ib) >> 16) + 128;
//   mmV = ((-9713 * ir - 19068 * ig + 28787 * ib) >> 16) + 128;
//   // NB! U and V are exchanged relative to calculated values
//   result.y = LIMIT_RGB(mmY);
//   result.v = LIMIT_RGB(mmU);
//   result.u = LIMIT_RGB(mmV);
//   /*
//   // y,u,v in [0..255], rgb in [0..255]
//   // for R,G,B,Y in [0..1] and U,V in [-1..1]:
//   // Y = r/3 + g/3 + b/3
//   // U = -r/2 -g/2 + b
//   // V = sqrt(3)/2*r - sqrt(3)/2*g
//   // sqrt(3)/2 * 256 = 221.7
//   const int sqrt3Half = 222;
//   // convert to integer
//   int R = ir;
//   int G = ig;
//   int B = ib;
//   int uv;
//   // Y is just average intensity this.v
//   y = (unsigned char)((R + G + B)/3);
//   // U and V is intended in range [-1..1]
//   // but is here converted to range [-128 .. 127] and
//   // offset to 0..255 by adding 128.
//   uv = ((-R -G + (B << 1)) >> 2) + 128;
//   if (uv > 255) uv = 255;
//   else if (uv < 0) uv = 0;
//   u = (unsigned char)uv;
//   // v value
//   uv = ((sqrt3Half * R - sqrt3Half * G) >> 9) + 128;
//   if (uv > 255) uv = 255;
//   else if (uv < 0) uv = 0;
//   v = (unsigned char)uv;
//   */
//   return result;
// }

////////////////////////////////////////////////////////

void UPixel::set(int serial, int palSize)
{ // sets a color number in a limited palette
  // of max 128
  int factor = 512 / palSize;
  int palCol = serial * factor + serial;
  unsigned char red = (palCol % 8 << 5);
  unsigned char green = ((palCol >> 3) % 8) << 5;
  unsigned char blue = ((palCol >> 6) % 8) << 5;
  p3 = red;
  p2 = green;
  p1 = blue;
}

////////////////////////////////////////////////////////

void UPixel::set(URRgb rrgb)
{ // sets colour from float version of colour
  p3 = roundi(maxd(0.0, mind(255.0, rrgb.r)));
  p2 = roundi(maxd(0.0, mind(255.0, rrgb.g)));
  p1 = roundi(maxd(0.0, mind(255.0, rrgb.b)));
}

///////////////////////////////////////////

void UPixel::setYUVPix8(int colorIndex, int intensity /*= 255*/)
{ // set pixel from color index (8 colors) in YUV format
  setRGBPix8(colorIndex, intensity);
  // convert to YUV;
  *this = RGBtoYUV(p3, p2, p1);
}

////////////////////////////////////////////

void UPixel::setRGBPix8(int colorIndex, int intensity /*= 255*/)
{ // set color to one of 8 major color directions in RGB format
  int intens = mini(255, maxi(0, intensity));
  //
  switch (colorIndex % 8)
  {
  case 7 : p3 =   0; p2 = 128; p1 =   0; break;
  case 6 : p3 = 128; p2 =   0; p1 =   0; break; // magenta
  case 5 : p3 = 255; p2 =   0; p1 = 255; break; // blue
  case 4 : p3 =   0; p2 = 255; p1 = 255; break; // cyan
  case 3 : p3 = 255; p2 = 255; p1 =   0; break;
  case 2 : p3 =   0; p2 =   0; p1 = 255; break; // green
  case 1 : p3 =   0; p2 = 255; p1 =   0; break; // yellow
  case 0 : p3 = 255; p2 =   0; p1 =   0; break; // red
  }
  p3 = (unsigned char)((p3 * 255)/intens);
  p2 = (unsigned char)((p2 * 255)/intens);
  p1 = (unsigned char)((p1 * 255)/intens);
}

/////////////////////////////////////////////////////////

float UPixel::colorDist(UPixel * pix)
{ // result is in "color units" 0..255
  float result;
  result = sqrt(float(sqr(int(pix->p1) - int(p1)) +
                      sqr(int(pix->p2) - int(p2)) +
                      sqr(int(pix->p3) - int(p3))));
  return result;
}

/////////////////////////////////////////////////////////

float UPixel::colorDistIHS(UPixel * pix)
{
  UIUVPixel pix1(this);
  UIUVPixel pix2(pix);
  float result;
  result = pix1.colorDist(&pix2);
  return result;
}

////////////////////////////////////////////////////////

UPixel UPixel::asRGB(int fromFormat)
{
  UPixel result;
  switch (fromFormat)
  {
    case PIX_PLANES_BW:
      result.p1 = y;
      result.p2 = y;
      result.p3 = y;
      break;
    case PIX_PLANES_RGB:
      result = *this;
      break;
    case PIX_PLANES_BGR:
      result.set(p3, p2, p1);
      break;
    case PIX_PLANES_YUV:
      result = YUVtoRGB(y, u, v);
      break;
    default:
      break;
  }
  return result;
}
////////////////////////////////////////////////////////

unsigned char UPixel::getRed(int fromFormat)
{
  unsigned char result;
  UPixel rgb;
  switch (fromFormat)
  {
    case PIX_PLANES_BW:
      result = p1;
      break;
    case PIX_PLANES_RGB:
      result = p1;
      break;
    case PIX_PLANES_BGR:
      result = p3;
      break;
    case PIX_PLANES_YUV:
      rgb = YUVtoRGB(y, u, v);
      result = rgb.p1;
      break;
    default:
      result = 0;
      break;
  }
  return result;
}

/////////////////////////////////////////////////////////////////

unsigned char UPixel::getBlue(int fromFormat)
{
  unsigned char result;
  UPixel rgb;
  switch (fromFormat)
  {
    case PIX_PLANES_BW:
      result = p1;
      break;
    case PIX_PLANES_RGB:
      result = p3;
      break;
    case PIX_PLANES_BGR:
      result = p1;
      break;
    case PIX_PLANES_YUV:
      rgb = YUVtoRGB(y, u, v);
      result = rgb.p3;
      break;
    default:
      result = 0;
      break;
  }
  return result;
}

/////////////////////////////////////////////////////////////////

unsigned char UPixel::getGreen(int fromFormat)
{
  unsigned char result;
  UPixel rgb;
  switch (fromFormat)
  {
    case PIX_PLANES_BW:
      result = p1;
      break;
    case PIX_PLANES_RGB:
    case PIX_PLANES_BGR:
      result = p2;
      break;
    case PIX_PLANES_YUV:
      rgb = YUVtoRGB(y, u, v);
      result = rgb.p2;
      break;
    default:
      result = 0;
      break;
  }
  return result;
}

/////////////////////////////////////////////////////////////////

UPixel UPixel::asBGR(int fromFormat)
{
  UPixel result;
  switch (fromFormat)
  {
    case PIX_PLANES_BW:
      result.p1 = y;
      result.p2 = y;
      result.p3 = y;
      break;
    case PIX_PLANES_RGB:
      result.set(p3, p2, p1);
      break;
    case PIX_PLANES_BGR:
      result = *this;
      break;
    case PIX_PLANES_YUV:
      result = YUVtoBGR(y, u, v);
      break;
    default:
      break;
  }
  return result;
}

/////////////////////////////////////////////////////////////////

UPixel UPixel::asCromaBGR(int fromFormat)
{
  UPixel result;
  UPixel bgr;
  int sum;
  switch (fromFormat)
  {
    case PIX_PLANES_BW:
      result.p1 = 256/3;
      result.p2 = 256/3;
      result.p3 = 256/3;
      break;
    case PIX_PLANES_RGB:
    case PIX_PLANES_YUV:
      bgr = asBGR(fromFormat);
    case PIX_PLANES_BGR:
      sum = p1 + p2 + p3;
      if (sum == 0)
      { // total black is an exception
        result.p1 = 256/3;
        result.p2 = 256/3;
        result.p3 = 256/3;
      }
      else
      {
        result.p1 = (256 * p1)/sum;
        result.p2 = (256 * p2)/sum;
        result.p3 = (256 * p3)/sum;
      }
      break;
    default:
      break;
  }
  return result;
}

////////////////////////////////////////////////////////

UPixel UPixel::asYUV(int fromFormat)
{
  UPixel result;
  switch (fromFormat)
  {
    case PIX_PLANES_BW:
      result.y = y;
      result.u = 128;
      result.v = 128;
      break;
    case PIX_PLANES_RGB:
      result = RGBtoYUV(p1, p2, p3);
      break;
    case PIX_PLANES_BGR:
      result = RGBtoYUV(p3, p2, p1);
      break;
    case PIX_PLANES_YUV:
      result = *this;
      break;
    default:
      break;
  }
  return result;
}

////////////////////////////////////////////////////////

void UPixel::print(char * prestring)
{
  printf("%s p1:%3d, p2:%3d, p3:%3d, Y:%3d\n",
    prestring, p1, p2, p3, (p1+p2+p3)/3);
}

////////////////////////////////////////////////////////


UPixel::~UPixel()
{
}

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
// UIUVPIXEL

  //float i;
  //float u;
  //float v;
  //float hue;
  //float sat;
UIUVPixel::UIUVPixel()
: i(0.0),
  u(0.0),
  v(0.9),
  hue(0.0),
  sat(0.0)
{
}

////////////////////////////////////////////////////////

UIUVPixel::UIUVPixel(UPixel * pix)
{
  SetPix(*pix);
}

////////////////////////////////////////////////////

void UIUVPixel::SetPix(UPixel pix)
{
  URRgb rpix;
  rpix.r = pix.p3;
  rpix.g = pix.p2;
  rpix.b = pix.p1;
  SetPix(rpix);
}

////////////////////////////////////////////////////////

void UIUVPixel::SetPix(URRgb pix)
{
  i = (pix.r + pix.g + pix.b)/3.0/256.0;   // [0..1]
  u = (pix.b - (pix.r + pix.g)/2.0)/128.0; // [-1..1]
  v = (0.866/128.0) * (pix.r - pix.g);     // [-1..1] (ca.)
  hue = atan2(v,u); // +/- Pi
  sat = hypot(u,v); // [0..1]
}

////////////////////////////////////////////////////////

float UIUVPixel::colorDist(UIUVPixel * pix)
{ // calculates the distance between two pixels in IHS space
  float result;
  result = sqrt(sqr(pix->i - i) + sqr(pix->u - u) + sqr(pix->v - v));
  return result;
}

////////////////////////////////////////////////////////

/**
Wait (suspend) for a minimum wait time SECS in
seconds and decimal seconds to about 1 milisec,
at least 0.1 msec */
void Wait(float secs)
{  // usleep()
  struct timespec waitt;
  waitt.tv_sec = int(secs);
  waitt.tv_nsec = (long int)((secs - float(waitt.tv_sec) + 0.0001) * 1e9);
  nanosleep(&waitt,NULL);
}


////////////////////////////////////////////////////////

void UPixel::tone(UPixel other, int pct)
{ // tone this pixel with 'pct' of other pixel
  y = (y * pct + other.y * (100 - pct)) / 100;
  u = (u * pct + other.u * (100 - pct)) / 100;
  v = (v * pct + other.v * (100 - pct)) / 100;
}

////////////////////////////////////////////////////////

UPixel UPixel::toned(UPixel other, int pct)
{ // tone this pixel with 'pct' of other pixel
  // and return the toned resuld without changing this
  // pixel
  UPixel result = *this;
  result.tone(other, pct);
  return result;
}

////////////////////////////////////////////////////////


/**
Returns true if cahracter is in range 0..9, a..z, A..Z,
else false */
bool isAlphaNum(const char c)
{
  bool result;
  //
  result = ((c >= '0') and (c <= '9')) or
           ((c >= 'a') and (c <= 'z')) or
           ((c >= 'A') and (c <= 'Z'));
  //
  return result;
}
/**
Returns true if alphanumberic ore one of "_-/+~" */
bool isFileNameChar(const char c)
{
  bool result;
  const char others[] = "-_+~.";
  //
  result = isAlphaNum(c);
  if ((not result) and
      (strchr(others, c) != NULL))
    result = true;
  //
  return result;
}

////////////////////////////////////////////////////////
/**
Convert source string so that only 'isFileNameChar()' characters is used.
At maximum 'bufLen - 1' characters is converted. Destnation string is
zero terminated. Source and destination may be the same.
Returns true if successful. */
bool convertToFilename(char * dest, const int bufLen, const char * source)
{
  bool result;
  int i, j, l;
  //
  l = strlen(source);
  j = 0;
  for (i = 0; i < l; i++)
  {
    if (j >= (bufLen - 1))
      break;
    if (isFileNameChar(source[i]))
      dest[j++] = source[i];
  }
  result = (j > 0);
  if (result)
    // terminate destination string
    dest[j] = 0;
  //
  return result;
}

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

int unpackZlib(char * source, int sourceCnt, char * destBuffer, int destBufCnt)
{
  int destCnt = -1;
#ifndef NO_ZLIB
  bool result = false;
  int ret;
  z_stream strm;
  char * in = source;
  char * out = destBuffer;
  //
  if (destBufCnt > 0)
  { // there is data to process and space to put it
    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    strm.avail_in = 0;
    strm.next_in = Z_NULL;
    ret = inflateInit(&strm);
    result = (ret == Z_OK);
    if (not result)
      printf("unpackZlib: memory allocation fault\n");
  }
  if (result)
  { // set buffer pointers
    strm.avail_in = sourceCnt;
    strm.next_in = (Bytef*)in;
    strm.avail_out = destBufCnt;
    strm.next_out = (Bytef*)out;
    // inflate data
    ret = inflate(&strm, Z_NO_FLUSH);
    result = (ret != Z_NEED_DICT) and
        (ret != Z_DATA_ERROR) and
        (ret != Z_MEM_ERROR);
    if (not result)
      printf("unpackZlib: Decompression error (%d)\n", ret);
    inflateEnd(&strm);
  }
  if (result)
    destCnt = destBufCnt - strm.avail_out;
  //
#endif
  return destCnt;
}

////////////////////////////////////////////////////

int packZlib(char * source, int sourceCnt, char * destBuf, int destBufSize)
{
  int byteCnt = -1;
#ifndef NO_ZLIB
  int ret, flush;
  z_stream strm;
  char * in = source;
  char * out = destBuf;
  const int level = 4;
  bool result;
  //
  strm.zalloc = Z_NULL;
  strm.zfree = Z_NULL;
  strm.opaque = Z_NULL;
  ret = deflateInit(&strm, level);
  result = (ret == Z_OK);
  if (result)
  {
    strm.avail_in = sourceCnt;
    strm.next_in = (Bytef *)in;
    strm.avail_out = destBufSize;
    strm.next_out = (Bytef *)out;
    flush = Z_FINISH;
    ret = deflate(&strm, flush);
    byteCnt = destBufSize - strm.avail_out;
    // threr must be at least one byte spare in buffer
    // or there is no way to tell weather the
    // compression fails.
    result = (strm.avail_out > 0) and (strm.avail_in == 0);
    if (not result)
    {
      printf("packZLIB: destBuffer too small (%d bytes) "
          "for packing %d source data\n", destBufSize, sourceCnt);
      byteCnt = -1;
    }
    deflateEnd(&strm);
  }
#endif
  return byteCnt;
}

////////////////////////////////////////////////////

bool hasOverlap(double minX1, double minY1,
                double maxX1, double maxY1,
                double minX2, double minY2,
                double maxX2, double maxY2)
{
  bool result;
  // x overlap
  result = (minX1 < maxX2) and (maxX1 > minX2);
  if (result)
    result = (minY1 < maxY2) and (maxY1 > minY2);
  return result;
}

////////////////////////////////////////////////////

extern char * strnupper(char * dest, char * source, int destCnt)
{
  int i, n;
  char *p1, *p2;
  //
  p1 = source;
  p2 = dest;
  n = mini(strlen(source), destCnt - 1);
  for (i = 0; i < n ; i++)
    *p2++ = toupper(*p1++);
  *p2 = '\0';
  return dest;
}

////////////////////////////////////////////////////

extern char * strnlower(char * dest, char * source, int destCnt)
{
  int i, n;
  char *p1, *p2;
  //
  p1 = source;
  p2 = dest;
  n = mini(strlen(source), destCnt - 1);
  for (i = 0; i < n ; i++)
    *p2++ = tolower(*p1++);
  *p2 = '\0';
  return dest;
}

////////////////////////////////////////////////////

bool latlon2UTM(double latDeg, double longDeg, int zone, double * easting, double *  northing)
{
  if (true)
  {
    // void LLtoUTM(int ReferenceEllipsoid, const double Lat, const double Long,
    //             double &UTMNorthing, double &UTMEasting, char* UTMZone)
    LLtoUTM(23, latDeg, longDeg, northing, easting, zone);
  }
  else
  { // older not-so-good method
    //Internal variables
    double lat = 0, zone_CM = 0, delta_lon = 0; //, lon;

    //Datum constants
    double a = 6378137.0;
    double b = 6356752.314;

    double k0 = 0.9996;    //Scale factor
    double e = sqrt(1-pow((b/a),2)); //Eccentricity
    double e_2 = e*e/(1-e*e);
    double n = (a-b)/(a+b);
    double nu = 0.0;

    //Calcualte Meridional Arc Length
    double A0 = a*(1-n+ (5*n*n/4)*(1-n) + (81*pow(n,4)/64)*(1-n));
    double b0 = (3.0 * a * n / 2.0) * (1 - n - (7.0 * n * n / 8.0) * (1.0 - n) + 55.0 * pow(n,4.0) / 64.0);
    double C0 = (15*a*n*n/16)*(1 - n +(3*n*n/4)*(1-n));
    double D0 = (35*a*pow(n,3)/48)*(1 - n + 11*n*n/16);
    double E0 = (315*a*pow(n,4)/51)*(1-n);
    double S = 0;

    //Calculate constants
    double p = 0, sin_1=0;

    //Coefficients for UTM coordinates
    double Ki = 0, Kii = 0, Kiv = 0, Kv = 0;
    double Kiii = 0;

    if (true)
    {
      lat = latDeg * M_PI / 180.0;
      //lon = longDeg * M_PI / 180.0;
      zone_CM = 6.0 * zone - 183.0;
      delta_lon = longDeg - zone_CM;
      nu = a/sqrt(1.0 - pow((e * sin(lat)),2));

      //Calcualte Meridional Arc Length
      S=A0*lat - b0*sin(2*lat) + C0*sin(4*lat) - D0*sin(6*lat) + E0*sin(8*lat);

      //Calculate constants
      p = delta_lon * 3600.0/10000.0;
      sin_1=M_PI/(180.0 * 3600.0);

      //Coefficients for UTM coordinates
      Ki=S*k0;
      Kii=nu*sin(lat)*cos(lat)*pow(sin_1,2)*k0*1e8/2.0;
      Kiii=((pow(sin_1,4)*nu*sin(lat)*pow(cos(lat),3))/24.0)*(5.0-pow(tan(lat),2)+9.0*e_2*
          pow(cos(lat),2)+4.0*e_2*e_2*pow(cos(lat),4))*k0*1e16;
      Kiv=nu*cos(lat)*sin_1*k0*1e4;
      Kv=pow(sin_1 * cos(lat),3) * (nu / 6.0) * (1-pow(tan(lat),2)+e_2*pow(cos(lat),2))*k0*1e12;
        //Transfer the data from latlon_struct to UTM_struct
      *northing = (Ki + Kii * p * p + Kiii * pow(p, 4));
      *easting = 500000.0 + (Kiv * p + Kv * pow(p,3));
    }
  }
  return true;
}

///////////////////////////////////////////////////////////////////////

bool utm2latlon(double easting, double northing, int zone,
                          double * latitude, double * longitude)
{
  if (true)
    UTMtoLL(23, northing, easting, zone, latitude, longitude);
  else
  { // older not-so-good method
    double x = easting - 500000.0; // signed in meters from central meridian
    double y = northing;
    // center meridian in radians
    double zone_CM = (6.0 * zone - 183.0) * M_PI / 180.0;
    //Datum constants
    double a = 6378137.0;  // WGS84 equatorial radius
    double b = 6356752.314;// polar radius
    double k0 = 0.9996;    //Scale factor - along longitude 0
    double e = sqrt(1.0 - sqr(b) / sqr(a)); // excentricity ~ 0.08
    // meditorial arc
    double M = y/k0;
    // footprint latitude
    double mu = M/(a*(1.0 - sqr(e)/4.0 - 3.0 * pow(e,4) / 64.0 -
          5.0 * pow(e,6) / 256.0));
    double e1 = (1.0 - sqrt(1.0 - sqr(e)))/(1.0 + sqrt(1.0 - sqr(e)));
    double j1 = (3.0 * e1 / 2.0 - 27.0 * pow(e1, 3) / 32.0);
    double j2 = (21.0 * sqr(e1) / 16.0 - 55.0 * pow(e, 4) / 32.0);
    double j3 = (151.0 * pow(e1, 3) / 96.0);
    double j4 = (1097.0 * pow(e1, 4) / 512.0);
    double fp = mu + j1 * sin(2.0 * mu) + j2 * sin(4.0 * mu) +
          j3 * sin(6.0 * mu) + j4 * sin(8.0 * mu);
    // and now lat-long
    double e2 = sqr(e * a / b);
    double c1 = e2 * sqr(cos(fp));
    double t1 = sqr(tan(fp));
    double r1 = a * (1.0 - sqr(e))/pow(1.0 - sqr(e) * sqr(sin(fp)), 3.0/2.0);
    double n1 = a / sqrt(1.0 - sqr(e) * sqr(sin(fp)));
    double d  = x / (n1 * k0);
    double q1 = n1 * tan(fp) / r1;
    double q2 = sqr(d)/2.0;
    double q3 = (5.0 + 3.0 * t1 + 10.0 * c1 - 4.0 * sqr(c1) - 9.0 * e2) *
          pow(d, 4) / 24.0;
    double q4 = (61.0 + 90.0 * t1 + 298.0 * c1 + 45.0 * sqr(t1) -
          3.0 * sqr(c1) - 252.0 * e2) * pow(d, 6) / 720.0;
    double q5 = d;
    double q6 = (1.0 + 2.0 * t1 + c1) * pow(d, 3) / 6.0;
    double q7 = (5.0 - 2.0 * c1 + 28.0 * t1 - 3.0 * sqr(c1) +
          8.0 * e2 + 24.0 * sqr(t1)) * pow(d, 5) / 120.0;
    double lat = fp - q1*(q2 - q3 + q4);
    double lon = zone_CM + (q5 - q6 + q7) / cos(fp);
    //
    *latitude = lat * 180.0 / M_PI;
    *longitude = lon * 180.0 / M_PI;
  }
  //
  return true;
}

//////////////////////////////////////////////////////

const char * getFullFilename(const char * prePath, const char * filename,
                                    char * buffer, const int bufferCnt)
{
  const char * result = buffer;
  bool fullName;
  //
  // should path be appended?
  fullName = (filename[0] == '/') or
      (filename[0] == '.' and (filename[1] == '/' or filename[1] == '.'));
  //
  if (buffer != NULL)
  {
    if (fullName)
    { // filename is fully qualified already
      strncpy(buffer, filename, bufferCnt);
    }
    else
    { // name is relative, so prepend the path
      snprintf(buffer, bufferCnt, "%s/%s", prePath, filename);
    }
  }
  return result;
}


/*Reference ellipsoids derived from Peter H. Dana's website-
             http://www.utexas.edu/depts/grg/gcraft/notes/datum/elist.html
             Department of Geography, University of Texas at Austin
             Internet: pdana@mail.utexas.edu
             3/22/95

             Source
             Defense Mapping Agency. 1987b. DMA Technical Report: Supplement to Department of Defense World Geodetic System
             1984 Technical Report. Part I and II. Washington, DC: Defense Mapping Agency
*/

//LatLong- UTM conversion..h
//definitions for lat/long to UTM and UTM to lat/lng conversions

const double PI = M_PI;
const double FOURTHPI = PI / 4;
const double deg2rad = PI / 180;
const double rad2deg = 180.0 / PI;

class Ellipsoid
{
  public:
    Ellipsoid(){};
    Ellipsoid(int Id, const char* name, double radius, double ecc)
    {
      id = Id; ellipsoidName = name;
      EquatorialRadius = radius; eccentricitySquared = ecc;
    }

    int id;
    const char * ellipsoidName;
    double EquatorialRadius;
    double eccentricitySquared;
};


static Ellipsoid ellipsoid[] =
{//  id, Ellipsoid name, Equatorial Radius, square of eccentricity
  Ellipsoid( -1, "Placeholder", 0, 0),//placeholder only, To allow array indices to match id numbers
  Ellipsoid( 1, "Airy", 6377563, 0.00667054),
  Ellipsoid( 2, "Australian National", 6378160, 0.006694542),
  Ellipsoid( 3, "Bessel 1841", 6377397, 0.006674372),
  Ellipsoid( 4, "Bessel 1841 (Nambia) ", 6377484, 0.006674372),
  Ellipsoid( 5, "Clarke 1866", 6378206, 0.006768658),
  Ellipsoid( 6, "Clarke 1880", 6378249, 0.006803511),
  Ellipsoid( 7, "Everest", 6377276, 0.006637847),
  Ellipsoid( 8, "Fischer 1960 (Mercury) ", 6378166, 0.006693422),
  Ellipsoid( 9, "Fischer 1968", 6378150, 0.006693422),
  Ellipsoid( 10, "GRS 1967", 6378160, 0.006694605),
  Ellipsoid( 11, "GRS 1980", 6378137, 0.00669438),
  Ellipsoid( 12, "Helmert 1906", 6378200, 0.006693422),
  Ellipsoid( 13, "Hough", 6378270, 0.00672267),
  Ellipsoid( 14, "International", 6378388, 0.00672267),
  Ellipsoid( 15, "Krassovsky", 6378245, 0.006693422),
  Ellipsoid( 16, "Modified Airy", 6377340, 0.00667054),
  Ellipsoid( 17, "Modified Everest", 6377304, 0.006637847),
  Ellipsoid( 18, "Modified Fischer 1960", 6378155, 0.006693422),
  Ellipsoid( 19, "South American 1969", 6378160, 0.006694542),
  Ellipsoid( 20, "WGS 60", 6378165, 0.006693422),
  Ellipsoid( 21, "WGS 66", 6378145, 0.006694542),
  Ellipsoid( 22, "WGS-72", 6378135, 0.006694318),
  Ellipsoid( 23, "WGS-84", 6378137, 0.00669438)
};


void LLtoUTM(int ReferenceEllipsoid, const double Lat, const double Long,
             double * UTMNorthing, double * UTMEasting, int zone)
{
//converts lat/long to UTM coords.  Equations from USGS Bulletin 1532
//East Longitudes are positive, West longitudes are negative.
//North latitudes are positive, South latitudes are negative
//Lat and Long are in decimal degrees
//Written by Chuck Gantz- chuck.gantz@globalstar.com
// from http://www.gpsy.com/gpsinfo/geotoutm/
// changed to forced zone number by Christian Andersen DTU

  double a = ellipsoid[ReferenceEllipsoid].EquatorialRadius;
  double eccSquared = ellipsoid[ReferenceEllipsoid].eccentricitySquared;
  double k0 = 0.9996;

  double LongOrigin;
  double eccPrimeSquared;
  double N, T, C, A, M;

//Make sure the longitude is between -180.00 .. 179.9
  double LongTemp = (Long+180)-int((Long+180)/360)*360-180; // -180.00 .. 179.9;

  double LatRad = Lat*deg2rad;
  double LongRad = LongTemp*deg2rad;
  double LongOriginRad;
  int    ZoneNumber;

  if (false)
  {
    ZoneNumber = int((LongTemp + 180)/6) + 1;

    if( Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0 )
      ZoneNumber = 32;
    // Special zones for Svalbard
    if( Lat >= 72.0 && Lat < 84.0 )
    {
      if(      LongTemp >= 0.0  && LongTemp <  9.0 ) ZoneNumber = 31;
      else if( LongTemp >= 9.0  && LongTemp < 21.0 ) ZoneNumber = 33;
      else if( LongTemp >= 21.0 && LongTemp < 33.0 ) ZoneNumber = 35;
      else if( LongTemp >= 33.0 && LongTemp < 42.0 ) ZoneNumber = 37;
    }
  }
  else
    ZoneNumber = zone;

  LongOrigin = (ZoneNumber - 1)*6 - 180 + 3;  //+3 puts origin in middle of zone
  LongOriginRad = LongOrigin * deg2rad;

  // compute the UTM Zone from the latitude and longitude
  // sprintf(UTMZone, "%d%c", ZoneNumber, UTMLetterDesignator(Lat));

  eccPrimeSquared = (eccSquared)/(1-eccSquared);

  N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad));
  T = tan(LatRad)*tan(LatRad);
  C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
  A = cos(LatRad)*(LongRad-LongOriginRad);

  M = a*((1       - eccSquared/4          - 3*eccSquared*eccSquared/64    - 5*eccSquared*eccSquared*eccSquared/256)*LatRad
      - (3*eccSquared/8       + 3*eccSquared*eccSquared/32    + 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*LatRad)
      + (15*eccSquared*eccSquared/256 + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*LatRad)
      - (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*LatRad));

  *UTMEasting = (double)(k0*N*(A+(1-T+C)*A*A*A/6
      + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
      + 500000.0);

  *UTMNorthing = (double)(k0*(M+N*tan(LatRad)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
      + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)));
  if(Lat < 0)
    *UTMNorthing += 10000000.0; //10000000 meter offset for southern hemisphere
}

char UTMLetterDesignator(double Lat)
{
//This routine determines the correct UTM letter designator for the given latitude
//returns 'Z' if latitude is outside the UTM limits of 84N to 80S
        //Written by Chuck Gantz- chuck.gantz@globalstar.com
  char LetterDesignator;

  if((84 >= Lat) && (Lat >= 72)) LetterDesignator = 'X';
  else if((72 > Lat) && (Lat >= 64)) LetterDesignator = 'W';
  else if((64 > Lat) && (Lat >= 56)) LetterDesignator = 'V';
  else if((56 > Lat) && (Lat >= 48)) LetterDesignator = 'U';
  else if((48 > Lat) && (Lat >= 40)) LetterDesignator = 'T';
  else if((40 > Lat) && (Lat >= 32)) LetterDesignator = 'S';
  else if((32 > Lat) && (Lat >= 24)) LetterDesignator = 'R';
  else if((24 > Lat) && (Lat >= 16)) LetterDesignator = 'Q';
  else if((16 > Lat) && (Lat >= 8)) LetterDesignator = 'P';
  else if(( 8 > Lat) && (Lat >= 0)) LetterDesignator = 'N';
  else if(( 0 > Lat) && (Lat >= -8)) LetterDesignator = 'M';
  else if((-8> Lat) && (Lat >= -16)) LetterDesignator = 'L';
  else if((-16 > Lat) && (Lat >= -24)) LetterDesignator = 'K';
  else if((-24 > Lat) && (Lat >= -32)) LetterDesignator = 'J';
  else if((-32 > Lat) && (Lat >= -40)) LetterDesignator = 'H';
  else if((-40 > Lat) && (Lat >= -48)) LetterDesignator = 'G';
  else if((-48 > Lat) && (Lat >= -56)) LetterDesignator = 'F';
  else if((-56 > Lat) && (Lat >= -64)) LetterDesignator = 'E';
  else if((-64 > Lat) && (Lat >= -72)) LetterDesignator = 'D';
  else if((-72 > Lat) && (Lat >= -80)) LetterDesignator = 'C';
  else LetterDesignator = 'Z'; //This is here as an error flag to show that the Latitude is outside the UTM limits

  return LetterDesignator;
}


void UTMtoLL(int ReferenceEllipsoid, const double UTMNorthing, const double UTMEasting,
             int zone /*const char* UTMZone*/, double * Lat,  double * Long )
{
//converts UTM coords to lat/long.  Equations from USGS Bulletin 1532
//East Longitudes are positive, West longitudes are negative.
//North latitudes are positive, South latitudes are negative
//Lat and Long are in decimal degrees.
//Written by Chuck Gantz- chuck.gantz@globalstar.com
  // from http://www.gpsy.com/gpsinfo/geotoutm/

  double k0 = 0.9996;
  double a = ellipsoid[ReferenceEllipsoid].EquatorialRadius;
  double eccSquared = ellipsoid[ReferenceEllipsoid].eccentricitySquared;
  double eccPrimeSquared;
  double e1 = (1-sqrt(1-eccSquared))/(1+sqrt(1-eccSquared));
  double N1, T1, C1, R1, D, M;
  double LongOrigin;
  double mu, phi1Rad;
  double x, y;
  int ZoneNumber;
//  char* ZoneLetter;
//  int NorthernHemisphere; //1 for northern hemispher, 0 for southern

  x = UTMEasting - 500000.0; //remove 500,000 meter offset for longitude
  y = UTMNorthing;

//  ZoneNumber = strtoul(UTMZone, &ZoneLetter, 10);
  ZoneNumber = zone;
//  if((*ZoneLetter - 'N') >= 0)
//  NorthernHemisphere = 1;//point is in northern hemisphere
//   else
//     NorthernHemisphere = 0;//point is in southern hemisphere

  LongOrigin = (ZoneNumber - 1)*6 - 180 + 3;  //+3 puts origin in middle of zone

  eccPrimeSquared = (eccSquared)/(1-eccSquared);

  M = y / k0;
  mu = M/(a*(1-eccSquared/4-3*eccSquared*eccSquared/64-5*eccSquared*eccSquared*eccSquared/256));

  phi1Rad = mu    + (3*e1/2-27*e1*e1*e1/32)*sin(2*mu)
      + (21*e1*e1/16-55*e1*e1*e1*e1/32)*sin(4*mu)
      +(151*e1*e1*e1/96)*sin(6*mu);
  //phi1 = phi1Rad*rad2deg;

  N1 = a/sqrt(1-eccSquared*sin(phi1Rad)*sin(phi1Rad));
  T1 = tan(phi1Rad)*tan(phi1Rad);
  C1 = eccPrimeSquared*cos(phi1Rad)*cos(phi1Rad);
  R1 = a*(1-eccSquared)/pow(1-eccSquared*sin(phi1Rad)*sin(phi1Rad), 1.5);
  D = x/(N1*k0);

  *Lat = phi1Rad - (N1*tan(phi1Rad)/R1)*(D*D/2-(5+3*T1+10*C1-4*C1*C1-9*eccPrimeSquared)*D*D*D*D/24
      +(61+90*T1+298*C1+45*T1*T1-252*eccPrimeSquared-3*C1*C1)*D*D*D*D*D*D/720);
  *Lat = *Lat * rad2deg;

  *Long = (D-(1+2*T1+C1)*D*D*D/6+(5-2*C1+28*T1-3*C1*C1+8*eccPrimeSquared+24*T1*T1)
      *D*D*D*D*D/120)/cos(phi1Rad);
  *Long = LongOrigin + *Long * rad2deg;

}

/////////////////////////////////////////////////////////////////

const char * stringSep(const char * source)
{
  return stringGet(source, NULL, 0, NULL);
}

/////////////////////////////////////////////////////////////////

const char * stringGet(const char * source, char * buff, const int buffCnt, bool * buffOverflow)
{
  const char * result = NULL;
  bool inString = false;
  bool inEsc = false;
  bool ignoreChar;
  int pos = 0;
  const int MSL = 20;
  char stringCharStack[MSL] = "";
  char * stringChar = stringCharStack;
  const char * next = source;
  //
  if (buffOverflow != NULL)
    *buffOverflow = false;
  while (*next != '\0')
  {
    ignoreChar = false;
    if (inEsc)
      inEsc = false;
    else if (*next == '\\')
    {
      inEsc = true;
      if (stringChar - stringCharStack == 1)
        /* un-escape escaped characters in outhermost string level */
        ignoreChar = true;
    }
    else if (inString)
    {
      if (*next == *stringChar)
      {
        stringChar--;
        if (*stringChar == '\0')
        { // out of string (and all nested parts)
          inString = false;
          ignoreChar = true;
          result = next;
        }
      }
      else if ((*next == '"') or (*next == '\''))
      { // nested string just started
        if ((stringChar - stringCharStack) < MSL - 1)
          /* new string start character not in string nesting overflow */
          stringChar++;
        *stringChar = *next;
      }
    }
    else if ((*next == '"') or (*next == '\''))
    { // string just started
      /* ignore first string start param */
      ignoreChar = true;
      inString = true;
      /* new string start character */
      stringChar++;
      *stringChar = *next;
    }
    else if (isspace(*next))
      // ignore space between substrings
      ignoreChar = true;
    else
      // not in string, and not a string start character
      break;
    if (buff != NULL and not ignoreChar)
    {
      buff[pos] = *next;
      if (pos < buffCnt - 1)
            // skip rest of caharacters if longer than buffer
        pos++;
      else if (buffOverflow != NULL)
        *buffOverflow = true;
    }
    next++;
  }
  if (buff != NULL)
    buff[pos] = '\0';
  if (result == NULL)
    result = next;
  return result;
}

///////////////////////////////////////////////////////

///////////////////////////////////////////////////////

int set_serial(int fd, int speed)
{ // set speed of serial port on this computer
  int result = 1;
  struct termios options;

  if (tcgetattr(fd, &options) == -1)
  {
    perror("set_serial: tcgetattr(fd, &options) ");
    //exit(-1);
    result = 0;
  }
  if (result)
  {
    if (speed == 2400)
    {
      cfsetispeed(&options, B2400);  /*/Set input baudrate */
      cfsetospeed(&options, B2400);  /*/Set output baudrate */
    }
    else if (speed == 4800)
    {
      cfsetispeed(&options, B4800);  /*/Set input baudrate */
      cfsetospeed(&options, B4800);  /*/Set output baudrate */
    }
    else if (speed == 9600)
    {
      cfsetispeed(&options, B9600);  /*/Set input baudrate */
      cfsetospeed(&options, B9600);  /*/Set output baudrate */
    }
    else if (speed == 19200)
    {
      cfsetispeed(&options, B19200);  /*/Set input baudrate */
      cfsetospeed(&options, B19200);  /*/Set output baudrate */
    }
    else if (speed == 38400)
    {
      cfsetispeed(&options, B38400);  /*/Set input baudrate */
      cfsetospeed(&options, B38400);  /*/Set output baudrate */
    }
    else if (speed == 57600)
    {
      cfsetispeed(&options, B57600);  /*/Set input baudrate */
      cfsetospeed(&options, B57600);  /*/Set output baudrate */
    }
    else if (speed == 115200)
    {
      cfsetispeed(&options, B115200);  /*/Set input baudrate */
      cfsetospeed(&options, B115200);  /*/Set output baudrate */
    }
    else if (speed == 230400)
    {
      cfsetispeed(&options, B230400);  /*/Set input baudrate */
      cfsetospeed(&options, B230400);  /*/Set output baudrate */
    }
    else if (speed == 500000)
    {
      cfsetispeed(&options, B500000);  /*/Set input baudrate */
      cfsetospeed(&options, B500000);  /*/Set output baudrate */
    }
    else
    {
      fprintf(stderr, "Trying to set speed of serial port to unsupported %d bit/sec\n", speed);
      fprintf(stderr, "- supports 2400,4800,9600,19200,38400,57600,115200,230400,500000 bit/sec\n");
    }

    /* Enable the receiver and set local mode... */
    options.c_cflag |= (CLOCAL | CREAD);

    /*/ Set to no parity 8 bit,1 stop, 8N1
    //options.c_cflag &= ~PARENB;
    //options.c_cflag &= ~CSTOPB;
    //options.c_cflag &= ~CSIZE; */
    options.c_cflag |= CS8;
    /*
    //options.c_cflag = 0;
    //options.c_cflag |= (CLOCAL | CREAD | CS8);


    //Using raw input mode
    //options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); */
    options.c_lflag = 0;
    /*
    //ignore parity errors
    //options.c_iflag |= IGNPAR; */
    options.c_iflag =0;  /*/Must be zero, change it at own risk */
    /*
    //Set output to raw mode
    //options.c_oflag &= ~OPOST; */
    options.c_oflag = 0;

    /*/Set the new options for the port... */
    if (tcsetattr(fd, TCSANOW, &options) == -1)
    {
      perror("can not set serial port parameters\n");
      //exit(-1);
      result = 0;
    }
  }
  if (result)
    /* flush unread data */
    tcflush(fd, TCIFLUSH);
  //
  return !result;
}

//////////////////////////////////////////

int setDeviceSpeed(const char * devName, int devSpeed)
{
  int result;
  int hfd = -1;
/*  const int MSL = 30;
  char s[MSL];*/
  //
  if (hfd < 0)
  {
    /* hfd = open(SERIAL_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY); */
    printf("open_port: Trying to open port %s\n", devName);

    hfd = open(devName, O_RDWR | O_NOCTTY );
    if (hfd == -1)
    { /* Could not open the port. */
      perror("open_port:: Unable to open port\n");
    }
    else
    { /* fcntl(fd, F_SETFL, FNDELAY);   non blocking by using FNDELAY */
      //fcntl(hfd, F_SETFL, 0);      /* Blocking */
      /* setting serial port speed */
      printf("Setting speed to %d bit/sec\n", devSpeed);
      set_serial(hfd, devSpeed);
      tcflush(hfd, TCIFLUSH); /* flush unread data */
      //
    }
  }
  result = hfd >= 0;
  close(hfd);
  return !result;
}

///////////////////////////////////////////////////////////////////

int wildcmp(const char *wild, const char *string) {
  // Written by Jack Handy - <A href="mailto:jakkhandy@hotmail.com">jakkhandy@hotmail.com</A>
  const char *cp = NULL, *mp = NULL;

  while ((*string) && (*wild != '*')) {
    if ((*wild != *string) && (*wild != '?')) {
      return 0;
    }
    wild++;
    string++;
  }

  while (*string) {
    if (*wild == '*') {
      if (!*++wild) {
        return 1;
      }
      mp = wild;
      cp = string+1;
    } else if ((*wild == *string) || (*wild == '?')) {
      wild++;
      string++;
    } else {
      wild = mp;
      string = cp++;
    }
  }

  while (*wild == '*') {
    wild++;
  }
  return !*wild;
}

//////////////////////////////////////////////////////////////////////

void fswap(double * a, double * b)
{
  double c = *a;
  *a = *b;
  *b = c;
}

