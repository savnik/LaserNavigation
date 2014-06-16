/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *  $Rev: 1969 $
 *  $Id: ucommon.h 1969 2012-08-07 09:09:17Z jca $
 *
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

#ifndef UCOMMON_H
#define UCOMMON_H


//#define OPENCV2
#ifdef OPENCV2
#include <opencv2/core/core_c.h>
#include <opencv2/core/core.hpp>
#else
#include <opencv/cxcore.h>
#endif

#include <math.h>
#include <stdio.h>
//
/**
Size used in as image basename */
#define MAX_IMG_NAME_SIZE 100
/**
Size to store a file extension (without the '.')
i.e max 4 characters */
#define MAX_EXT_SIZE 5
/**
Standard size for error and information strings
Should at least be 80 characters */
#define MAX_VINFO_SIZE 250

/**
When a filename buffer is needed this is the
preferred minimum size. */
#define MAX_FILENAME_SIZE 500

/**
Color coding for image is black and white (Y only) */
#define PIX_PLANES_BW  0
/**
Color coding is in RGB order */
#define PIX_PLANES_RGB 1
/**
Color coding is in BGR order */
#define PIX_PLANES_BGR 2
/**
Color coding is in YUV order */
#define PIX_PLANES_YUV 3
/**
16 bit (signed) format - may be converted back to
8 bit BW */
#define PIX_PLANES_BW16S  4
/**
16 bit (signed) format - may be converted back to
8 bit BW */
#define PIX_PLANES_YUV420  5
/**
Color coding is in RGB order - alligned to integer size
NB! not compatible with UPixel size. */
#define PIX_PLANES_RGBA 6
/**
Color coding is in BGR order - alligned to integer size
NB! not compatible with UPixel size. */
#define PIX_PLANES_BGRA 7
/**
Color coding is Bayer coded RGGB order - sized as one-plane 8 bit BW image. */
#define PIX_PLANES_RGGB 8
/**
Color coding is Bayer coded BGGR order  - sized as one-plane 8 bit BW image. */
#define PIX_PLANES_BGGR 9
/**
Color coding is Bayer coded GBRG order  - sized as one-plane 8 bit BW image. */
#define PIX_PLANES_GBRG 10
/**
Color coding is Bayer coded GRBG order  - sized as one-plane 8 bit BW image. */
#define PIX_PLANES_GRBG 11
/**
Color coding is Bayer but treated as one-plane 8 bit BW image - original coding is lost. */
#define PIX_PLANES_BAYER 12
/**
16 bit (unsigned) format - may be converted back to
8 bit BW */
#define PIX_PLANES_BW16U 13
/**
 * color coding YUV422, as used as default image format in new v4l2 plugin */
#define PIX_PLANES_YUV422 14

/**
Size of longest filepath allowed */
#define MAX_PATH_LENGTH MAX_FILENAME_SIZE

/// max camera mount-name size
// (used by both ucam4 and client side)
#define MAX_MOUNT_NAME_SIZE 20

/**
Path where to save images */
extern char imagePath[MAX_PATH_LENGTH];
/**
Path where to save datafiles other than images (e.g. logfiles) */
extern char dataPath[MAX_PATH_LENGTH];
/**
Path where to expect replay logfiles */
extern char replayPath[MAX_PATH_LENGTH];
/**
Default value for server port, that can be set
by a -p or --port command line parameter */
extern int serverPort;
/**
Default server host name.
Intended for use by client type applications */
//extern char serverName[MAX_PATH_LENGTH];
/**
Default server host name.
Intended for use by client type applications */
extern const char * appName;
/**
 * Copy of application parameters */
extern int s_argc;
extern char ** s_argv;
/**
Test image and data path */
extern void testPathSettings();
/**
 * copy ini-files to local path if ini-script is not found */
extern void getIniFiles(const char * iniScript);

/**
Set command line options: -h (help),
-a (run as daemon), -s (ini script), -p (serverPort),
-i (imagePath) and -d (dataPath).
Also sets openCV to NOT throw an exception on errors (may crash instead, but a coredump will then created and may be traceable)
\param argc is command line argument count
\param argv is command line argument values
\param script is buffer for script file name
\param scriptCnt is size of buffer for script file name
\param daemon is pointer to daemon flag
\param defaultServerPort is default server port for this server
\param appHelpPreText is pretekst written before start of help text for command line options
\returns true if help is requested, else false. */
extern bool setCommonPathAndOtherOptions(int argc, char *argv[],
           char * script, int scriptCnt, bool * daemon,
           int defaultServerPort, const char * appHelpPreText);

/**
 * Expand the filename to a full path if not already specified as a full path.
 * The returned file is intended to be used in a fopen(.) statement.
 * \param prePath the path to be prepended if filename is not explicit
 * \param filename specific tile, if it starts with './', '/' or '..' then
 * the datapath is not prepended.
 * \param buffer if a buffer where to save the full filepath.
 * \param bufferCnt is the size of the buffer
 * \returns a pointer to the resulting filename. */
extern const char * getFullFilename(const char * prePath, const char * filename,
                                    char * buffer, const int bufferCnt);

/**
Some simple in-line calculations */
inline double mind(const double a, const double b)
         { if (a < b) return a; else return b; }
inline int mini(const int a, const int b)
         { if (a < b) return a; else return b; }
inline float minf(const float a, const float b)
         { if (a < b) return a; else return b; }
inline float maxf(const float a, const float b)
         { if (a > b) return a; else return b; }
inline int maxi(const int a, const int b)
         { if (a > b) return a; else return b; }
inline double maxd(const double a, const double b)
         { if (a > b) return a; else return b; }
inline int roundi(const double v)
         { if (v > 0) return int(v + 0.5); else  return int(v - 0.5);}
inline unsigned long roundu(const double v)
         { if (v > 0) return (unsigned long)(v + 0.5); else  return (unsigned long)(v - 0.5);}
inline double absd(const double v)
         { if (v < 0) return -v; else  return v; }
inline float absf(const float v)
         { if (v < 0) return -v; else  return v; }
inline int absi(const int v)
         { if (v < 0) return -v; else  return v; }
inline int signofi(const int v)
         { if (v >= 0) return 1; else return -1;}
inline double signofd(const double v)
         { if (v >= 0.0) return 1.0; else return -1.0;}
inline float signoff(const float v)
         { if (v >= 0.0) return 1.0; else return -1.0;}
inline int limiti(int v, int min, int max)
{ if (v < min) return min; else if (v > max) return max; return v; }
inline double limitd(double v, double min, double max)
{ if (v < min) return min; else if (v > max) return max; return v; }
inline float limitf(float v, float min, float max)
{ if (v < min) return min; else if (v > max) return max; return v; }
/**
Square of double argument */
inline double sqr(const double v)
         { return v * v; }
/**
Square of integer argument */
inline int sqri(const int v)
         { return v * v; }
/**
Square of long integer argument */
inline long sqrl(const long v)
         { return v * v; }

/**
Ensure this angle 'v' is within plus/minus PI. */
inline double limitToPi(double v)
{
  double result = v;
  if (fabs(result) > 1e6)
    // insane value, but avoids endless loop
    result = 0.0;
  else
  {
    while (result < -M_PI)
      result += 2.0 * M_PI;
    while (result > M_PI)
      result -= 2.0 * M_PI;
  }
  return result;
}

/**
Ensure this angle 'v' is within plus/minus PI. */
inline double limitTo2Pi(double v)
{
  double result = v;
  if (fabs(result) > 1e6)
    // insane value, but avoids endless loop
    result = 0.0;
  else
  {
    while (result < 0.0)
      result += 2.0 * M_PI;
    while (result > 2.0*M_PI)
      result -= 2.0 * M_PI;
  }
  return result;
}

/**
Pack a data stream using ZLIB.
Packs the bytes in the source - total 'sourceCnt' bytes.
The packed data stream in destBuf (max destBufSize).
Reurns number of used data in destination.
Returns -1 if packing fails. */
extern int packZlib(char * source, int sourceCnt, char * destBuf, int destBufSize);

/**
Unpack ZLIB packed data.
The packed data must be in 'source' in total 'sourceCnt' bytes.
The unpacked data is placed in 'destBuffer' of size (destBufCnt).
Returns number of used bytes in destination buffer.
Returns -1 if unpack fails. */
extern int unpackZlib(char * source, int sourceCnt, char * destBuffer, int destBufCnt);

/**
Special keypress values */
enum MOOS_KEYS {MOOS_PAGEUP = 53, MOOS_PAGEDOWN = 54, MOOS_UP = 65,
           MOOS_DOWN = 66, MOOS_RIGHT = 67, MOOS_LEFT = 68, MOOS_HOME = 72, MOOS_END = 70,
           MOOS_DEL = 51, MOOS_INS = 50,
           MOOS_SPACE = 32, MOOS_OTHER = 63, MOOS_UNKNOWN = 64};
/** Taken from MOOSGenLibGlobalHelper by Paul Newman
     via Søren & Rufus.
     gets one character (or keycode) from console.
     returns ASCII code if normal key.
     returns 126 if special key, then if
       27 (esc), then
         91 escape code 2, then
           e.g. arrow keys. */
extern int MOOSGetch();
/**
Get a combined keypress from console.
Returns the number of bytes (call to MOOSGetch())
that were needed to do the job.
The normal 7-bit keys are in nkey and
escape keys (UP DOWN etc.) are returned in ckey. */
extern int getKey(int * nkey, int * ckey);

/**
 Returns t value for the closest point of [E,H,N] on the
 parameterised line [A,B,C]*t + [E0,H0,N0]
 (assumes [A,B,C] is a unit vector). */
inline double PositionOnEHNLine(const double E, const double H, const double N,
                       const double A, const double B, const double C,
                       const double E0, const double H0, const double N0)
{
  return -(A * (E0 - E) + B * (H0 - H) + C * (N0 - N));
}

/**
Distance from 3D line. Calculates the minimum distance from
a 3D point [E,H,N] to a 3D line from point [E0,H0,N0] with vector [A,B,C].
NB! (A^2 + B^2 + C^2) is assumed to be 1! (as returned by cluster to line).
Returns distance (always positive). */
double DistanceSqFromEHNLine(const double E, const double H, const double N,
                       const double A, const double B, const double C,
                       const double E0, const double H0, const double N0);
/**
Same as DistanceSqFromEHNLine, but returns the distance, not the squared
distance. */
inline double DistanceFromEHNLine(const double E, const double H, const double N,
                       const double A, const double B, const double C,
                       const double E0, const double H0, const double N0)
{
  return sqrt(DistanceSqFromEHNLine(E, H, N, A, B, C, E0, H0, N0));
}
/**
 Find line parameters from two points (x1,y1) and (x2,y2)
 line equation is Ax + By + C = 0.
 Returns -1 if points are not separated (within precition) else 0. */
int pointsToLine(const float x1, const float y1,
                 const float x2, const float y2,
                 float * A, float * B, float * C);
/**
 Find line parameters from two points (x1,y1) and (x2,y2)
 line equation is Ax + By + C = 0.
 Returns -1 if points are not separated (within precition) else 0. */
int pointsToLine(const double x1, const double y1,
                 const double x2, const double y2,
                 double * A, double * B, double * C);
/**
Signed distance from point to 2D line, to be used for crossing
calculations.
A and B must not both be zero. */
inline float DistanceFrom2DlineSigned(const float x, const float y,
                 const float A, const float B, const float C)
{
  return (A*x + B*y + C)/sqrt(sqr(A) + sqr(B));
}
/**
Signed distance from point to 2D line, to be used for crossing
calculations.
A and B must not both be zero. */
inline double DistanceFrom2DlineSigned(const double x, const double y,
                 const double A, const double B, const double C)
{
  return (A*x + B*y + C)/sqrt(sqr(A) + sqr(B));
}
/**
Unsigned (absolute) distance from point to 2D line, to be used for crossing
calculations.
A and B must not both be zero. */
inline float DistanceFrom2Dline(const float x, const float y,
                 const float A, const float B, const float C)
{
  return absf((A*x + B*y + C)/sqrt(sqr(A) + sqr(B)));
}
/**
Unsigned (absolute) distance from point to 2D line, to be used for crossing
calculations.
A and B must not both be zero. */
inline double DistanceFrom2Dline(const double x, const double y,
                 const double A, const double B, const double C)
{
  return absd((A*x + B*y + C)/sqrt(sqr(A) + sqr(B)));
}

/**
Is this value within or at the limits */
inline bool isInside(double x, double minX, double maxX)
{ return (x >= minX) and (x <= maxX); }

/**
Has these 2 squares an overlap */
extern bool hasOverlap(double minX1, double minY1,
                      double maxX1, double maxY1,
                      double minX2, double minY2,
                      double maxX2, double maxY2);

/**
Wait (suspend) for a minimum wait time 'secs' in
seconds and decimal seconds to about 1 milisec,
(resolution at least 0.1 msec) */
extern void Wait(float secs);

////////////////////////////////////////////////

/**
Returns true if character is in range 0..9, a..z, A..Z,
else false */
extern bool isAlphaNum(const char c);
/**
Returns true if alphanumeric or one of "_-/+~" */
extern bool isFileNameChar(const char c);
/**
Convert source string so that only 'isFileNameChar()' characters is used.
At maximum 'bufLen - 1' characters is converted. Destnation string is
zero terminated. Source and destination may be the same.
Returns true if successful. */
extern bool convertToFilename(char * dest, const int bufLen, const char * source);
/**
Convert a string to upper case using the character function toupper */
extern char * strnupper(char * dest, char * source, int destCnt);
/**
Convert a string to lower case using the character function tolower */
extern char * strnlower(char * dest, char * source, int destCnt);

/**
 * \brief find the end of the string starting at (or after som whitespace) source
 * the string must start with a ' or a ".
 * Nested strings with alternating use of these two string characters are allowed.
 * Single escape characters starting with a '\' are allowed, and will be 'unescaped'
 * (escape caracter removed) if in the outher most level of strings.
 * String may be divided over several lines, where the string at ended before end of line and restarted
 * at the new line with a new start string character (optionally after some whitespace).
 * \param source is the source string,
 * \param buff is an optional destination string, where the 'unpacked' concatenated string is assembled
 * without the outhermost string characters.
 * \param buffCnt is the length of the destination buffer.
 * \param buffOverflow is an optional overflow flag, that is set true if the destination buffer is
 * too small for the resultion string.
 * \returns a pointer to the last string end character that terminates the string, or NULL
 * if no string start character is found (that is not white space). */
extern const char * stringGet(const char * source, char * buff, const int buffCnt, bool * buffOverflow);

/**
 * \brief find the end of the string starting at (or after som whitespace) source
 * the string must start with a ' or a ".
 * Nested strings with alternating use of these two string characters are allowed.
 * Single escape characters starting with a '\' are allowed, and will be 'unescaped'
 * (escape caracter removed) if in the outher most level of strings.
 * String may be divided over several lines, where the string at ended before end of line and restarted
 * at the new line with a new start string character (optionally after some whitespace).
 * Calls internally stringGet(...).
 * \param source is the source string,
 * \returns a pointer to the last string end character that terminates the string, or NULL
 * if no string start character is found (that is not white space). */
extern const char * stringSep(const char * source);

////////////////////////////////////////////////

/**
Save a boolean value in html-like format.
The value is saved to file fmap tn format:
\<key=value> e.g. if key is "valid" and value is false
then "\<valid=false>" is saved.
No linefead at the end.
Returns true if saved. */
extern bool saveBool(FILE * fmap, const char * key, bool value);

/**
Analyze a HTML string in buffer for start "<", equal sign "=" and end ">" of field.
The pointers point at the delimiter character, or NULL in none is found.
Returns true if an end delimitor ">" is found. */
extern bool getBlockData(const char * buff, char ** start,
                  char ** equal, char ** end);

/**
Return a boolean as a string */
extern const char * bool2str(bool value);

/**
Return a boolean value of a string.
Returns true if first 4 non-space characters are
'true', otherwise false.
Function is not case-sensitive. */
extern bool str2bool(const char * value);

/**
Return a boolean value of a string.
It tests the first non-space caharacters for either 'true' or 'false'
if neither of these, then the default value 'def' is returned
\param value is a string with the boolean value
\param def is the default value, result is either this default value or the value contains "true" or "false".
\returns found boolean value */
extern bool str2bool2(const char * value, bool def);

/**
Return a boolean value of a string.
It tests the first non-space caharacters for either 'true' or 'false'
if neither of these, then the default value 'def' is returned
  \param value is a string with the boolean value
  \param def is the default value, result is either this default value or the value contains "true" or "false".
  \param p2 is an optional pointer to a char pointer to the first unused character. This is the same as 'value' is value is neither true nor false.
  \returns found boolean value */
extern bool str2bool3(const char * value, bool def, const char ** p2 = NULL);

////////////////////////////////////////////////
////////////////////////////////////////////////

/**
Convert two hex characters to an (unsigned) integer value.
expects msn (most significant nipple) and lsn (least significant nipple)
to have one of the following values "0123456789abcdef", otherwise
an unpredicted result may occur. */
extern int hex2int(char msn, char lsn);

////////////////////////////////////////////////
////////////////////////////////////////////////

/**
The URRgb class holds pixel values in floating point (float)
but has no methods (more like a struct). */
class URRgb
{ // values for one pixel near epipolar line
public:
  float r,g,b; // pixel values (converted to float)
};

////////////////////////////////////////////////

/**
Find if this string is one of the strings in the stringlist */
extern  bool inThisStringList(const char * str, const char * strList);

/**
Match a string against a pattern that includes wildcards - * and ? are allowed anywhere.
\param string full string that may matsch the pattern.
\param pattern is the match pattern that may hold any number of *? wildcards.
\returns true if strig matches pattern. */
extern bool pattern_match(const char *str, const char *pattern);


////////////////////////////////////////////////
////////////////////////////////////////////////
// pixel in an image
/**
The pixel class is a 24 bit color code in r,g,b or y,u,v format.
Each color is an unsigned char.
  NB! first channel may be red (and thus not blue),
  if converted to RGB and not BGR, as
  is the default coding in openCV */
class UPixel
{
public:
  /**
  first byte (plane 1) is either blue (b) or intensity (y)
  NB! may be red, if converted to RGB and not BGR, as
  isthe default coding in openCV */
  union
  {
    //unsigned char b;
    unsigned char y;
    unsigned char p1;
  };
  /**
  Second byte is either green (g) or u component of
  heu-saturation in rectangluar coordinates */
  union
  {
    //unsigned char g;
    unsigned char u; // should be signed range +/- 128
    unsigned char p2;
  };
  /**
  Third byte is either blue (b) or v color component */
  union
  {
    //unsigned char r;
    unsigned char v;
    unsigned char p3;
  };
  /**
  Constructor */
  UPixel();
  /**
  Constructor from rgb values */
  UPixel(unsigned char ir,
            unsigned char ig,
            unsigned char ib);
  /**
  Destructor */
  ~UPixel();
  /**
  Clear all channels to zero.*/
  void clear();
  /**
  Convert pixle from RGB to BGR or the other way. */
  inline void swapRB()
  {
    unsigned char rb;
    rb = p1;
    p1 = p3;
    p3 = rb;
  }
  /**
  Return pixel as cvScalar for cv library use */
  CvScalar cvRGB()
    { return CV_RGB(p3, p2, p1);};
  /**
   * convert from integer representation to openCV RGB structure */
  inline static CvScalar toCvRGB(int pix)
  {
    return CV_RGB(pix & 0xff, (pix >> 8) & 0xff, (pix >> 16) & 0xff);
  }
  /**
  Set pixel value in this order */
  inline void set(unsigned char channel1,
                  unsigned char channel2,
                  unsigned char channel3)
  { p1 = channel1; p2 = channel2; p3 = channel3;};
  /**
  Return a pixel with these RGB colors */
  static UPixel pixRGB(unsigned char p1, unsigned char p2, unsigned char p3)
    {
      UPixel result(p1, p2, p3);
      return result;
    };
  /**
  Convert a pixel from YUV to RGB format */
  inline static UPixel YUVtoRGB(unsigned char iy,
                      unsigned char iu,
                      unsigned char iv)
  {
    UPixel result;
    // converion from file:
    // ... camstream/camstream-0.26pre2/lib/ccvt/ccvt_c1.c
    // R = Y +                0. (V - 128)
    // G = Y - 0. (U - 128) - 0. (V - 128)
    // B = Y + 1.499 (U - 128)
    int yy, uu ,vv;
    yy = iy << 8;
    uu = iu - 128;
    vv = iv - 128;
    // red
    result.p1 = LIMIT_RGB((yy            + 359 * vv) >> 8);
    // green
    result.p2 = LIMIT_RGB((yy -  88 * uu - 183 * vv) >> 8);
    // blue
    result.p3 = LIMIT_RGB((yy + 454 * uu           ) >> 8);
    return result;
  };
  /**
  Convert a pixel from YUV to BGR format */
  inline static UPixel YUVtoBGR(unsigned char iy,
                      unsigned char iu,
                      unsigned char iv)
  {
    UPixel result;
    int yy, uu ,vv;
    yy = iy << 8;
    uu = iu - 128;
    vv = iv - 128;
    // red
    result.p3 = LIMIT_RGB((yy            + 359 * vv) >> 8);
    // green
    result.p2 = LIMIT_RGB((yy -  88 * uu - 183 * vv) >> 8);
    // blue
    result.p1 = LIMIT_RGB((yy + 454 * uu           ) >> 8);
    return result;
  };
  /** convert to RGB format (non standard byte order) */
//   static inline UPixel YUVtoRGB(unsigned char iy,
//                       unsigned char iu,
//                       unsigned char iv)
//      {
//        UPixel result;
//        result = YUVtoBGR(iy, iu, iv);
//        result.swapRB();
//        return result;
//      }
  /**
  Convert a pixel from RGB to YUV format.
  Expects parameter order as (Red, Green, Blue) */
  static inline UPixel RGBtoYUV(unsigned char ir, unsigned char ig, unsigned char ib)
  { // modified to new calculation
    UPixel result;
    /*
      65536*(y-16)  = 19592*R + 38480*G +  7468*B
      65536*(U-128) = 28787*R - 24107*G -  4674*B
      65536*(V-128) = -9713*R - 19068*G + 28787*B
    */
    int mmY, mmU, mmV;
    //
    mmY = ((19592 * ir + 38480 * ig +  7468 * ib) >> 16);
    mmU = ((28787 * ir - 24107 * ig -  4674 * ib) >> 16) + 128;
    mmV = ((-9713 * ir - 19068 * ig + 28787 * ib) >> 16) + 128;
    // NB! U and V are exchanged relative to calculated values
    result.y = LIMIT_RGB(mmY);
    result.v = LIMIT_RGB(mmU);
    result.u = LIMIT_RGB(mmV);
    return result;
  }
  /**
  Set pixel in RGB format from specified YUV format */
  void setYUVto(const unsigned char iy,
                const unsigned char iu,
                const unsigned char iv,
                const int toFormat /*= PIX_PLANES_BGR*/);
  /**
  Set pixel in YUV format from specified RGB values */
  void setRGBto(const unsigned char ir,
                const unsigned char ig,
                const unsigned char ib,
                const int toFormat /*= PIX_PLANES_BGR*/);
  /**
  Sets to a palette serial color out of palSize available colors */
  void set(int serial, int palSize);
  /**
  From real color values. */
  void set(URRgb rrgb);
  /**
  Get color diatance to this pixel (3D og RGB basis) */
  float colorDist(UPixel * pix);
  /**
  Get distance in IHS space. */
  float colorDistIHS(UPixel * pix);
  /**
  Set pixel value using a color palette with 8 colors
  corresponding to 8 major directions in color space.
  Intensity is default maksimum.
  0 is red, 1 is yellow, 2 is yellowgreen, 3 is green, 4 is cyan, 5 is blue
  6 magenta, 7 is purpel-red.
  Pixel is in YUV scheme. */
  void setYUVPix8(int colorIndex, int intensity = 255);
  /**
  Set pixel value using a color palette with 8 colors
  corresponding to 8 major directions in color space.
  Intensity is default maksimum.
  0 is red, 1 is yellow, 2 is yellowgreen, 3 is green, 4 is cyan, 5 is blue
  6 magenta, 7 is purpel-red.
  Pixel is in RGB scheme. */
  void setRGBPix8(int colorIndex, int intensity = 255);
  /**
  Tone pixel between this and 'other' value for 'pct' of this and the rest
  from the 'other' value. */
  void tone(UPixel other, int pct);
  /**
  Tone pixel between this and 'other' value for 'pct' of this and the rest
  from the 'other' value.
  Returns the toned resuld without changing this
  pixel */
  UPixel toned(UPixel other, int pct);
  /**
  Returns a pixel, where this YUV pixel is converted to RGB.
  there is no test to se if source is realy YUV. */
  UPixel asRGB(int fromFormat);
  /**
  Returns a pixel, where this YUV pixel is converted to RGB.
  there is no test to se if source is realy YUV. */
  UPixel asBGR(int fromFormat);
  /**
  Returns a pixel, where this RGB pixel is converted to YUV.
  there is no test to se if source is realy RGB. */
  UPixel asYUV(int fromFormat);
  /**
  Return a cromatisity pixel version of this pixel
  assuming that this pixel is in format 'fromFormat' */
  UPixel asCromaBGR(int fromFormat);
  /**
  Reduce range to 0..255*/
  static inline unsigned char LIMIT_RGB(int x)
    { return (unsigned char)(((x) < 0) ? 0 : (((x) > 255) ? 255 : (x)));};
  /**
  Print pixel values to console with
  this prestring */
  void print(char * prestring);
  /**
  Get green value assuming color format is as specified.
  Reurns 0 if colorformat is unknown. */
  unsigned char getGreen(int fromFormat);
  /**
  Get blue value assuming color format is as specified.
  Reurns 0 if colorformat is unknown. */
  unsigned char getBlue(int fromFormat);
  /**
  Get red value assuming color format is as specified.
  Reurns 0 if colorformat is unknown. */
  unsigned char getRed(int fromFormat);
  /**
  Get sum of planes */
  inline int getSum()
  { return p1 + p2 + p3; };
  /**
  Convert gray level to color where 255 towards 0 becomes white->red->green->blue->magenta->dark->black */
  UPixel grayToColor(unsigned char gray)
  { return grayToColorInv(255 - gray); };
  /**
  Convert gray level to color 0 towards 255 becomes white->red->green->blue->magenta->dark->black */
  UPixel grayToColorInv(unsigned char gray)
  {
    int msb = gray >> 5;
    int lsb = (gray & 0x1f) << 3;
    switch (msb)
    {
    case 0: // white to red
            p1 = 255;      p2 = 255-lsb;  p3 = 255-lsb;  break;
    case 1: // red to yellow
            p1 = 255;      p2 = lsb;      p3 = 0;        break;
    case 2: // yellow to green
            p1 = 255-lsb;  p2 = 255;      p3 = 0;        break;
    case 3: // green to cyan
            p1 = 0;        p2 = 255;      p3 = lsb;      break;
    case 4: // cyan to blue
            p1 = 0;        p2 = 255-lsb;  p3 = 255;      break;
    case 5: // blue to magenta
            p1 = lsb;      p2 = 0;        p2 = 255;      break;
    case 6: // magenta to (light) gray
            p1 = 255 - lsb/3;      p2 = (lsb * 2) / 3;   p2 = 255 - lsb/3;  break;
    default: // (light) gray to black
            int v = 171 - (lsb * 2) / 3;
            p1 = v;  p2 = v;  p3 = v;  break;
    }
    return *this;
  };
  /**
  Get pixel as integer value 0xRRGGBB */
  inline unsigned int getInt()
  { return (p1 << 16) + (p2 << 8) + p3; };

};


////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////

/**
Class for calculation on pixel in IHS and IUV format.
All vaiables are in float in the range:
Saturation (i): [0..1[,
u and v: [-1..1] (approx),
hue: -Pi..Pi,
sat: [0..1]. */
class UIUVPixel
{
public:
  float i;
  float u;
  float v;
  float hue;
  float sat;
  /**
  Constructor */
  UIUVPixel();
  /**
  Constructor from RGB pixel */
  UIUVPixel(UPixel * pix);
  /**
  Set pixel from RGB pixel */
  void SetPix(UPixel pix);
  /**
  Set pixel from float version of an RGB pixel */
  void SetPix(URRgb pix);
  /**
  Color distance to this pixel
  black to white is 1 */
  float colorDist(UIUVPixel * pix);
};

///////////////////////////////////////////////////////////////

  /**
  \brief Function to transform from Lat-Long to UTM in a given zone.
 *
 * The algorithm is supplied by Anders Gaasedal and is also used in
 * his truck control project.
 *
 * \attention This algorithm is supplied as is. There is no guarantee that
 * is works consistently and without faults. As can be seen from the code
 * there are several unused variables, which there is no use for. This
 * does not give much confidence in the code, but no errors has been seen
 * when parsing messages.
  - source is: ????? @todo find source
  \param [in] latitude as calculated (in degrees)
  \param [in] longitude as calculated (in degrees)
  \param [in] zone of central meridian
  \param [out] easting  (pointer to double) in meter relative to central median (+ 500km)
  \param [out] northing (pointer to double) in meter from equator
   */

extern bool latlon2UTM(double latDeg, double longDeg, int zone,
                       double * easting, double *  northing);

  /**
  Conversion from UTM in a zone to lat-long.
  Produces (almost) the inverse of latlong2utm(...).
  - source is: ????? @todo find source
  \param [in] easting in meter relative to central median (+ 500km)
  \param [in] northing in meter from equator
  \param [in] zone of central meridian
  \param [out] latitude as calculated (in degrees)
  \param [out] longitude as calculated (in degrees)
  \return true. */
extern bool utm2latlon(double easting, double northing, int zone,
                double * latitude, double * longitude);

/**
 * Conversion from LatLong to UTM
 * \param ReferenceEllipsoid, 23 is using VGS84
 * \param Lat is latitude in decimal degrees
 * \param long is longitude in decimal degrees
 * \param northing is returned UTM northing in meters
 * \param Easting is easting relative to zone line in meters
 * \param zone is the zone used for the reference longitude
 * */
extern void LLtoUTM(int ReferenceEllipsoid, const double Lat, const double Long,
             double * UTMNorthing, double * UTMEasting, int zone);
/**
 * Conversion from UTM to LatLong
 * \param ReferenceEllipsoid, 23 is using VGS84
 * \param northing is UTM northing in meters
 * \param Easting is easting relative to zone line in meters
 * \param zone is integer zone number, assuming northeren hemisphere (see code for southern)
 * \param Lat is returned latitude (in decimal degrees)
 * \param LONg is returned longitude (in decimal meters)
 * */
extern void UTMtoLL(int ReferenceEllipsoid, const double UTMNorthing, const double UTMEasting,
             int zone /*const char* UTMZone*/,
             double * Lat,  double * Long );
/**
 * Convert latitude to single character */
char UTMLetterDesignator(double Lat);

/**
Set speed for a device with string name of device. Uses set_serial() to
set the speed.
\param devName is a string with device name, i.e. '/dev/ttyS0'
\param devSpeed is integer valued speed, i.e. 9600, 38400, 500000.
\returns true (1) if device exist and speed set is attempted. */
extern int setDeviceSpeed(const char * devName, int devSpeed);
/**
Set serial speed for numbered device
\param fd is integer file descriptor for device.
\param speed is integer speed for the setting. Other modes are: 8bit, no parity, 1 stop-bit.
\returns error = 0 if no errors were detected. */
extern int set_serial(int fd, int speed);

/**
 * This is a fast, lightweight, and simple pattern matching function.
 * Written by Jack Handy - <A href="mailto:jakkhandy@hotmail.com">jakkhandy@hotmail.com</A>
 * usage:
 * if (wildcmp("bl?h.*", "blah.jpg")) {
 *   //we have a match!
 * } else {
 *   //no match =(
 * }                                       */
extern int wildcmp(const char *wild, const char *string);

/**
 * swap the values pointed to by the two double pointers.
 * returns nothing fails (crashes) if either pointer is NULL (or invalid) */
extern void fswap(double * a, double * b);

#endif
