/***************************************************************************
 *   Copyright (C) 2006 by DTU (by Christian Andersen, Lars m.fl.)         *
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

#ifndef URESGPS_H
#define URESGPS_H

#include <cstdlib>

#include <urob4/uresvarpool.h>
#include <urob4/uresposehist.h>

/// Conversion factor from knots to m/s
#define KNOTS2M_S (1852.0/3600.0)
//#define KNOTS2M_S 0.5144444444
/// Number of satellites supported by the GPS (12 by the SIRFII chipset)
#define GPS_SATELLITES_SUPPORTED 20
/// Number of satellites in the GPS system
#define GPS_SATELLITES_TOTAL 32
/// GPS max sentence length
#define GPS_SENTENCE_MAX_LENGTH 200

/// GPS status struct to hold the advanced status information about the GPS system
class UGpsStatus : public ULock
{
public:
  /**
  Constructor */
  UGpsStatus()
  {
    clear();
  };
  /**
  Clear the structure */
  void clear();
  /**
  GSA - GPS DOP and active satellites. This sentence provides details on the nature of the fix.
  It includes the numbers of the satellites being used in the current solution and the DOP.
  DOP (dilution of precision) is an indication of the effect of satellite geometry on the
  accuracy of the fix. It is a unitless number where smaller is better.
  For 3D fixes using 4 satellites a 1.0 would be considered to be a perfect number,
  however for overdetermined solutions it is possible to see numbers below 1.0.

  There are differences in the way the PRN's are presented which can effect the ability
  of some programs to display this data. For example, in the example shown below there
  are 5 satellites in the solution and the null fields are scattered indicating that the
  almanac would show satellites in the null positions that are not being used as part of
  this solution.
  Other receivers might output all of the satellites used at the beginning of the sentence
  with the null field all stacked up at the end.
  This difference accounts for some satellite display programs not always being able to
  display the satellites being tracked. Some units may show all satellites that have
  ephemeris data without regard to their use as part of the solution but this is non-standard.

  $GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39

Where:
  GSA      Satellite status
  A        Auto selection of 2D or 3D fix (M = manual)
  3        3D fix - values include: 1 = no fix
  2 = 2D fix
  3 = 3D fix
  04,05... PRNs of satellites used for fix (space for 12)
  2.5      PDOP (dilution of precision)
  1.3      Horizontal dilution of precision (HDOP)
  2.1      Vertical dilution of precision (VDOP)
   *39      the checksum data, always begins with *
   */
  bool parseGPGSA(char * inBuf);
  /**
    GSV - Satellites in View shows data about the satellites that the unit might be able to
    find based on its viewing mask and almanac data. It also shows current ability to track this data.
    Note that one GSV sentence only can provide data for up to 4 satellites and thus there may
    need to be 3 sentences for the full information. It is reasonable for the GSV sentence to contain
    more satellites than GGA might indicate since GSV may include satellites that are not used as part
    of the solution.
    It is not a requirment that the GSV sentences all appear in sequence.
    To avoid overloading the data bandwidth some receivers may place the various sentences in
    totally different samples since each sentence identifies which one it is.

    The field called SNR (Signal to Noise Ratio) in the NMEA standard is often referred to as
    signal strength. SNR is an indirect but more useful value that raw signal strength.
    It can range from 0 to 99 and has units of dB according to the NMEA standard, but the various
    manufacturers send different ranges of numbers with different starting numbers so the values
    themselves cannot necessarily be used to evaluate different units. The range of working values
    in a given gps will usually show a difference of about 25 to 35 between the lowest and
    highest values, however 0 is a special case and may be shown on satellites that are in
    view but not being tracked.

    $GPGSV,2,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75

  Where:
    GSV          Satellites in view
    2            Number of sentences for full data
    1            sentence 1 of 2
    08           Number of satellites in view

    01           Satellite PRN number
    40           Elevation, degrees
    083          Azimuth, degrees
    46           SNR - higher is better
    for up to 4 satellites per sentence
    *75          the checksum data, always begins with *
  */
  bool parseGPGSV(char * inBuf);
  /**
  Get number of visible satellites with a signal strength obove zero */
  int getTrueVisCnt();


public:
   /// Operational mode
  char opr_mode;
   /// Calculation mode
  int mode;
   /// Number of the satellites used to make calculation (GID)
  int satUsed[GPS_SATELLITES_SUPPORTED];
  int satUsedCnt;
   /// Visible satellites with information: Number (GID), elevation (0-90 deg),
   /// azimuth (0-360 deg), Signal to Noise Ratio (1-99 db-Hz)
  int satVisGID[GPS_SATELLITES_TOTAL];
  /// elevation of satelite 0..90 (0 is hirizon and 90 is zenith)
  int satVisElev[GPS_SATELLITES_TOTAL];
  /// azimuth in compas degrees (0..359)
  int satVisAz[GPS_SATELLITES_TOTAL];
  /// Signal to noice ratio 1--99 db-Hz
  int satVisSN[GPS_SATELLITES_TOTAL];
  /// total number of satellites in view
  int satVisCnt;
  /// index during reception of data
  int satVisIdx;
   /// Positional Dillution of Precision (3D)
  double PDOP;
   /// Horizontal Dillution of Precision (In a plane)
  double HDOP;
   /// Vertical Dillution of precision (Height only)
  double VDOP;
  /** Show if the GPS is in SBAS augmentation mode (EGNOS or WAAS)
      from SiRF message */
  int EGNOS;
  /// last message received
  char sentence[GPS_SENTENCE_MAX_LENGTH];
};

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/// GPS UTM struct to hold the GPS fix information in the Universal Transversal Metacore system
class UGpsUTM  : public ULock
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
  void clear();
  /**
  Parse a RSK-GPS proparity message, where data is converted to UTM
  \param [in] string like $PTNL,PJK,123148.30,091807,+6174357.366,N,+707854.368,E,1,07,3.1,EHT+64.037,M*74
  \returns true if message is of this type, and time, dop, and easting,northing in this structure
  */
  bool parsePTNL(char * inBuf);
  /**
   * Parse UTM position using Leika protocold */
  bool parseGPLLK(char * inBuf);

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

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/// GPS latlon struct to hold the GPS fix information in the latitude/lontitude system
class UGpsLatLong  : public ULock
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
  void clear();
  /**
   * Parse the GPRMC message
   * \param[in] *inBuf Pointer to the string to be parsed.
   * \param[in] *lastTime is the last time data were received, to reconstruct date (also for replay).
   * \return true on succes
   */
  bool parseGPRMC(char * inBuf, UTime lastTime);
  /**
   * Parse the GPRMC message
   * \param[in] *inBuf Pointer to the string to be parsed.
   * \param[in] *lastTime is the last time data were received, to reconstruct date (also for replay).
   * \return true on succes
   */
  bool parseGPGGA(char * inBuff, UTime lastTime);

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

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
This is the shared resource class.
It must enherit from the resource base class (or one of its decendent) as shown.

@author Christian Andersen
*/
class UResGps : public UResVarPool
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResGps) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResGps()
  { // set name and version
    setResID(getResClassID(), 200);
    UResGpsInit();
  };
  /**
  Destructor */
  virtual ~UResGps();
  /**
   * Initialize resource */
  void UResGpsInit();
  /**
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "gps"; };
  /**
  Set (or remove) ressource (core pointer needed by event handling) */
  bool setResource(UResBase * resource, bool remove);
  /**
  print status to a string buffer */
  virtual const char * print(const char * preString, char * buff, int buffCnt);
// the above methods are used be the server core and should be present
// the print function is not strictly needed, but is nice.
public:
  // Public functions that this resource provides for all users.
  /**
  Open seial port to GPS */
  void openPort()
  { varOpen->setBool(true); };
  /**
  Is the serial port to the GPS open */
  bool isConnected();
  /**
  Close serial port */
  void closePort()
  { varOpen->setBool(false); };
  /**
  Open (or close) logfile for raw gps data.
  Nothing happens if trying to open an open file or
  close a closed file.
  \param[in] Set parameter to true to open port and false to close  */
  void openLogfile(bool open)
  {
    lock();
    logGps.setLogName(getResID());
    logGps.openLog(open);
    unlock();
  };
  /**
  Is logfile open */
  inline bool isLogGpsOpen()
  { return logGps.isOpen(); };
  /**
  Run the receice loop for the GPS device.
  This call do not return until the threadStop flag is set true. */
  void run();
  /**
  Get serial port name */
  inline const char * getDeviceName()
  { return varDevice->getValues(); };
  /**
  Get serial port speed (bit/sec) - may not be used if USB device. */
  inline int getDeviceSpeed()
  { return varBaud->getInt(); };
  /**
  Get the latest received sentence - may or may not be used. */
  inline const char * getLastSentance()
  { return sentence; };
  /**
  Get the latest lat-long received sentence. */
  inline const char * getLastLatLongSentance()
  { return latLong.sentence; };
  /**
  Get the latest UTM received sentence. */
  inline const char * getLastUtmSentance()
  { return utm.sentence; };
  /**
  Get the latest status received sentence. */
  inline const char * getLastStatusSentance()
  { return status.sentence; };
  /**
  set the serial device filename. */
  inline void setDevice(const char * value)
  { varDevice->setValues(value, 0, true); };
  /**
  set the serial device filename. */
  inline const char * getDevice()
  { return getDeviceName(); };
  /**
  set the serial device speed. */
  inline void setSpeed(int value)
  { varBaud->setInt(value); };
  /**
  get the serial device speed. */
  inline int getSpeed()
  { return varBaud->getInt(); };
  /**
  Get pointer to the UTM pose history structure (or NULL is not available) */
  UResBase * getUtmHist()
  { return utmPose;};
  /**
  Get time of laset received sentence position sentence. */
  inline UTime getLastPositionTime()
  { return getTimeLocal(); };
    /**
  Decode this replay line, that is assumed to be valid.
  \param line is a pointer to the line.
  \returns true if the line is valid and used (step is successfull).
  \returns false if line is not a valid step. */
  virtual bool decodeReplayLine(char * line);

protected:
  /**
  Make the variables that will be available to other plugins */
  void createBaseVar();
  /**
  Save this string to the logfile after a timestamp with current time. */
  void toLog(const char * message);
  /**
  Start read thread
  \return Returns true if the read thread started. */
  bool start();
  /**
  Stop read thread - and wait for thread join */
  void stop();
  /**
  * Receive data from device
  * Returns when a newline '\n' is received, and there may be more data and possibly more messages.
  * in the received data buffer.
  * \param[in] 'buff' is the buffer place where the received data is stored,
  * \param[in] Returns when 'buffCnt' is reached - regardless of data
  * \param[in] Return if timeout ('timeoutSec' sec) has passed with no data.
  * \return the number of characters added to the buffer. If a read error
  * occured, then -1 is returned. */
  int receiveFromDevice(FILE * device,
                               char * buff, int buffCnt, const double timeoutSec);
  /**
  Update global variables from received data */
  void updateVars();

private:
  /**
  Clear all structures in this class */
  void clear();
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
   */
  /**
  Conversion from UTM in a zone to lat-long.
  \param [in] easting in meter relative to central median (+ 500km)
  \param [in] northing in meter from equator
  \param [in] zone of central meridian
  \param [out] latitude as calculated (in degrees)
  \param [out] longitude as calculated (in degrees)
  \return true. */
  /**
  \brief  Parser for the contents of the NMEA strings.
   *
   * Currently GPRMC, GPGGA, GPGSA, GPGSV anf PFST strings are supported.
   *
   * \param[in] *inBuf Pointer to the string to be parsed.
   * \param[in] *tod is the computer time data were received, to be used in utmHistory stack.
   * \return true on validate checksum, else the message is not parsed
  */
  bool parseNMEA(char * inBuf, UTime tod);
  /**
  Get time local time for last (lat-long) message */
  UTime getTimeLocal();
  /**
   * checks if first character is a '$' and ends with a '*xx' sequence.
   * The CR character after the checksum is replaced by a '\0'
   * \param rxBuff is the buffer to check.
   * \param rxBuffCnt is the number of bytes in buffer
   * \param pEnd will be set to just after the message (most likely point to LF
   * character after checksum) if there is a message,
   * if no message is found, then pEnd is unchanged. (pEnd may be NULL).
   * \returns true if at least one NMEA message is available in buffer */
  bool hasNmeaMessage(char * rxBuff, int rxBuffCnt, char ** pEnd);
  /**
   * Removes one NMEA message from input buffer, and if there is more than one, then
   * the start of the new message - the '$' is moved to start of buffer, and the new buffer length is
   * returned.If no new measse is found (no '$') then the buffer is returned empty.
   * \param rxBuff is the buffer to check.
   * \param rxBuffCnt is the number of bytes in buffer.
   * \param pEnd is end of massage to be discarded.
   * \returns number of bytes in new buffer */
  int advanceNmeaMessage(char * rxBuff, int rxBuffCnt, char * pEnd);

protected:
  /**
  check if this is a valid NMEA message checksum is correct.
  \param [in] the sting with the message, of max length GPSSTRINGSIZE.
  \return true if checksum match */
  bool validateNMEA(char* in_buf);
  /**
  \brief Implementation of a ASCII to HEX coverter.
   *
   * Takes both upper and lower case ascii values but has no check for
   * non HEX values.
   * \param[in] input Character input in ascii values
   * \return hex value of the parameter for further processing. */
  char ascii2hex(char input);

protected:
  /**
  Name of logfile */
  char logGpsName[MAX_FILENAME_LENGTH];
  /**
  Thread handle for frame read thread. */
  pthread_t threadHandle;
  /**
  Is thread actually running */
  bool threadRunning;
  /**
  Should thread stop - terminate */
  bool threadStop;
  /**
  Timestamp of the letest received data */
  UTime dataRxTime;
  /**
  Number of full messages received */
  int dataRxMsgCnt;
  /**
  The gps data received from GPS receiver */
  //UGpsData gpsData;
  /**
  Pointer to UTM pose history resource */
  UResPoseHist * utmPose;
  /**
  Pointer to odometry pose history resource */
  UResPoseHist * odoPose;
  /**
  Pointer to Map pose history resource */
  UResPoseHist * mapPose;

public:
  /**
  File handle to GPS log */
  ULogFile logGps;

private:
  /// replay line in NMEA format to replay
  char replaySentence[GPS_SENTENCE_MAX_LENGTH];
  /// is new replay data available
  bool replaySentenceNew;
  /// lock to protect integrity of sentence
  ULock replaySentenceLock;
  /// index to variable with UTM zone
  UVariable * varUTMZone;
  /// Number of visible satellites
  UVariable * varSatsVisible;
  /// Number of used satelites in calculation
  UVariable * varSatsUsed;
  /// Dilution of position 1 is perfect, 5 is fair, >8 is bad
  UVariable * varDop;
  /// Number of fixes per second
  UVariable * varFixRate;
  /// Latitude in decimal degrees, positive is North
  UVariable * varLat;
  /// Longitude in decimal degrees, positive is East
  UVariable * varHeight;
  /// Longitude in decimal degrees, positive is East
  UVariable * varLong;
  ///  Last fix time
  UVariable * varTime;
  ///  Last received GMT time
  UVariable * varGmt;
  ///  Last fix time
  UVariable * varTimes;
  ///  Last received GMT time
  UVariable * varGmts;
  ///  index to Heading received from GPS (compas degrees)
  UVariable * varHeading;
  ///  index to Speed received from GPS [m/s]
  UVariable * varSpeed;
  /// index to UTM Easting in current zone in meter
  UVariable * varEasting;
  /// UTM northing in current zone in meter
  UVariable * varNorthing;
  /// Do automatic conversion to UTM
  UVariable * varMakeUTM;
  /// Maintain UTM-pose with raw UTM coordinates
  UVariable * varUtmPoseKeep;
  /// Maintain Map-pose with UTM coordinates
  UVariable * varMapPoseKeep;
  /// use odometry history to estimate heading for map and utm pose update
  UVariable * varUseOdoHeading;
  /// Map-pose set with UTM coordinates relative to this position [east, north]
  UVariable * varMapPoseRef;
  /// index to EGNOS variable
  UVariable * varEgnos;
  /// index to gps mode (1=noFix 3 = 3D fix)
  UVariable * varMode;
  /// index to replay flag
  //UVariable * varReplay;
  /// index to number of UTM updates received
  UVariable * varUpdCntUTM;
  /// index to number of UTM updates received
  UVariable * varUpdCntLL;
  /// device file name
  UVariable * varDevice;
  /// device file name
  UVariable * varOpen;
  UVariable * varIsOpen;
  /// device baudrate
  UVariable * varBaud;
  /// NMEA strings
  UVariable * varGPVTG;
  UVariable * varGPGLL;
  UVariable * varGPGSA;
  UVariable * varGPGSV;
  UVariable * varGPRMC;
  UVariable * varGPGGA;
  UVariable * varPTNL;

private:
  /// received Lat-Long position
  UGpsLatLong latLong;
  /// set when Lat-Long is updated
  bool latLongUpdated;
  /// Received UTM position
  UGpsUTM utm;
  /// set when utm is updated
  bool utmUpdated;
  /// Status of the GPS reception
  UGpsStatus status;
  /// set when status is updated
  bool statusUpdated;
  /** Last time data was received - including date
  Used when message holds time but no date, especially needed during replay. */
  UTime lastTime;
  /// Local computer time for last message - to be used in UTM pose history
  UTime msgTime;
  /// last message received
  char sentence[GPS_SENTENCE_MAX_LENGTH];
  /// debug log
  ULogFile log2;
};

#endif

