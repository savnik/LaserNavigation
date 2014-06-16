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
#ifndef ULASERDEVICE_H
#define ULASERDEVICE_H

#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include <ugen4/ucommon.h>
#include <ugen4/utime.h>
#include <urob4/ucmdexe.h>
#include <ugen4/uposrot.h>
#include <urob4/ulogfile.h>

#include "ulaserdata.h"

/**
Max length of name returned from laser scanner */
#define MAX_NAME_LNG 1000
/**
length of serial device name - e.g. "/dev/tty4" */
#define MAX_DEVICE_NAME_LNG 300
#define MAX_SEND_MSG_LNG 300
/**
Root class with basical laser scanner services, start, stop, get data etc.

@author Christian Andersen
*/
class ULaserDevice: public UServerPush
{
public:
  /**
  Constructor */
  ULaserDevice();
  /**
  Destructor */
  virtual ~ULaserDevice();
  /**
  Is sick scanner running */
  inline bool isRunning()
    { return threadRunning and (isPortOpen() or modeSimulated); };
  /**
  Set device name */
  inline virtual void setDeviceName(const char * device)
    { strncpy(devName, device, MAX_DEVICE_NAME_LNG); } ;
  /**
  Get name of scanner */
  inline char * getName()
    { return name; } ;
  /**
  Get name of scanner */
  inline unsigned long getSerial()
  { return serial; } ;
  /**
  Is the port to the device open */
  virtual bool isPortOpen();
  /**
  Start sick scanner and grap the data */
  bool start();
  /**
  Stop sick scanner port.
  Stop also thread - if not 'justClosePort'. */
  void stop(bool justClosePort);
  /**
  Start the receive loop for continous messages */
  void threadRunLoop();
  /**
  Set simulated mode */
  inline void setSimulatedMode(bool value)
    { modeSimulated = value; };
  /**
  Change scanner resolution mode.
  (modeAngleScan and angleResolution)
  Returns true if new resolution is implemented.
  If not open, then values are just saved */
  virtual bool changeMode(int scanangle, double resolution);
  /**
  Get actual scan resolution in centi degrees */
  inline double getScanResolution()
    { return angleResolution; };
  /**
  Get actual scan angle in degrees */
  inline int getScanAngle()
    { return modeAngleScan; };
  /**
  Get statistics - good count */
  inline unsigned int getGood()
    { return statGoodCnt; };
  /**
  Get statistics - bad count */
  inline unsigned int getBad()
    { return statBadCnt; };
  /**
  Get statistics - number of good messages per second */
  inline double getMsgRate()
    { return statMsgRate; };
  /**
  Get serial device name */
  inline char * getDeviceName()
    { return devName;} ;
  /**
  Get number of measurements in scan-mode */
  int getMaxMeasurements()
    { return roundi(double(modeAngleScan) / angleResolution) + 1; };
  /**
  Get angle from measurement number - result is in degrees */
  double getScanAngle(int measurement)
    { return angleResolution * double(measurement) -
             double(modeAngleScan)/2.0; };
  /**
  Print status */
  void print(char * preString);
  /**
  Print device status to a buffer string */
  virtual const char * print(const char * preString, char * buff, int buffCnt);
  /**
  Get newest available data
  to this destination structure.
  Returns true if data is available. */
  virtual bool getNewestData(ULaserData * dest, 
                      unsigned long lastSerial,
                            int fake);
  /**
  Send data to device
  Data must be a zero-terminated string. */
  void send(char * msg);
  /**
  Is last string send */
  inline bool isSend()
  { return not sendNewData; };
  /**
  Get name of device */
  virtual const char * getNameFromDevice();
  /**
  Set verbose flag (for debugging) */
  inline void setVerbose(bool value)
  { verbose = value; };
  /**
  Is logfile open */
  virtual inline bool isLogFileOpen()
  { return datalog.isOpen(); };
  /**
  Is verbose flag on */
  virtual inline bool isVerbose()
  { return verbose; };
  /**
   * Is laserscanner a replay device scanner */
  virtual inline bool isReplayDevice()
  { return false; };
  /**
  Got new measurement data from device.
  if 'gotData' == NULL, then
  data is available by a getNewestData(ULaserData*) call.
  Data should not be requested (by call) unless there is
  users waiting for the data.
  NB! this call is called by the read thread, and not
  the server message handler thread. */
  void gotNewScan(ULaserData * gotData);
  /**
  Called by cmdExe to get push object
  followed by a call to 'gotNewData(object)'.
  Should be overwritten by push object holder. */
  virtual void callGotNewDataWithObject();
  /**
  Open a logfile in the data directory adding this
  device number at the end.
  If the log is open, then it is closed before (re) opening. */
  bool logFileOpen();
  /**
  * Get name of logfile (open or not) - made form devicenumber as 'laser_N'.
  * Returns 'buffer'*/
  char * getLogFileName(char * buffer, int bufferCnt);
  /**
  Close logfile for this device (if open). */
  void logFileClose();
  /**
  Set log interval */
  inline void setLogInterval(int value)
  { datalogInterval = value; };
  /**
  Get log interval */
  inline int getLogInterval()
  { return datalogInterval; };
  /**
  Set log used scans */
  inline void setLogUsedScans(bool value)
  { datalogUsedScans = value; };
  /**
  Get log used scans */
  inline bool getLogUsedScans()
  { return datalogUsedScans; };
  /**
  Set mirror flag. The mirror flag indicates that
  the laserscanner is mounted upside down, and thus
  the data is in reverse angle order.
  This will result in the start angle and resolution has
  negated sign. In normal mode first range is to the right, i.e. a negative angle. The resolution (angle step size) is then positive.
  With the mirror flag set, the start angle will be positive (first range to the left) and the resolution is negative. */
  inline void setMirror(bool value)
  { mirrorData = value; };
  /**
  Get mirror value. If false the first range is to the right. If
  mirror is true then first value is assumed to be to the left. */
  bool getMirror()
  { return mirrorData; };
  /**
  Get the device position on robot. The position (in 3D) is part of the
  device pose that also includes the rotation around the three axex (see gerDeviceRot().
  The position is in meter x is forward, y is left and z is up. */
  UPosition getDevicePos();
  /**
  * Get the device rotation relative to robot. The roattion (in 3D) is part of the
    device pose that also includes the position (see gerDeviceRos().
  * The rotation is in radians where Omega is rotation around the
    x-axis (Roll) positive is roll to the right.
  * Phi is rotation around the Y-axis (tilt), positive is tilt down.
  * Kappa is rotation around the Z-axis (turn) positive is left.
  * The specified rotation order is like for a crane, forst rotation, then tilt and
    finally roll of the in this way turned and tilted axis.*/
  URotation getDeviceRot();
  /**
  * Get the full device pose including 3D position (x,y,z) (see getDevicePos()) and rotation
    (Omega, Phi, Kappa) (see getDeviceRot)).
  * Returns a pointer to the structure, and can thus be used to modify the value too. */
  UPosRot getDevicePose();
  /**
  * Set the full device pose including 3D position (x,y,z) (see getDevicePos()) and rotation
    (Omega, Phi, Kappa) (see getDeviceRot)).
  * Returns a pointer to the structure, and can thus be used to modify the value too. */
  void setDevicePose(UPosRot * newPose);
  /**
  Get device number for device */
  inline int getDeviceNum()
  { return deviceNum; };
  /**
  Set device number for device */
  inline void setDeviceNum(int value)
  { deviceNum = value; };
  /**
  Log this scan */
  void logThisScan(ULaserData * scan);
  /**
   * set server core pointer - just a debug feature - I think) */
  virtual void setCore(UCmdExe * pCore)
  { };
  /**
  Create entry infor the globas variable database for this device */
  virtual void createBaseVars();
  /**
  Set pointer to var-pool structure for variables for this device. */
  void setVarStructure(UVarPool * varStruct)
  {
    vars = varStruct;
  };
  
protected:
  /**
  Open serial post.
  Return true if open(ed) */
  virtual bool openPort();
  /**
  Receive data from device -- called from
  device loop, should return as fast as possible
  after dooing the job, i.e. no blocking read.
  Should add number of good and bad blocks of data
  to statBadCnt and statGootcnt.
  Returns true if data received. */
  virtual bool receiveData();
  /**
  Close serial port */
  virtual void closePort();
  /**
  Send data to device. Sends 'lng' bytes from 'msg'.
  Data must be a zero-terminated string. */
  virtual bool sendToDevice(const char * msg, int lng);
  /**
    Get fake data to this destination and advance fake position
    if scan number is used before, otherwise maintain position
      \param dest is where to load the scan.
      \param lastSerial is last used serial number - set next number in scan
      \param int fake is fake number - 0 is live
      \param double fakeDt is update time for fake position - default is 0.2 sec */
  void getFakeScan(ULaserData * dest,
                   unsigned long lastSerial,
                   int fake, double fakeDt = 0.2);
  /**
  Update global variables with new scan data
  \param scantime scantime of the new scan (for framerate calculation)
  */
  void updateScanData(UTime scanTime);
  /**
  Get default delat estimate for the device type */
  inline virtual double getDefaultDelay()
  { return 0.0; };

protected:
  /**
  Serial device, where laser scanner is attached */
  char devName[MAX_DEVICE_NAME_LNG];
  /**
  Name returned from serial device */
  char name[MAX_NAME_LNG];
  /**
  Print more (debug) messages */
  bool verbose;
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
  Data logfile for received data - debug */
  ULogFile datalog;
  /**
  interval beteen systematic loggings.
  * 0 is no (systematic) logging.
  * 1 is log every scan.
  * N log every N'th scan.*/
  int datalogInterval;
  /**
  Log the used scans - that is scans requested by 'getNewestData' */
  bool datalogUsedScans;
  /**
  Logging sequence number */
  int datalogSeq;
  /**
  The scanning angle of the SICK laser scanner,
  it can be either 100 or 180 or 240 (URG only) deg.
  Unit is degrees. */
  int modeAngleScan;
  /**
  Angle resolution, this can be 1.0 0.5 or 0.25 degrees.
  NB! natural resolution is 1.0 deg, the others take
  2 or 4 resolutions to generate.
  Unit is 1 degree.
  For the URG scanner the resolution is fixed to 360/1024 = 0.35156 deg. */
  double angleResolution;
  /**
  Use simulated data and not real data from Scanner */
  bool modeSimulated;
  /**
  Read statistics total good messages */
  unsigned int statGoodCnt;
  /**
  Read statistics total bad messages */
  unsigned int statBadCnt;
  /**
  Good messages per second. */
  double statMsgRate;
  /**
  Buffer for message to send */
  char sendStr[MAX_SEND_MSG_LNG];
  /**
  Send string length, may include a zero. */
  int sendStrCnt;
  /**
  Flag for new data to send */
  bool sendNewData;
  /**
  Loop count for receive thread */
  int loopCnt;
  /**
  Serial number */
  unsigned long serial;
  /**
  Mirror data - laser is mounted upside down
  reverse the angle reading */
  bool mirrorData;
  /**
  Fake scan pose */
  //UPoseTVQ fakePose;
  /**
  Fake scan pose state */
  //int fakeState;
  /**
  Max valid range for this laserscanner */
  double maxValidRange;
  /**
  Device pose on robot (3D).
  This pose is mostly informative as the data provided in a
  scanget is in sensor coordinates onle. The only exception
  is the mirror flag, that influences the read-out angle, when
  data is extracted from the ULaserData structure.
  The device pose is pimarily made available for
  the following coordinate transformation.
  Default value is zero - i.e. at robot origin forward looking */
  // UPosRot devicePose; - moved to global var
  /**
  Device number in current laser pool */
  int deviceNum;
  /**
  Scan to be used by a pending push command */
  ULaserData * pushData;
  /**
  Pointer to local variable structure */
  UVarPool * vars;
  struct
  { // status variables for this device
    UVariable * name;
    UVariable * versionInfo;
    UVariable * type;
    UVariable * serial;
    UVariable * scanwidth;
    UVariable * scanres;
    UVariable * maxRange;
    UVariable * pose;
    UVariable * isOpen;
    UVariable * framerate;
    UVariable * scanDelay;
  } var;
  /**
  Time of last scan */
  UTime lastScanTime;
};

#endif
