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
#ifndef UREPLAYDEVICE_H
#define UREPLAYDEVICE_H

#include "ulaserdata.h"
#include "ulaserdevice.h"

class UResPoseHist;

/**
Laser scanner device that gets the data from a logfile

	@author Christian Andersen <jca@oersted.dtu.dk>
*/
class UReplayDevice : public ULaserDevice
{
public:
  /**
  Constructor */
  UReplayDevice();
  /**
  destructor */
  ~UReplayDevice();
  /**
  Change scanner resolution mode.
  Returns true if new resolution is set */
  //virtual bool changeMode(int scanangle, double resolution);
  /**
  Is the port to the device open */
  virtual bool isPortOpen()
  { return true; };
  /**
  * Get the newest data unpacked to this structure.
  * Returns true if valid. */
  virtual bool getNewestData(ULaserData * dest,
                             unsigned long lastSerial,
                             int fake);
  /**
  * Is laserscanner a replay device scanner */
  virtual inline bool isReplayDevice()
  { return true; };
  /**
  * Print device status to a buffer string */
  //virtual const char * print(const char * preString, char * buff, int buffCnt);
  /**
  * Device name determines max range for device.
  * Decode the range from the name. */
  double maxRange();

public:

  /**
 * Set replay flag.
 * If replay flag is changed to true, the logfile (odo.log) is
  attempted opened.
 * If successfull, the current pose history is cleared and
  the first record read and implemented.
 * If not successfull, the replay flag is set to true,
  and the pose history is cleared only.
   * Returns true if replay file is opened */
  bool setReplay(bool value);
  /**
   * Set replay subdir. Sets the value to the subdir buffer,
  but nothing else is changed. */
//  void setReplaySubdir(const char * subdir);
  /**
  Get pointer to replay subdir */
/*  inline const char * getReplaySubdir()
  { return replaySubPath; };*/
  /**
  Get replat time now */
  inline UTime getReplayTimeNow()
  { return replayTimeNow; };
  /**
  Get replat time next */
  inline UTime getReplayTimeNext()
  { return replayTimeNext; };
  /**
  Get replay flag */
  virtual inline bool isReplay()
  { return replay; };
  /**
  is replay file open */
  inline bool isReplayFileOpen()
  { return (replayFile != NULL); };
  /**
  Get current line number in replay logfile */
  inline int getReplayLogLine()
  { return replayLogLine; };
  /**
   * Get the full replay logfile name.
   * Returns pointer fn[] string. */
  char * getReplayFileName(char * fn, const int fnCnt);
  /**
   * Replay N singles from the logfile.
   * \param steps number of steps to advance
   * \param lasPool is pointer to laser pool - to notify other replay partners.
   * \Returns true if N steps were available in replay file. */
  bool replayStep(int steps, UResBase * lasPool);
  /**
   * Advance replay to this scannumber
   * \param toSerial is the scan serial to advance to
   * \param lasPool is pointer to laser pool - to notify other replay partners.
   * \Returns true if scan is found. */
  bool replayStepToScan(unsigned int toSerial, UResBase * lasPool);
  /**
  Replay until just before this time */
  bool replayToTime(UTime untilTime);
  /**
  Test if replay file with current specifications exist (and read is allowed) */
  bool getReplayFileExist();

private:
  /**
  * Replay a single step from the logfile.
  * \Returns true if step is performed (not EOF). */
  bool replayStep();


protected:
  /**
  Replay flag - source of info for pose history
  a change in this flag clears the current pose history.*/
  bool replay;
  /**
  Replay file handle */
  FILE * replayFile;
  /**
  Maximum length of a line in the logfile */
  static const int MAX_LOG_LINE_LENGTH = 10000;
  /**
  Buffer for latest line (but not used) line from logfile */
  char replayLine[MAX_LOG_LINE_LENGTH];
  /**
  Replay time of last used data from logfile */
  UTime replayTimeNow;
  /**
  Replay time of next data from in logfile (line buffer) */
  UTime replayTimeNext;
  /**
  Current line number in logfile */
  int replayLogLine;
  /**
  Buffer for laser data */
  ULaserData scan;
};

#endif
