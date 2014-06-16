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
#ifndef UCAMPOOL_H
#define UCAMPOOL_H

#include <urob4/uvariable.h>
//#include <urob4/ureplay.h>

#include "ucampush.h"
#include "uimagelog.h"

/**
Maximum number of cameras on robot. */
#define MAX_MOUNTED_CAMERAS 30

class UImagePool;

/**
Pool of cameras mounted on a robot.

@author Christian Andersen
*/
class UCamPool : public UResVarPool
{
public:
  /**
  Constructor */
  UCamPool()
  { // set name and version number
    setResID(getResClassID(), 200);
    UCamPoolInit();
  };
  /** destructor */
  virtual ~UCamPool();
  /**
   * Camera pool initialization */
  void UCamPoolInit();
  /**
  Resource ID for this class */
  static const char * getResClassID()
  { return "camPool"; };
  /**
  Resource version of this class */
/*  static int getResVersion()
  { return 161; };*/
  /**
  Set verbose messages - mostly for debug purpose */
  inline void setVerbose(bool value)
    { verboseMessages = value;};
  /**
  Create camera structures from found devices */
  bool findDevices();
  /**
  Make a device, that needs no physical camera - or a camera with a separate interface, like the kinect
  \param devNum is the device number to create.
  \param posName is an optional posName (must be unique), set to NULL if to be set later 
  \returns a pointer to the new camera structure, or to the existing camera with this device number */
  UCamPush * makeDevice(int devNum, const char * posName);
  /**
  Get a camera device from the device number */
  UCamPush * getCam(int device);
  /**
  Get first camera device number.
  \returns -1 if no camera is available, else device number of first camera. */
  int getFirstCamDevice();
  /**
  Get a camera device from the camera position name, i.e. 'cameraLeft' */
  UCamPush * getCam(const char * posName);
  /**
  Set command executor */
  inline void setCmdExe(UCmdExe * executor)
    { cmdExe = executor; };
  /**
  Get camera device count */
  inline int getDeviceCount()
    { return camCnt; };
  /**
  Get camera by index number in cam-pool. this
  may not be in device order */
  inline UCamPush * getCamByPoolIndex(int index)
    { return cam[index]; };
  /**
  Print the state of cameras */
  virtual void print(const char * preString);
  /**
  Print the state of cameras to a string buffer */
  virtual const char * print(const char * preString, char * buff, int buffCnt);
  /**
  Flush all active commands from (a dead) client */
  void flushClientCmds(int clientIdx);
  /**
  Save camera settings to configuration file.
  This saves the settings if the position name is different from
  the default 'posUnknown'. If the position is 'posUnknown', then
  the position name is set to 'posD' (D beeing the device number)
  and the parametes are set under this name.
  If two cameras has the same position name, then the last
  are assigned a new name.
  If 'ini' is NULL, then the values are saved in the default config-file. */
  bool saveSettings(Uconfig * ini);
  /**
  Save settings to default configuration file - the file mentioned in
  the global string configFileCal (defined in ugen4/ucommon.h. */
  virtual void saveSettings()
  { saveSettings(NULL); };
  /**
  Open logfile - same as request logging of images */
  bool openImageLogging(const char * name);
  /**
  Open image logging in default logfile */
  bool inline openImageLogging()
  { return openImageLogging("image");  };
  /**
  Is image log open */
  inline bool isImagelogOpen()
  { return imageLog->isOpen();};
  /**
  set image log format */
  inline void setLogPng(bool value)
  { imageLog->setPng(value);};
  /**
  is image log format PNG */
  inline bool isLogPng()
  { return imageLog->isPng();};
  /**
  Is image logging set to save image itself too? */
  inline bool isLogSaveImg()
  { return imageLog->isLogSaveImg(); };
  /**
  Set image logging set to save image itself or not. */
  inline void setLogSaveImg(bool value)
  { imageLog->setSaveImg(value); };
  /**
  Get image log filename */
  const char * getImagelogFilename()
  { return imageLog->getLogName();};
  /**
  Close logfile - and stop logging */
  void closeImageLogging();
  /**
  Set (or remove) ressource (core pointer needed by event handling) */
  bool setResource(UResBase * resource, bool remove);
  /**
  Test if pool has all needed resources. */
  bool gotAllResources(char * missingThese, int missingTheseCnt);

protected:
  /**
  Create initial var-pool variables */
  void createBaseVar();
    /**
  Decode this replay line, that is assumed to be valid.
  \param line is a pointer to the line.
  \returns true if the line is valid and used (step is successfull).
  \returns false if line is not a valid step. */
  virtual bool decodeReplayLine(char * line);


private:
  /**
  Array of camera structures. */
  UCamPush * cam[MAX_MOUNTED_CAMERAS];
  /**
  Number of created camera structures. */
  int camCnt;
  /**
  Command executor for push commands. */
  UCmdExe * cmdExe;
  /**
  Image pool pointer for push buffer images */
  UImagePool * imgPool;
  /**
  Print more when relevant */
  bool verboseMessages;
  protected:
  /**
  Logfile for continued logging of images */
  UImageLog * imageLog;
  /// guppy support
  UVariable * varUseGuppy;
  /// guppy support
  UVariable * varUseIeeeOld;
  /// guppy support
  UVariable * varUseGigE;
  /// camera devices
  UVariable * varCams;
  /// number of camera devices
  UVariable * varCamsCnt;
};

#endif
