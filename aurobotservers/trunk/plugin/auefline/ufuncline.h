/** ************************************************************************
 *                                                                         *
 *   \file              scanfeatures.h                                     *
 *   \author            Lars Pontoppidan Larsen, Aske Olsson               *
 *   \date              Dec 2006                                           *
 *   \brief             ScanFeatures plugin for ulmsserver v. > 1.5        *
 *                                                                         *
 *   Implementation of the ScanFeatures main class and communication with  *
 *   the laser server.                                                     *
 *                                                                         *
 *                      Copyright (C) 2006-2008 by DTU                     *
 *                      rse@oersted.dtu.dk                                 *
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


#ifndef UFUNC_EF_LINE_H
#define UFUNC_EF_LINE_H

#include <cstdlib>

#include <urob4/ufuncplugbase.h>
#include <ulms4/ulaserpool.h>

#include "uresauef.h"

class UResPoseHist;

/**
 * \brief ScanFeatures class converted to line extract
 *
 * Plugin for Ulmsserver with required functions.
 *
 * @author Aske Olsson
 * @author Lars Pontoppidan
 * @author Lars Mogensen
 * @author Christian Andersen (feb 2008)
 */
class UFuncEfLine : public UFuncPlugBase
{ // NAMING convention recommend that the main plugin class
  // starts with UFunc (as in UFuncEfLine) followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFuncEfLine();
  /**
  Destructor */
  virtual ~UFuncEfLine();
  /**
   * Called by server after this module is integrated into the server core structure,
   * i.e. all core services are in place, but no commands are serviced for this module yet.
   * Create any resources that this modules needs and add these to the resource pool
   * to be integrated in the global variable pool and method sharing.
   * Remember to add to resource pool by a call to addResource(UResBase * newResource, this).
   * This module must also ensure that any resources creted are deleted (in the destructor of this class).
   * Resource shut-down code should be handled in the resource destructor. */
  void createResources();
  /**
  Handle incomming command
  (intended for command separation)
  Must return true if the function is handled -
  otherwise the client will get a failed - reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);

protected:
  /**
   * \brief get a laserscan based on these parameters
   * \param device use this device of default if device is -1
   * \param las [out] returns a pointer to the used laser scanner
   * \param pushData points to push data if such is available, else NULL
   * \param getOnly if true, then the laser scanner device is returned only, and the returned scan is a pointer to a scan buffer (with old information from last time)
   * \param fake if fake > 0 then the data produced is fake, generated from static data. Very usefull when testing without any real scanner (or simulator environment)
   * \returns a pointer to a laserscan (never NULL) */
  ULaserData * getScan(int device,
                    ULaserDevice ** las,
                    ULaserData * pushData,
                    bool getOnly,
                    int fake);
  /**
   * \brief get the current pose of the robot at the time of the laser data
   * The pose may be fake, and the fake pose is in the laser data structure too
   * \param data holds current laserscan, detection time and possibly a fake pose if the data is fake.
   * \returns the current pose, or the zero pose if odometry resource is not available. */
  UPose getCurrentOdoPose(ULaserData * data);
  /**
   * \brief send robot pose and sensor position (orientation) to client
   * \param msg holds the client information
   * \param las is a pointer to the laser device
   * \param data is a pointer to the current laserscan
   * \returns true if the data could be send.  */
  bool sendRobotPoseAndSensorPosition(UServerInMsg * msg,
                                      ULaserDevice * las,
                                      ULaserData * data);
  /**
   * \brief Process a request for all detected (wall-like) lines in the scan
   * \param msg holds the client information
   * \param las is a pointer to the laser device
   * \param data is a pointer to the current laserscan
   * \param getonly is set to true, if no new processing is to take place, just get result of last processing.
   * \return true if full reply message is send. */
  bool sendAllLines(UServerInMsg * msg,
                    ULaserDevice * las,
                    ULaserData * data,
                    bool getOnly);
  /**
   * \brief Process a request for (4) wall lines in the scan
   * \param msg holds the client information
   * \param las is a pointer to the laser device
   * \param data is a pointer to the current laserscan
   * \param getonly is set to true, if no new processing is to take place, just get result of last processing.
   * \return true if full reply message is send. */
  bool sendBoxLines(UServerInMsg * msg,
                    ULaserDevice * las,
                    ULaserData * data,
                    bool getOnly);
  /**
   * \brief Process a request for the pose of the south-west corner of the box
   * The reply is send as a special \<laser l0=\"0.0\" l1=\"1.0\" .../\> type tag
   * \param msg holds the client information
   * \param las is a pointer to the laser device
   * \param data is a pointer to the current laserscan
   * \param getonly is set to true, if no new processing is to take place, just get result of last processing.
   * \return true if full reply message is send. */
  bool sendBoxPose(UServerInMsg * msg,
                   ULaserDevice * las,
                   ULaserData * data,
                   bool getOnly);

protected:
  /**
  Last used scan number */
  unsigned long lastSerial;
  /**
  Last device used */
  int lastDevice;
  /**
  Pointer to a ExtractFeatures resource and resource handler flag */
  UResAuEf * auef;
  //bool auefLocal;

private:
  /**
  Function to handle scanfeatures command */
  bool handleLine(UServerInMsg * msg, ULaserData * pushData);
  /**
  Opens log file, if not already open */
  void openLog(char *);
  /**
  Closes log file if open */
  void closeLog();
  /**
   * Data buffer for a laserscan, and information
   * about the last used scan, i.e. serial number and device */
  ULaserData dataBuff;
  /**
  Log file handle, null if no file open */
  FILE *logFile;

};

#endif
