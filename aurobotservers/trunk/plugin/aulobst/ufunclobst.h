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
#ifndef UFUNC_LOBST_H
#define UFUNC_LOBST_H

#include <cstdlib>

#include <urob4/ufuncplugbase.h>
#include <ulms4/ulaserpool.h>

#include "ureslobst.h"

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
 * This module implements a simple file finterface intended to transfer mission files from a GUI interface to the robot. The file (or a selected file) can then be loaded into the mission manager.
@author Christian Andersen
*/
class UFuncLobst : public UFuncPlugBase
{ // NAMING convention recommend that the main plugin function class
  // starts with UFunction (as in UFunctionLine) followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFuncLobst()
  { // command list and version text
    setCommand("lobst", "laserObstacleDetect", "laser ransac obstacle detect (by jca/enis " __DATE__ " " __TIME__ ")");
    lobst = NULL;
  }
  /**
  Destructor */
  virtual ~UFuncLobst();
  /**
   * Called by server after this module is integrated into the server core structure,
   * i.e. all core services are in place, but no commands are serviced for this module yet.
   * Create any resources that this modules needs and add these to the resource pool
   * to be integrated in the global variable pool and method sharing.
   * Remember to add to resource pool by a call to addResource(UResBase * newResource, this).
   * This module must also ensure that any resources creted are deleted (in the destructor of this class).
   * Resource shut-down code should be handled in the resource destructor. */
  virtual void createResources();
  /**
   * Handle incomming commands flagged for this module
   * \param msg pointer to the message and the client issuing the command
   * \return true if the function is handled - otherwise the client will get a 'failed' reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);

protected:
  /**
  * \brief get a laserscan based on these parameters
  * \param device use this device or default if device is "-1", may else be "v360" or "urg" or "0"
  * \param las [out] returns a pointer to the used laser scanner
  * \param pushData points to push data if such is available, else NULL
  * \param getOnly if true, then the laser scanner device is returned only, and the returned scan is a pointer to a scan buffer (with old information from last time)
  * \param fake if fake > 0 then the data produced is fake, generated from static data. Very usefull when testing without any real scanner (or simulator environment)
  * \returns a pointer to a laserscan (never NULL) */
  ULaserData * getScan(const char * device,
                       ULaserDevice ** las,
                       ULaserData * pushData,
                       bool getOnly,
                       int fake);
private:
  /// to test method call
  void testMethodCall();


protected:
  /**
  line pointer */
  UResLobst * lobst;
  /**
  * Data buffer for a laserscan, and information
  * about the last used scan, i.e. serial number and device */
  ULaserData dataBuff;
};


#endif

