/** ************************************************************************
 *                                                                         *
 *   \file              scanfeatures.h                                     *
 *   \author            Christian Andersen                                 *
 *   \date              mar 2008                                           *
 *   \brief             stereo capture plugin                              *
 *                                                                         *
 *   plugin for (camera) server to get disparity image                     *
 *                                                                         *
 *                      Copyright (C) 2008 by DTU                     *
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


#ifndef UFUNC_SVS_H
#define UFUNC_SVS_H

#include <cstdlib>

#include <urob4/ufuncplugbase.h>

//#ifdef USE_SVS
#include "uressvs.h"

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
 * \brief stereo capture interface
 *
 * Plugin for (cameraserver) with required functions.
 *
 * @author Christian Andersen (mar 2008)
 */
class UFuncSVS : public UFuncPlugBase
{ // NAMING convention recommend that the main plugin function class
  // starts with UFunction (as in UFunctionLine) followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFuncSVS();
  /**
  Destructor */
  virtual ~UFuncSVS();
  /**
  Called by the server core. Should return the
  name of function. There should be a first short part separated
  by a space to some additional info (e.g. version and author).
  The returned name is intended as informative to clients
  and should include a version number */
  virtual const char * name();
  /**
  Called by the server core when loaded, to get a list of
  keywords (commands) handled by this plugin.
  Return a list of handled functions in
  one string separated by a space.
  e.g. return "COG".
  The functions should be unique on the server. */
//  virtual const char * commandList();
  /**
   * Called by server after this module is integrated into the server core structure,
   * i.e. all core services are in place, but no commands are serviced for this module yet.
   * Create any resources that this modules needs and add these to the resource pool
   * to be integrated in the global variable pool and method sharing.
   * Remember to add to resource pool by a call to addResource(UResBase * newResource, this).
   * This module must also ensure that any resources creted are deleted (in the destructor of this class)
   * Resource shut-down code should be handled in the resource destructor.
   * \return true if any resources are created. */
  virtual void createResources();
  /**
  Handle incomming command
  (intended for command separation)
  Must return true if the function is handled -
  otherwise the client will get a failed - reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);
  /**
  print status to a string buffer */
  virtual const char * print(const char * preString, char * buff, int buffCnt);
  /**
   * Set time of module load, this is probably the closest we get
   * to the real reference time for the camera images */
  virtual void setLoadTime(UTime loadTime)
  {
    printf("Set Load time call - is %.1f ms ago\n", svsTimeRef - loadTime);
    svsTimeRef = loadTime;
    if (ressvs != NULL)
      ressvs->setImageTimeRef(svsTimeRef);
  };
  /**
   * Do som hard-coded calibration code */
  bool doFranCode(UServerInMsg * msg, int franVal);

protected:
  /**
  Last used scan number */
  unsigned long lastSerial;
  /**
  Pointer to a ExtractFeatures resource and resource handler flag */
  UResSVS * ressvs;
  //bool ressvsLocal;

private:
  /**
  Function to handle scanfeatures command */
  bool handleSVS(UServerInMsg * msg, USvsImageSet * pushUsi);
  /**
   * Handle the svsPush command */
  bool handlePushCommand(UServerInMsg * msg);
  /**
   * Reference time for image timestamp */
  UTime svsTimeRef;
};


#endif
