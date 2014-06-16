/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
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

#ifndef URESLASERIF_H
#define URESLASERIF_H

#include <cstdlib>

#include <urob4/uresbase.h>
#include <urob4/uresvarpool.h>
#include <urob4/uclienthandler.h>
#include <urob4/ucmdexe.h>

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
This is the shared resource class.
It must enherit from the resource base class (or one of its decendent) as shown.

@author Christian Andersen
*/

class UResLaserIf : public UClientHandler, public UResVarPool, public UServerPush
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResLaserIf) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResLaserIf()
  { // set name and version
    setResID(getResClassID(), 200);
    // create global var space
    createVarSpace(20, 0, 2, "Laser scanner server interface status", false);
    createBaseVar();
    // default port
    port = 24919;
    // default host
    strncpy(host, "localhost", MAX_HOST_LENGTH);
    // try to connect
    tryHoldConnection = true;
  };
  /**
  Destructor */
  virtual ~UResLaserIf();
  /**
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "laserif"; };
  /**
  Fixed varsion number for this resource type.
  Should follow release version, i.e. version 1.28 gives number 128.
  Should be incremented only when there is change to this class
  definition, i.e new or changed functions or variables. */
/*  static int getResVersion()
  { return 169; };*/
  /**
  print status to a string buffer */
  virtual const char * print(const char * preString, char * buff, int buffCnt);
  /**
  Print status for this resource */
  inline virtual void snprint(const char * preString, char * buff, int buffCnt)
  { print(preString, buff, buffCnt); };
  /**
  * Called by server core when new resources are available.
  * return true is resouurce is used
  * Save a pointer to the resource as needed. */
  bool setResource(UResBase * resource, bool remove);

// the above methods are udes be the server core and should be present
// the print function is not strictly needed, but is nice.
public:
  /**
  The varPool has methods, and a call to one of these is needed.
  Do what is needed and return (a double sized) result in 'value' and
  return true if the method call is allowed.
  The 'paramOrder' indicates the valid parameters d, s or c for double, string or class that is available as input values for the call.
 * If the returnStruct and returnStructCnt is not NULL, then
  a number (no more than initial value of returnStructCnt) of
  structures based on UDataBase may be returned into returnStruct array
  of pointers. The returnStructCnt should be modified to the actual
  number of used pointers (if needed). */
  virtual bool methodCall(const char * name, const char * paramOrder,
                  char ** strings, const double * pars,
                  double * value,
                  UDataBase ** returnStruct,
                  int * returnStructCnt);

protected:
  // Locally used methods
  /**
  Create the smrif related variables */
  void createBaseVar();
  /**
  Connection status is changed */
  void connectionChange(bool connected);

protected:
  /**
  local variables provided by this resource. */

};

#endif

