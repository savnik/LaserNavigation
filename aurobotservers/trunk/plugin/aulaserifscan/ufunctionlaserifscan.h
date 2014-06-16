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
#ifndef UFUNC_LASERIFSCAN_H
#define UFUNC_LASERIFSCAN_H

#include <cstdlib>

#include <urob4/ufuncplugbase.h>
#include <urob4/usmltag.h>
#include <urob4/uresclientifvar.h>

//#include "ureslaserif.h"
//#include "ureslaserifroad.h"
//#include "ureslaserifobst.h"
//#include "ureslaserifsf.h"
#include "ureslaserifscan.h"

//class UResLaserIfScan;

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
Example plugin that demonstrates a plogin that provides a resource.
A similar example plugin is available that uses the shared resource.

The shared resource provides the simple functionality in the form of a line.
The resource provides functions to calculate the length of the line.

@author Christian Andersen
*/
class UFunctionLaserIfScan : public UFuncPlugBase
{ // NAMING convention recommend that the main plugin function class
  // starts with UFunction (as in UFunctionLaserIfScan) followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFunctionLaserIfScan();
  /**
  Destructor */
  virtual ~UFunctionLaserIfScan();
  /**
  List (space separated) of shared resources
  provided by this function.
  Must be an empty string if no resources are to be shared.
  Each resource ID must be no longer than 20 characters long. */
//  virtual const char * resourceList();
  /**
  Create any resources that this modules needs and add these to the resource pool
  to be integrated in the global variable pool and method sharing. */
  virtual void createResources();
  /**
  Called by the server core, when a new resource is
  available (or is removed), local pointers to the resource should be
  updated as appropriate. */
  virtual bool setResource(UResBase * resource, bool remove);
  /**
  This function is called by the server core, when it needs a
  resource provided by this plugin.
  Return a pointer to a resource with an ID taht matches this 'resID' ID string.
  The string match should be case sensitive.
  Returns false if the resource faled to be created (e.g. no memory space). */
//  UResBase * getResource(const char * resID);
  /**
  return true if all ressources is available */
//  virtual bool gotAllResources(char * missingThese, int missingTheseCnt);
  /**
  Handle incomming command
  (intended for command separation)
  Must return true if the function is handled -
  otherwise the client will get a failed - reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);
  /**
  print status to a string buffer */
  virtual const char *  print(const char * preString, char * buff, int buffCnt);
  /**
  Data that should be transferred to a client directly is received */
  bool dataTrap(USmlTag * tag);


protected:
  /**
  Reference to interface connection for obstacle data */
  UResLaserIfScan * ifScan;
  /**
  Is interface ressource created locally by this function */
  //bool ifScanLocal;

private:
  static const int MAX_RESOURCE_LIST_SIZE = 100;
  /**
  Function to hadle laserscan commands */
  bool handleLaserIf(UServerInMsg * msg);
  /**
  Ressource list provided */
  //char resList[MAX_RESOURCE_LIST_SIZE];
};


#endif

