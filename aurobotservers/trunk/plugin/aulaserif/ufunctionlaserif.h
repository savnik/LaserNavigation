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
#ifndef UFUNC_LASERIF_H
#define UFUNC_LASERIF_H

#include <cstdlib>

#include <urob4/ufuncplugbase.h>
#include <urob4/usmltag.h>
//#include <urob4/uresclientifvar.h>

//#include "ureslaserif.h"
#include "ureslaserifroad.h"
#include "ureslaserifobst.h"
#include "ureslaserifsf.h"

class UResLaserIfScan;
class UResNavIfMan;

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
class UFunctionLaserIfData : public UFuncPlugBase
{ // NAMING convention recommend that the main plugin function class
  // starts with UFunction (as in UFunctionLaserIfData) followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFunctionLaserIfData();
  /**
  Destructor */
  virtual ~UFunctionLaserIfData();
  /**
  List (space separated) of shared resources
  provided by this function.
  Must be an empty string if no resources are to be shared.
  Each resource ID must be no longer than 20 characters long. */
  //virtual const char * resourceList();
  /**
  Create any resources that this modules needs and add these to the resource pool
  to be integrated in the global variable pool and method sharing. */
  //virtual void createResources();
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
  //UResBase * getResource(const char * resID);
  /**
  return true if all ressources is available */
  //virtual bool gotAllResources(char * missingThese, int missingTheseCnt);
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
  Add a data handler resource to the interface client */
  //bool addResource(const char * resName);
  /**
  handle commands to laserdata */
  bool handleLaserIf(UServerInMsg * msg);
  /**
  handle commands to laserdata obstacles */
  bool handleLaserObst(UServerInMsg * msg);
  
protected:
  /**
  Reference to interface connection for road line data */
  UResLaserIfRoad * ifRoad;
  /**
  Is interface ressource created locally by this function */
  bool ifRoadLocal;
  /**
  Reference to interface connection for obstacle data */
  UResLaserIfObst * ifObst;
  /**
  Is interface ressource created locally by this function */
  bool ifObstLocal;
  /**
  Reference to interface connection for obstacle data */
  UResLaserIfSf * ifSf;
  /**
  Is interface ressource created locally by this function */
  bool ifSfLocal;
  /**
  Navigation path resource from navigation server */
  UResNavIfMan * ifMan;
  /**
   * Is the nav resource created locally */
  bool ifManLocal;

private:
  static const int MAX_RESOURCE_LIST_SIZE = 100;
  /**
  Data trap serial number (0 is no data) */
  unsigned int dataTrapSerial;
  /**
  Data trap serial number (0 is no data) for trap of help messages */
  unsigned int dataTrapSerialHelp;
  /**
  Client number for trapped data */
  int dataTrapClient;
  /**
  Ressource list provided */
  char resList[MAX_RESOURCE_LIST_SIZE];
};


#endif

