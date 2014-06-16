/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 * ----------------------------------------------------------------------- *
 *   Plugin for aurobotservers version 2.206.                              *
 *   Contains map database						   *
 *   Edited by Peter Tjell (s032041) & Søren Hansen (s021751)              *
 * ----------------------------------------------------------------------- *
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
/**
@author Christian Andersen
@editor Peter Tjell / Søren Hansen
*/
 
#ifndef UFUNC_MAPBASE_H
#define UFUNC_MAPBASE_H

#include <cstdlib>
#include <urob4/ufunctionbase.h>
//#include <urob4/uresvarpool.h>

# include "uresmapbase.h"

class UFunctionMapbase : public UFunctionBase
{ // NAMING convention recommend that the main plugin function class
  // starts with UFunction (as in UFunctionNearGet) followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFunctionMapbase()
  {
    setCommand("mapbase", "mapbaseIf", "interface to map (by Anders, Søren, jca " __DATE__ " " __TIME__ ")");
    mapbase = NULL;  // initially the resource is not created
  }
  /**
  Destructor */
  virtual ~UFunctionMapbase();

    /**
  Called by the server core. Should return the
  name of function. There should be a first short part separated
  by a space to some additional info (e.g. version and author).
  The returned name is intended as informative to clients
  and should include a version number */
//  virtual const char * name();
  /**
  Called by the server core when loaded, to get a list of
  keywords (commands) handled by this plugin.
  Return a list of handled functions in
  one string separated by a space.
  e.g. return "COG".
  The functions should be unique on the server. */
//  virtual const char * commandList();
   /**
  Create any resources that this modules needs and add these to the resource pool
  to be integrated in the global variable pool and method sharing.
  When this is called the plugin is integrated into the server, and everything is in place
  for resource creation, but no messages are handled yet.*/
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

protected:
	/**
	 * Mapbase resource
	 */
  UResMapbase *mapbase;
	/**
	 * Is mapbase local
	 */
	bool mapbaseLocal;
  
private:
  /**
  Function to get closest distance */
  bool handleMapbase(UServerInMsg * msg);
  /**
   * Send all maplines as line segments */
  bool sendMapLines();
};

/** called when server makes a dlopen() (loads plugin into memory) */
void libraryOpen();

/** called when server makes a dlclose() (unloads plugin from memory) */
void libraryClose();

/**
Needed for correct loading and linking of library */
void __attribute__ ((constructor)) libraryOpen(void);
void __attribute__ ((destructor)) libraryClose(void);
/**
Allows server to create object(s) of this class */
extern "C" UFunctionBase * createFunc();
/**
... and to destroy the created object(s) */
extern "C" void deleteFunc(UFunctionBase * p);

#endif   // UFUNC_MAPBASE_H

