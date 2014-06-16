/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 * ----------------------------------------------------------------------- *
 *   Plugin for aurobotservers version 2.206.                              *
 *   Does laserbased localisation for a robot running in an orchard        *
 *   environment.                                                          *
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
 
#ifndef UFUNC_LOCATER_H
#define UFUNC_LOCATER_H

#include <cstdlib>
#include <urob4/ufunctionbase.h>
#include "ureslocater.h"

class UFunctionLocater : public UFunctionBase
{ // NAMING convention recommend that the main plugin function class
  // starts with UFunction (as in UFunctionNearGet) followed by
  // a descriptive extension for this specific plugin
public: 
  /**
  Constructor */
  UFunctionLocater()
  {
    setCommand("locater", "treeRowLocate", "interface to locater based on tree detection by laser scanner");
    locater = NULL;  // initially the resource is not created
  }
  /**
  Destructor */
  virtual ~UFunctionLocater();
  
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
  UResLocater *locater;

//  int landmarkno;
  //prepep
  
//  void handledrive(UServerInMsg *msg);
  
private:
  /**
  Function to get closest distance */
  bool handleLocater(UServerInMsg * msg, ULaserData * pushData);
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

#endif   // UFUNC_LOCATER_H

