/** \file ufuncplanner.h
 *  \ingroup Mission Planner
 *  \brief UFunction class for Robot Mission planner
 *
 *  Robot mission planner plugin for AU Robot Servers
 *
 *  \author Anders Billesø Beck
 *  $Rev: 966 $
 *  $Date: 2010-05-13 13:12:20 +0200 (Thu, 13 May 2010) $
 */
/***************************************************************************
 *                  Copyright 2010 Anders Billesø Beck, DTU                *
 *                       anbb@teknologisk.dk                               *
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

#ifndef UFUNCPLANNER_H
  #define UFUNCPLANNER_H

#include <cstdlib>

#include <urob4/ufuncplugbase.h>

#include "uresplanner.h"

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
 * The command interface of the robot mission manager plugin
@author Anders Billesø Beck
*/
class UFuncPlanner : public UFuncPlugBase
{ // NAMING convention recommend that the main plugin function class
  // starts with UFunction (as in UFunctionLine) followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFuncPlanner();
  /**
  Destructor */
  virtual ~UFuncPlanner();
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
  virtual const char * commandList();
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
  resource pointer */
  UResPlanner *planner;

  /** Name string of plugin */
  char nameStr[256];
  /** Revision string for plugin */
  char revStr[64];
};


#endif

