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
#ifndef UFUNC_SMRIF_H
#define UFUNC_SMRIF_H

#include <cstdlib>

#include <urob4/ufuncplugbase.h>

#include "uressmr.h"
#include "uressmrctl.h"

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
* A plugin that handles the connecton to SMRDEMO.
* The functionality allows to send commands to MRC - line by line,
  and a function to empty the "getevent" information maintaining variables
  for queued, started, finished and syntax error line status.
@author Christian Andersen
*/
class UFunctionSmrIf : public UFuncPlugBase
{ // NAMING convention recommend that the main plugin function class
  // starts with UFunction (as in UFunctionSmrIf) followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFunctionSmrIf();
  /**
  Destructor */
  virtual ~UFunctionSmrIf();
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
  pointer to smr interface resource */
  UResSmrIf * smrif;
  /**
  Is the resource created locally */
//  bool smrifLocal;
  /**
  pointer to smr - non real time - controller resource */
  UResSmrCtl * smrctl;
  /**
  Is the resource created locally */
//  bool smrctlLocal;

private:
  /**
  Function to get closest distance */
  bool handleSmr(UServerInMsg * msg);
  /**
  Handle push on connect handler */
  bool handleSmrPush(UServerInMsg * msg);
};


#endif

