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
#ifndef UFUNCCRON_H
#define UFUNCCRON_H

#include <cstdlib>

#include <urob4/ufuncplugbase.h>

#include "urescron.h"

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
 * The command interface of a job scheduler that is asunchronous to the the main command queue, and thus long timeconsuming jobs are holding the main command thread.
 * It adds though one additional thread only, and commands are run as pipes.
@author Christian Andersen
*/
class UFuncCron : public UFuncPlugBase
{ // NAMING convention recommend that the main plugin function class
  // starts with UFunction (as in UFunctionLine) followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFuncCron()
  { // command list and version text
    setCommand("cron", "bashCommands", "handles time initiated bash scripts and commands");
  }
  /**
  Destructor */
  virtual ~UFuncCron();
  /**
   * Called by server after this module is integrated into the server core structure,
   * i.e. all core services are in place, but no commands are serviced for this module yet.
   * Create any resources that this modules needs and add these to the resource pool
   * to be integrated in the global variable pool and method sharing.
   * Remember to add to resource pool by a call to addResource(UResBase * newResource, this).
   * This module must also ensure that any resources creted are deleted (in the destructor of this class).
   * Resource shut-down code should be handled in the resource destructor.
   * \return true if any resources are created. */
  virtual void createResources();
  /**
  Handle incomming command
  (intended for command separation)
  Must return true if the function is handled -
  otherwise the client will get a failed - reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);

protected:
  /**
  line pointer */
  UResCron * cron;
};


#endif
