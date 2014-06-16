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
#ifndef UFUNCPOLY_H
#define UFUNCPOLY_H

#include <cstdlib>

#include <urob4/ufuncplugbase.h>
#include <urob4/userverport.h>

#include "urespoly.h"

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
 * The command interface of a job scheduler that is asunchronous to the the main command queue, and thus long timeconsuming jobs are holding the main command thread.
 * It adds though one additional thread only, and commands are run as pipes.
@author Christian Andersen
*/
class UFuncPoly : public UFuncPlugBase
{ // NAMING convention recommend that the main plugin function class
  // starts with UFunction (as in UFunctionLine) followed by
  // a descriptive extension for this specific plugin
public:
  /**
  Constructor */
  UFuncPoly()
  { // command list and version text
    setCommand("poly", "polygons", "storage of polygons, polylines and points");
    poly = NULL;
  }
  /**
  Destructor */
  virtual ~UFuncPoly();
  /**
   * Called by server after this module is integrated into the server core structure,
   * i.e. all core services are in place, but no commands are serviced for this module yet.
   * Create any resources that this modules needs and add these to the resource pool
   * to be integrated in the global variable pool and method sharing.
   * Remember to add to resource pool by a call to addResource(UResBase * newResource, this).
   * This module must also ensure that any resources creted are deleted (in the destructor of this class).
   * Resource shut-down code should be handled in the resource destructor.*/
  virtual void createResources();
  /**
   * Handle incomming commands flagged for this module
   * \param msg pointer to the message and the client issuing the command
   * \return true if the function is handled - otherwise the client will get a 'failed' reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);

protected:
  /**
  line pointer */
  UResPoly * poly;
  /** last update for each client */
  UTime updateTimes[MAX_SOCKET_CLIENTS_SERVED + 20];
};


#endif

