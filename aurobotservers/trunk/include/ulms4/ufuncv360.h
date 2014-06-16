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
#ifndef UFUNCV360_H
#define UFUNCV360_H

#include <urob4/ufuncplugbase.h>

#include "uresv360.h"


/**
Interface function for virtual 360 deg laser scanner

@author Christian Andersen
*/
class UFuncV360 : public UFuncPlugBase
{
public:
  /**
  Constructor */
  UFuncV360();
  /**
  DEstructor */
  ~UFuncV360();
  /**
   * Called by server after this module is integrated into the server core structure,
   * i.e. all core services are in place, but no commands are serviced for this module yet.
   * Create any resources that this modules needs and add these to the resource pool
   * to be integrated in the global variable pool and method sharing.
   * Remember to add to resource pool by a call to addResource(UResBase * newResource, this).
   * This module must also ensure that any resources creted are deleted (in the destructor of this class).
   * Resource shut-down code should be handled in the resource destructor. */
  virtual void createResources();
  /**
  Handle command from client (or console).
  Return true if command is handled and all needed
  actions are taken.
  Return false if command is not known (due
  to syntax error) or if it belongs to another
  function) */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);

protected:
  /**
  Handle commands for the virtual 360 deg laserscanner */
  bool handleV360Command(UServerInMsg * msg, ULaserData * data);
  /**
  Handle commands for the virtual 360 deg laserscanner */
  bool handleV360Push(UServerInMsg * msg);

protected:
  /**
  Pointer to the virtual laser scanner ressource */
  UResV360 * v360;
};

#endif
